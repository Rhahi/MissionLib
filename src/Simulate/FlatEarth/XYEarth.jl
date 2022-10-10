using DifferentialEquations
using StaticArrays
using LinearAlgebra
using Geophysics
using Plots

P₀ = pressure(0)


function rhs!(du, u, p, t)
    # states
    v₁ = u[1]
    v₂ = u[2]
    h = u[4]

    # intermediates
    v = √(v₁^2 + v₂^2)
    a = (drag(h, v) + thrust(h, t)) / mass(t)

    # assignment
    du[1] = v₁/v * a
    du[2] = v₂/v * a - gravity(h)
    du[3] = v₁
    du[4] = v₂
    nothing
end


function drag(h, v)
    d = 0.04  # drag coefficient
    A = 0.155  # area, m²
    v² = v⋅v
    0.5 * pressure(h) / P₀ * v² * d * A
end


function thrust(h, t)
    if t > 80
        return 0.
    end
    vac = 13.8e+3  # N
    asl = 11.7e+3  # N
    vac - (vac - asl) * pressure(h) / P₀  # N
end


function mass(t)
    m₀ = 468.  # stage of stage 1
    m₁ = 215.  # end of stage 1
    mfr = 1.674 + 4.286 + 0.1783
    expended_fuel = mfr * t
    max(m₁, m₀ - expended_fuel)
end


θ = 10.
v₀ = 150.
τ = (0.0, 300.)
u₀ = [sind(θ)*v₀, cosd(θ)*v₀, 0., 0., ]

println("problem defined")
prob = ODEProblem(rhs!, u₀, τ)
println("begin solving")
@time sol = solve(prob)
println("maxh:", max(hcat(sol.u...)[4,:]...))
println("finx:", sol.u[end][3])
overall = plot(sol, idxs=[(0, 4), (0, 3)], tspan=τ)
initial = plot(sol, idxs=[(0, 4), (0, 3)], tspan=(0, 10))
overall_v = plot(sol, idxs=[(0, 2), (0, 1)], tspan=τ)
initial_v = plot(sol, idxs=[(0, 2), (0, 1)], tspan=(0, 10))
display(plot(overall, initial, overall_v, initial_v))
