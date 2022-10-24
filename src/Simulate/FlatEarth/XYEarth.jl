using DifferentialEquations
using StaticArrays
using LinearAlgebra
using Geophysics
using Plots
using Formatting

P₀ = pressure(0)


function rhs!(du, u, p, t)
    # states
    v₁ = u[1]
    v₂ = u[2]
    h = u[4]

    # intermediates
    v = √(v₁^2 + v₂^2)
    a = (drag(h, v, t) + thrust(h, t)) / mass(t)

    # assignment
    du[1] = v₁/v * a
    du[2] = v₂/v * a - gravity(h)
    du[3] = v₁
    du[4] = v₂
    nothing
end


function simulate(θ, v₀, tf=1000.)
    condition1(u, t, i) = u[4]
    affect1!(i) = terminate!(i)
    cb1 = ContinuousCallback(condition1, affect1!)
    condition2(u, t, i) = u[2]
    cb2 = ContinuousCallback(condition2, nothing)

    τ = (0.0, tf)
    u₀ = [sind(θ)*v₀, cosd(θ)*v₀, 0., 1., ]
    prob = ODEProblem(rhs!, u₀, τ)
    @time sol = solve(prob, callback=CallbackSet(cb1, cb2))
    sol
end


function plot_sol(sol)
    tf = sol.t[end]
    println("tf:: ", tf)
    println("maxh:", format(max(hcat(sol.u...)[4,:]...)))
    println("finx:", format(sol.u[end][3]))
    overall = plot(sol, idxs=[(0, 4), (0, 3)], tspan=(0, tf), labels=["y" "x"], formatter=:plain)
    initial = plot(sol, idxs=[(0, 4), (0, 3)], tspan=(0, 10), labels=["y" "x"], formatter=:plain)
    overall_v = plot(sol, idxs=[(0, 2), (0, 1)], tspan=(0, tf), labels=["ẏ" "ẋ"], formatter=:plain)
    initial_v = plot(sol, idxs=[(0, 2), (0, 1)], tspan=(0, 10), labels=["ẏ" "ẋ"], formatter=:plain)
    plot(overall, initial, overall_v, initial_v)
end
