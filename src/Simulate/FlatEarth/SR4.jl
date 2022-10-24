include("XYEarth.jl")


struct Stage
    m₀::Float64
    m₁::Float64
    ṁ::Float64
    τ::Float64
    vac::Float64
    asl::Float64
    A::Float64
end
s1 = Stage(14778., 4478., 62.08 +80.0 +2.059,  70., 333.0e+3,  282.8e+3, 2.93666)
s2 = Stage( 1690.,  719.,  4.961+15.23+0.2104, 45.,  49.3e+3,   39.2e+3, 0.669207)


function mass(t)
    t < s1.τ && return max(s1.m₁, s1.m₀ - s1.ṁ * t)
    max(s2.m₁, s2.m₀ - s2.ṁ * t-s1.τ)
end


function thrust(h, t)
    t < s1.τ && return s1.vac - (s1.vac - s1.asl) * pressure(h) / P₀
    s2.vac - (s2.vac - s2.asl) * pressure(h) / P₀
end


function drag(h, v, t)
    d = 0.114  # drag coefficient
    A = s2.A
    if t < s1.τ
        A = s1.A
    end
    v² = v⋅v
    0.5 * pressure(h) / P₀ * v² * d * A
end

θ = 1.
sol = simulate(θ, 10., 1000.)
maxh = format(round(max(hcat(sol.u...)[4, :]...)), commas=true)
plot!(plot_sol(sol), title=string(θ, "°, ", maxh))
