include("XYEarth.jl")


function mass(t)
    m₀ = 468.  # stage of stage 1
    m₁ = 215.  # end of stage 1
    mfr = 1.674 + 4.286 + 0.1783
    expended_fuel = mfr * t
    max(m₁, m₀ - expended_fuel)
end


function thrust(h, t)
    if t > 80
        return 0.
    end
    vac = 13.8e+3  # N
    asl = 11.7e+3  # N
    vac - (vac - asl) * pressure(h) / P₀  # N
end



function drag(h, v)
    d = 0.04  # drag coefficient
    A = 0.155  # area, m²
    v² = v⋅v
    0.5 * pressure(h) / P₀ * v² * d * A
end


@gif for i=1:40
    sol = simulate(i, 150.)
    maxh = round(max(hcat(sol.u...)[4, :]...))
    plot!(plot_sol(sol), title=string(i, "°, ", maxh))
end


