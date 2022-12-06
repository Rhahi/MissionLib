using SpaceLib
using LinearAlgebra
using DifferentialEquations
using Unitful.DefaultSymbols
using Formatting
using Plots
import KerbalGuidance.CelestialBody: Earth

function model(time)
    time, s1 = KerbalGuidance.NextStage(time;
        ṁ=(62.08+80.0 +2.059)kg/s,
        m₀=14778kg,
        m₁=4478kg,
        duration=70s,
        vac=333.0kN,
        asl=282.8kN,
        area=2.93666m^2,
        Cd=0.114
    )
    time, s2 = KerbalGuidance.NextStage(time;
        ṁ=(4.961+15.23+0.2104)kg/s,
        m₀=1690kg,
        m₁=719kg,
        duration=45s,
        vac=49.3kN,
        asl=39.2kN,
        area=0.669207m^2,
        Cd=0.114
    )
    KerbalGuidance.HotStageRocket([s1, s2])
end

function simulate(u₀, body, rocket)
    τ = [0., 1000.]
    p = [body, rocket]
    prob = ODEProblem(KerbalGuidance.bci_rhs!, u₀, τ, p)
    condition1(u, t, i) = norm(view(u, 4:6)) - 6370e+3
    condition2(u, t, i) = view(u, 1:3)⋅view(u, 4:6)
    cb1 = ContinuousCallback(condition1, terminate!)
    cb2 = ContinuousCallback(condition2, nothing)
    solve(prob, callback=CallbackSet(cb1, cb2))
end

function plot_sol(u₀, sol)
    tf = sol.t[end]
    println("tf:: ", tf)
    println("finx:", format(sol.u[end][3]))
    traj = reduce(hcat, sol.u)'
    v = view(traj, :, 1:3)
    r = view(traj, :, 4:6)
    r₀ = u₀[4:6]
    velocity = norm.(eachrow(v))
    position = norm.(eachrow(r))
    downrange = norm(r₀)*acos.(min.(1, fixed_dot(r₀).(eachrow(r))./position/norm(r₀)))
    vvel = dot.(eachrow(v), eachrow(r)./norm.(eachrow(r)))
    hvel = pythagoras.(velocity, vvel)

    plot_vel = plot(sol.t, [velocity, vvel, hvel];
        tspan=(0, tf),
        labels=["velocity" "vvel" "hvel"], formatter=:plain, legend=:bottomleft,
        xlabel="time (s)", ylabel="speed (m/s)")
    plot_pos = plot(sol.t, [(position.-norm(r₀))/1000, downrange/1000];
        tspan=(0, tf), title="↑$(convert(Int, round((max(hcat(sol.u...)[4,:]...))-6371e+3))) km →$(convert(Int64, round(downrange[end]/1000)))km",
        labels=["altitude" "downrange"], formatter=:plain, legend=:topleft,
        xlabel="time (s)", ylabel="distance (km)")
    plot(plot_vel, plot_pos)
end

pythagoras(c, a) = sqrt(c^2 - a^2)
fixed_dot(a) = x->a⋅x

function evaluate(solution, target)
    traj = reduce(hcat, solution.u)'
    r = view(traj, :, 4:6)
    return maximum(norm.(eachrow(r))) - target, solution
end

function solver(u₀, body, rocket, tf)
    time_range = [0., F64(tf)]
    p = [body, rocket]
    prob = ODEProblem(KerbalGuidance.bci_rhs!, u₀, time_range, p)
    condition1(u, t, i) = norm(view(u, 4:6)) - 6370e+3
    condition2(u, t, i) = view(u, 1:3)⋅view(u, 4:6)
    cb1 = ContinuousCallback(condition1, terminate!)
    cb2 = ContinuousCallback(condition2, terminate!)
    solve(prob, callback=CallbackSet(cb1, cb2))
end

function make_path()
    ϕ = 0°
    r = [908640.5359560507, 3.0447910832054615e6, -5.522112283919509e6]
    v = [0.00027219847987680623, 0.0009130111675403896, -0.001671877356557161]
    d = [0.1425565756577733, 0.47789403796195984, -0.8667727510890744]
    t = 0.
    θ = clamp(dot(r, d) / norm(r) / norm(d), -1, 1) |> acos |> rad2deg
    @show θ
    u_gen = KerbalGuidance.u_generator(v, r, d, ϕ)
    sol, u = KerbalGuidance.binary_search(solver, u_gen, evaluate;
        body=Earth,
        rocket=model(t),
        target=150e+3+Earth.radius,
        input_tolerance=0.0001°,
        output_tolerance=15e+3,
        var_min=max(0, θ-5)°,
        var_max=max(0, θ+5)°,
        t₀=t,
        t₁=1000.
    )
    t, sol, u
end

_, sol, u = make_path()
plot!(plot_sol(u, sol))
score, _ = evaluate(sol, 150e+3+Earth.radius)
@show u
