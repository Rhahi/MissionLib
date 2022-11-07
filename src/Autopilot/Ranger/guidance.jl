function model(time)
    time, s1 = KerbalGuidance.NextStage(time;
        massflow = ((62.08+80.0 +2.059)kg/s),
        m₀       = (14778kg),
        m₁       = (4478kg),
        duration = (70s),
        vac      = (333.0kN),
        asl      = (282.8kN),
        area     = (2.93666m^2),
        Cd       = (0.114)
    )
    time, s2 = KerbalGuidance.NextStage(time;
        massflow = (4.961+15.23+0.2104)kg/s,
        m₀       = 1690kg,
        m₁       = 719kg,
        duration = 45s,
        vac      = 49.3kN,
        asl      = 39.2kN,
        area     = 0.669207m^2,
        Cd       = 0.114
    )
    KerbalGuidance.HotStageRocket([s1, s2])
end

function make_path(sp::Spacecraft, ref; show=false)
    @log_guidance "making new path"
    ϕ = 90°
    r = T2V(Navigation.coordinate(sp, ref))
    v = T2V(Navigation.velocity(sp, ref))
    d = T2V(Navigation.direction(sp, ref))
    θ = clamp(dot(r, d) / norm(r) / norm(d), -1, 1) |> acos |> rad2deg
    t₀ = sp.system.met
    u_gen = KerbalGuidance.u_generator(v, r, d, ϕ)
    success, sol, u = KerbalGuidance.binary_search(solver, u_gen, evaluate;
        body=CelestialBody.Earth,
        rocket=model(offset),
        target=150e+3+CelestialBody.Earth.radius,
        input_tolerance=0.0001°,
        output_tolerance=15e+3,
        var_min=max(0, θ-5)°,
        var_max=max(0, θ+5)°,
        t₀=t₀,
        t₁=sp.system.met+1000.
    )
    show && plot_sol(u, sol)
    return success ? sol : nothing
end

function source_path(sp::Spacecraft, ref, initial_solution)
    @log_entry "source_path"
    source = Channel{Any}(1)
    isnothing(initial_solution) || put!(path, initial_solution)
    @asyncx begin
        @log_dev "begin source path"
        Telemetry.ut_periodic_stream(sp, 5) do
            sol = make_path(sp, ref)
            put!(source, sol)
            yield()
        end
    end
    @log_exit "source_path"
    return source
end

function setup_guidance(sp::Spacecraft, initial_solution=nothing)
    ref = Navigation.ReferenceFrame.BCBF(sp)

    @log_dev "waiting for guidance start..."
    wait(sp.events[:guidance])
    offset = sp.system.met
    control, ap = Control.init_autopilot(sp, ref)
    @asyncx update_path!(sp, path)

    # sources
    src_engage = Channel{Bool}(1)
    src_thrust = Channel{Float32}(1)
    src_path = source_path(sp, ref, initial_solution)
    src_spin = Channel{Float64}(1)

    # filters
    snk_dir = Control.filter_direction(sp, src_path; period=0.05, offset=offset)
    snk_roll = Control.filter_spin(sp, ref, src_spin; period=1)

    # sinks
    Control.sink_engage(ap, src_engage)
    Control.sink_thrust(control, src_thrust)
    Control.sink_direction(sp, ap, ref, snk_dir)
    Control.sink_roll(ap, snk_roll)
end

function solver(u₀, body, rocket, t₀, t₁)
    time_range = [t₀, t₁]
    p = [body, rocket, t₀]
    prob = ODEProblem(KerbalGuidance.nonrotating_rhs!, u₀, time_range, p)
    condition1(u, t, i) = norm(view(u, 4:6)) - 6370e+3
    condition2(u, t, i) = view(u, 1:3)⋅view(u, 4:6)
    cb1 = ContinuousCallback(condition1, terminate!)
    cb2 = ContinuousCallback(condition2, terminate!)
    solve(prob, callback=CallbackSet(cb1, cb2))
end

function evaluate(solution, target)
    traj = reduce(hcat, solution.u)'
    r = view(traj, :, 4:6)
    return maximum(norm.(eachrow(r))) - target, solution
end

pythagoras(c, a) = sqrt(c^2 - a^2)
fixed_dot(a) = x->a⋅x

function plot_sol(u₀, sol)
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
        tspan=(sol.t[begin], sol.t[end]),
        labels=["velocity" "vvel" "hvel"],
        formatter=:plain,
        legend=:bottomleft,
        xlabel="time (s)",
        ylabel="speed (m/s)"
    )
    plot_pos = plot(sol.t, [(position.-norm(r₀))/1000, downrange/1000];
        tspan=(sol.t[begin], sol.t[end]),
        title="↑$(convert(Int, round((max(hcat(sol.u...)[4,:]...))-6371e+3))) km →$(convert(Int64, round(downrange[end]/1000)))km",
        labels=["altitude" "downrange"],
        formatter=:plain,
        legend=:topleft,
        xlabel="time (s)",
        ylabel="distance (km)"
    )
    plot(plot_vel, plot_pos) |> display
end
