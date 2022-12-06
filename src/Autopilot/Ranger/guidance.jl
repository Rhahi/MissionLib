# function model(time)
#     time, s1 = KerbalGuidance.NextStage(time;
#         massflow = ((62.08+80.0 +2.059)kg/s),
#         m₀       = (14778kg),
#         m₁       = (4478kg),
#         duration = (70s),
#         vac      = (333.0kN),
#         asl      = (282.8kN),
#         area     = (2.93666m^2),
#         Cd       = (0.114)
#     )
#     time, s2 = KerbalGuidance.NextStage(time;
#         massflow = (4.961+15.23+0.2104)kg/s,
#         m₀       = 1690kg,
#         m₁       = 719kg,
#         duration = 45s,
#         vac      = 49.3kN,
#         asl      = 39.2kN,
#         area     = 0.669207m^2,
#         Cd       = 0.114
#     )
#     KerbalGuidance.HotStageRocket([s1, s2])
# end
# function model(time)
#     time, s1 = KerbalGuidance.NextStage(time;
#         massflow = ((62.08+80.0+2.059)kg/s),
#         m₀       = (15694kg),
#         m₁       = (5080kg),
#         duration = (70s),
#         vac      = (333.0kN),
#         asl      = (282.8kN),
#         area     = (2.93666m^2),
#         Cd       = (0.114)
#     )
#     time, s2 = KerbalGuidance.NextStage(time;
#         massflow = (4.499+14.76+0.2062)kg/s,
#         m₀       = 2990kg,
#         m₁       = 2006kg,
#         duration = 49s,
#         vac      = 49.3kN,
#         asl      = 39.3kN,
#         area     = 0.669207m^2,
#         Cd       = 0.114
#     )
#     KerbalGuidance.HotStageRocket([s1, s2])
# end
function model(time)
    time, s1 = KerbalGuidance.NextStage(time;
        massflow = ((62.08+80.0+2.059)kg/s),
        m₀       = (15837kg),
        m₁       = (5304kg),
        duration = (70s),
        vac      = (333.0kN),
        asl      = (282.8kN),
        area     = (2.93666m^2),
        Cd       = (0.114)
    )
    time, s2 = KerbalGuidance.NextStage(time;
        massflow = (4.499+14.76+0.2062)kg/s,
        m₀       = 3216kg,
        m₁       = 2231kg,
        duration = 49s,
        vac      = 49.3kN,
        asl      = 39.3kN,
        area     = 0.669207m^2,
        Cd       = 0.114
    )
    time, s3 = KerbalGuidance.NextStage(time;
        massflow = (1.674+4.286+0.1783)kg/s,
        m₀       = 1701kg,
        m₁       = 1425kg,
        duration = 40s,
        vac      = 13.8kN,
        asl      = 11.7kN,
        area     = 0.669207m^2,
        Cd       = 0.114
    )
    KerbalGuidance.HotStageRocket([s1, s2, s3])
end

function make_path(sp::Spacecraft, ref::SCR.ReferenceFrame; show=false, offset::Real=0, target_alt=160e+3)
    @log_guidance "making new path"
    ϕ = 90
    search_range = 20
    r = T2V(Navigation.coordinate(sp, ref))
    v = T2V(Navigation.velocity(sp, ref))
    d = T2V(Navigation.direction(sp, ref))
    θ = clamp(dot(r, d) / norm(r) / norm(d), -1, 1) |> acos |> rad2deg
    t₀ = sp.system.met
    u_gen = KerbalGuidance.u_generatorₗ(v, r, d, ϕ)
    success, sol, u = KerbalGuidance.binary_search(solver, u_gen, evaluate;
        body=CelestialBody.Earth,
        rocket=model(t₀-offset),
        target=target_alt+CelestialBody.Earth.radius,
        input_tolerance=0.0001,
        output_tolerance=20e+3,
        var_min=max(0, θ-search_range),
        var_max=max(0, θ+search_range),
        t₀=t₀,
        t₁=sp.system.met+1000
    )
    show && plot_sol(u, sol, rad2deg(∠θ(view(u, 1:3), r)))
    return success ? sol : nothing
end

function source_path(sp::Spacecraft, ref::SCR.ReferenceFrame; initial_solution::Any, offset::Real)
    @log_entry "source_path"
    source = Channel{Any}(1)
    isnothing(initial_solution) || put!(source, initial_solution)
    @asyncx begin
        @log_dev "begin source path"
        Telemetry.ut_periodic_stream(sp, 5) do stream
            try
                for _ in stream
                    sol = make_path(sp, ref; show=true, offset=offset)
                    put!(source, sol)
                    yield()
                end
            catch e
                if !isa(e, InvalidStateException)
                    @warn "Unexpected error during source path loop" e
                end
            end
        end
    end
    @log_exit "source_path"
    return source
end

function setup_guidance(sp::Spacecraft, initial_solution::Any=nothing)
    ref = Navigation.ReferenceFrame.BCBF(sp)

    @log_dev "waiting for guidance start..."
    offset = sp.system.met
    control, ap = Control.init_autopilot(sp, ref)

    # sources
    @log_status "setup sources"
    src_engage = Channel{Bool}(1)
    src_thrust = Channel{Float32}(1)
    src_path = source_path(sp, ref; initial_solution=initial_solution, offset=offset)

    # filters
    @log_status "setup filters"
    snk_dir1 = Control.filter_direction(sp, src_path; period=0.05, offset=offset)
    snk_dir2 = Control.filter_vector_limit(sp, snk_dir1; degrees_per_second=1)

    # sinks
    @log_status "setup sinks"
    Control.sink_engage(ap, src_engage)
    Control.sink_thrust(control, src_thrust)
    Control.sink_direction(sp, ap, ref, snk_dir2; line_length=40, line_color=:cyan)

    @asyncx begin
        wait(sp.events[:guidance])
        @log_mark "first command issued"
        put!(src_engage, true)
        put!(src_thrust, F32(1))
    end
    @log_exit "setup_guidance"
    return src_engage, src_thrust, src_path
end

function solver(u₀, body, rocket, t₀, t₁)
    time_range = [t₀, t₁]
    p = [body, rocket, t₀]
    prob = ODEProblem(KerbalGuidance.nonrotating_rhs!, u₀, time_range, p)
    condition1(u, t, i) = norm(view(u, 4:6)) - 6370e+3
    condition2(u, t, i) = view(u, 1:3)⋅view(u, 4:6)
    cb1 = ContinuousCallback(condition1, terminate!)
    cb2 = ContinuousCallback(condition2, nothing)
    solve(prob, callback=CallbackSet(cb1, cb2))
end

function evaluate(solution, target)
    traj = reduce(hcat, solution.u)'
    r = view(traj, :, 4:6)
    return maximum(norm.(eachrow(r))) - target, solution
end

pythagoras(c, a) = sqrt(c^2 - a^2)
fixed_dot(a) = x->a⋅x

function plot_sol(u₀, sol, θ)
    traj = reduce(hcat, sol.u)'
    v = view(traj, :, 1:3)
    r = view(traj, :, 4:6)
    r₀ = u₀[4:6]
    velocity = norm.(eachrow(v))
    position = norm.(eachrow(r))
    downrange = norm(r₀)*acos.(min.(1, fixed_dot(r₀).(eachrow(r))./position/norm(r₀)))
    vvel = dot.(eachrow(v), eachrow(r)./norm.(eachrow(r)))
    hvel = pythagoras.(velocity, vvel)
    radius = CelestialBody.Earth.radius

    plot_vel = plot(sol.t, [velocity, vvel, hvel];
        tspan=(sol.t[begin], sol.t[end]),
        labels=["velocity" "vvel" "hvel"],
        formatter=:plain,
        legend=:bottomleft,
        xlabel="time (s)",
        ylabel="speed (m/s)"
    )
    plot_pos = plot(sol.t, [(position.-radius)/1000, downrange/1000];
        tspan=(sol.t[begin], sol.t[end]),
        title="↑$(convert(Int, round((maximum(position)-radius)/1000))) km →$(convert(Int64, round(downrange[end]/1000)))km",
        labels=["altitude" "downrange"],
        formatter=:plain,
        legend=:topleft,
        xlabel="time (s)",
        ylabel="distance (km), with angle $(round(θ))"
    )
    plot(plot_vel, plot_pos) |> display
end
