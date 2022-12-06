function ascent_guidance(sp::Spacecraft, input::Channel{Symbol})
    output = Channel{Any}(1)
    @asyncx begin
        time_since_launch = nothing
        try
            while true
                cmd = take!(input)
                cmd == :stop && break
                if cmd == :init
                    time_since_launch = sp.system.met
                    @asyncx generate_stage1_solution(sp, output, time_since_launch)
                end
            end
        catch e
            if !isa(e, InvalidStateException)
                @log_warn "Unexpected error during guidance loop -- $e"
            end
        finally
            close(input)
        end
    end
    return output
end

function generate_stage1_solution(sp::Spacecraft, output::Channel, time_since_launch)
    ϕ = 90
    ref = Navigation.ReferenceFrame.BCBF(sp)
    r = T2V(Navigation.coordinate(sp, ref))
    v = T2V(Navigation.velocity(sp, ref))
    d = T2V(Navigation.direction(sp, ref))
    θ = ∠θ(r, d) |> rad2deg

    search_range = 10
    u_gen = KerbalGuidance.u_generatorₗ(v, r, d, ϕ)
    success, sol, u = KerbalGuidance.binary_search(solver, u_gen, evaluate;
        body=CelestialBody.Earth,
        rocket=model_stage1(time_since_launch),
        target=CelestialBody.Earth.radius+200e+3,
        search_resolution=0.00001,
        acceptable_error_low=-20e+3,
        acceptable_error_high=40e+3,
        θ_min=max(0, θ-search_range/2),
        θ_max=max(0, θ+search_range/2),
        t₀=sp.system.met,
        t₁=sp.system.met+1000,
        returnable_error=100
    )
    @log_dev string(success)
    if !isnothing(sol)
        plot_sol(u, sol)
    else
        @log_dev "plot not available"
    end
    if success
        put!(output, sol)
    end
end

pythagoras(a, b) = sqrt(max(a, b)^2 - min(a, b)^2)
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
        ylabel="distance (km)"
    )
    plot_angle = plot(sol.t, [∠θ.(eachrow(v), eachrow(r)) .|> rad2deg];
        tspan=(sol.t[begin], sol.t[end]),
        labels=["angle from vertical"],
        formatter=:plain,
        legend=:topleft,
        xlabel="time (s)",
        ylabel="angle (degrees)"
    )
    plot(plot_vel, plot_pos, plot_angle) |> display
end

function model_stage1(time)
    time, s1 = KerbalGuidance.NextStage(time;
        # LR43-NA-3
        massflow = (85.39+192.2)kg/s,
        m₀       = 44295kg,
        m₁       = 6994kg,
        duration = 135s,
        vac      = 756.8kN,
        asl      = 667kN,
        area     = 2.7612m^2,
        Cd       = 0.114
    )
    return KerbalGuidance.HotStageRocket([s1])
end

function solver(u₀, body, rocket, t₀, t₁)
    time_range = [t₀, t₁]
    p = [body, rocket, t₀]
    prob = ODEProblem(KerbalGuidance.nonrotating_rhs!, u₀, time_range, p)
    condition1(u, t, i) = norm(view(u, 4:6)) - 6370e+3
    condition2(u, t, i) = view(u, 1:3)⋅view(u, 4:6)
    crash = ContinuousCallback(condition1, terminate!)
    horizontal = ContinuousCallback(condition2, terminate!)
    solve(prob, callback=CallbackSet(crash, horizontal))
end

function evaluate(solution, target)
    traj = reduce(hcat, solution.u)'
    r = view(traj, :, 4:6)
    return maximum(norm.(eachrow(r))) - target, solution
end
