function ascent_guidance(sp::Spacecraft, input::Channel{Tuple{Symbol, Channel}})
    @asyncx begin
        try
            while true
                cmd, chan = take!(input)
                if cmd == :lower
                    sol = generate_lower_solution(sp, sp.system.met)
                    push!(chan, sol)
                    continue
                end
                if cmd == :coast
                    sol, time = generate_coast_solution(sp)
                    push!(chan, time)
                    push!(chan, sol)
                    continue
                end
                @log_error "unknown guidance command $(str(cmd))"
            end
        catch e
            if !isa(e, InvalidStateException)
                @log_warn "Unexpected error during guidance loop -- $e"
            end
        finally
            close(input)
        end
    end
    nothing
end

function generate_lower_solution(sp::Spacecraft, time_since_launch)
    ϕ, r, v, d, θ = get_state_vectors(sp)
    search_range = 10
    p = [CelestialBody.Earth, model_lower(time_since_launch), 0.]
    u_gen = KerbalGuidance.u_generatorₗ(v, r, d, ϕ)
    success, sol, u = KerbalGuidance.binary_search(solve_lower, u_gen, evaluate_lower;
        target=CelestialBody.Earth.radius+220e+3,
        search_resolution=0.00001,
        acceptable_error_low=-20e+3,
        acceptable_error_high=40e+3,
        θ_min=max(0, θ-search_range/2),
        θ_max=max(0, θ+search_range/2),
        p=p,
        t₀=sp.system.met,
        t₁=sp.system.met+1000,
        returnable_error=100
    )
    if !isnothing(sol)
        plot_sol(u, sol)
    else
        @log_dev "plot not available"
    end
    success && return sol
    @log_attention "Solution unavailable"
    return nothing
end

function generate_coast_solution(sp::Spacecraft)
    highest = -Inf
    solution = nothing
    u₀ = nothing
    coast = 0
    model = model_upper(0)
    ϕ, r, v, d, θ = get_state_vectors(sp)
    t₀ = sp.system.met
    for t in 0:10:200
        p = (CelestialBody.Earth, model, t₀+t)
        score, sol, u = score_trajectory(sp, p, ϕ, r, v, d, θ)
        if score > highest
            highest = score
            solution = sol
            u₀ = u
            coast = t
        end
    end
    plot_sol(u₀, solution)
    return solution, t₀+coast
end

function score_trajectory(sp::Spacecraft, p, ϕ, r, v, d, θ)
    search_range = 10
    target = 10000
    u_gen = KerbalGuidance.u_generatorₗ(v, r, d, ϕ)
    success, sol, u = KerbalGuidance.binary_search(solve_coast, u_gen, evaluate_upper;
        target=target,
        search_resolution=0.1,
        acceptable_error_low=-3000,
        acceptable_error_high=Inf,
        θ_min=max(0, θ-search_range/2),
        θ_max=max(0, θ+search_range/2),
        p=p,
        t₀=sp.system.met,
        t₁=sp.system.met+1000,
        returnable_error=0
    )
    return evaluate_upper(sol, target), sol, u
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

function model_lower(time)
    time, s1 = KerbalGuidance.NextStage(time;
        # LR43-NA-3
        massflow = (85.39+192.2)kg/s,
        m₀       = 43946kg,
        m₁       = 6379kg,
        duration = 135s,
        vac      = 756.8kN,
        asl      = 667kN,
        area     = pi*(1.875m)^2,
        Cd       = 0.114
    )
    return KerbalGuidance.HotStageRocket([s1])
end

function model_upper(time)
    time, s2 = KerbalGuidance.NextStage(time;
        # AJ10-37
        massflow = (3.348+9.37+0.03966)kg/s,
        m₀       = 3225kg,
        m₁       = 1701kg,
        duration = 115s,
        vac      = 33.8kN,
        asl      = 29.9kN,
        area     = pi*(1.25m)^2,
        Cd       = 0.114
    )
    time, s3 = KerbalGuidance.NextStage(time;
        # LR101-NA-11
        massflow = (0.698+1.501)kg/s,
        m₀       = 1313kg,
        m₁       = 439kg,
        duration = 400s,
        vac      = 5.3kN,
        asl      = 4.5kN,
        area     = pi*(1.25m)^2,
        Cd       = 0.114
    )
    return KerbalGuidance.HotStageRocket([s2, s3])
end

function solve_lower(u₀, p, t₀, t₁)
    time_range = [t₀, t₁]
    prob = ODEProblem(KerbalGuidance.nonrotating_rhs!, u₀, time_range, p)
    condition1(u, t, i) = norm(view(u, 4:6)) - 6370e+3
    condition2(u, t, i) = view(u, 1:3)⋅view(u, 4:6)
    crash = ContinuousCallback(condition1, terminate!)
    horizontal = ContinuousCallback(condition2, terminate!)
    solve(prob, callback=CallbackSet(crash, horizontal))
end

function solve_coast(u₀, p, t₀, t₁)
    time_range = [t₀, t₁]
    prob = ODEProblem(KerbalGuidance.nonrotating_rhs!, u₀, time_range, p)
    condition1(u, t, i) = norm(view(u, 4:6)) - 6370e+3
    condition2(u, t, i) = 515 - (t - p[3])
    crash = ContinuousCallback(condition1, terminate!)
    empty = ContinuousCallback(condition2, terminate!)
    solve(prob, callback=CallbackSet(crash, empty))
end

function evaluate_lower(solution, target)
    traj = reduce(hcat, solution.u)'
    r = view(traj, :, 4:6)
    return maximum(norm.(eachrow(r))) - target, solution
end

function evaluate_upper(solution, target)
    t₁ = sol.t[end]
    v = view(solution(t₁), 1:3)
    r = view(solution(t₁), 4:6)
    vᵥ = v⋅hat(r)
    vₕ = pythagoras(v, vᵥ)
    return target-vₕ
end

function get_state_vectors(sp::Spacecraft)
    ref = Navigation.ReferenceFrame.BCBF(sp)
    ϕ = 90
    r = T2V(Navigation.coordinate(sp, ref))
    v = T2V(Navigation.velocity(sp, ref))
    d = T2V(Navigation.direction(sp, ref))
    θ = ∠θ(r, d) |> rad2deg
    return ϕ, r, v, d, θ
end
