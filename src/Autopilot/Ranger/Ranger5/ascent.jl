struct Controls
    engage::Channel{Bool}
    thrust::Channel{Float32}
    roll::Channel{Float32}
    direction::Channel{NTuple{3, Float64}}
    guidance::Channel{Symbol}
end

function ascent(sp; trigger, next)
    s2 = Condition()
    s3 = Condition()
    c1 = Condition()
    ctrl = ascent_control(sp)
    @sync begin
        @asyncx stage1(sp, ctrl; trigger=nothing, next=c1)
        @asyncx coast1(sp, ctrl; trigger=c1, next=s2)
        @asyncx stage2(sp, ctrl; trigger=s2, next=s3)
        @asyncx stage3(sp, ctrl; trigger=s3, next=nothing)
    end
    @log_mark "Mission complete."
end

function ascent_control(sp::Spacecraft)
    ref = Navigation.ReferenceFrame.BCBF(sp)
    control = SCH.Control(sp.ves)
    ap = SCH.AutoPilot(sp.ves)
    SCH.TimeToPeak!(ap, (2.0, 2.0, 2.0))  # used to be (3 3 3)
    SCH.Overshoot!(ap, (0.05, 0.05, 0.05))  # used to be (0.1)
    SCH.ReferenceFrame!(ap, ref)

    ctrl_guidance = Channel{Symbol}(1)
    src_engage = Channel{Bool}(1)
    src_thrust = Channel{Float32}(1)
    src_roll = Channel{Float32}(1)
    src_solution = ascent_guidance(sp, ctrl_guidance)

    sink_direction = Control.filter_solution_to_direction(sp, src_solution; period=0.04)
    # sink_direction = Control.filter_vector_limit(sp, sol_direction; degrees_per_second=1)

    Control.sink_engage(ap, src_engage)
    Control.sink_thrust(control, src_thrust)
    Control.sink_roll(ap, src_roll)
    Control.sink_direction(sp, ap, ref, sink_direction; line_length=40, line_color=:cyan)
    return Controls(src_engage, src_thrust, src_roll, sink_direction, ctrl_guidance)
end

function stage1(sp::Spacecraft, ctrl::Controls; trigger, next)
    engine = SCH.Engine(sp.parts[:e1])
    control = SCH.Control(sp.ves)
    flight = SCH.Flight(sp.ves, Navigation.ReferenceFrame.BCBF(sp))
    ṁ = 85.39+192.2

    # start guidance
    put!(ctrl.guidance, :init)
    wait(ctrl.direction)

    # ignite engine
    Modules.Engine.ignite!(sp, engine; target_thrust=SCH.Mass(sp.ves)*9.80665*1.5)
    put!(ctrl.engage, true)
    put!(ctrl.thrust, 1)
    Control.stage!(sp)
    Timing.delay(sp, 1)

    # queue fairing separation
    @asyncx begin
        Timing.wait_for_value(sp; timeout=120, name="Fairings") do
            30000 - Navigation.altitude(flight)
        end
        SCH.ToggleActionGroup(control, convert(UInt32, 10))
    end

    # wait for next stage
    Timing.wait_for_value(sp; timeout=135, name="Stage1", anticipate=:half) do
        Modules.Engine.burn_time(sp, engine, ṁ) - 4.5
    end

    notify(next)
end

function coast1(sp, ctrl; trigger, next)
    wait(trigger)

    Timing.delay(sp, 80, "Coasting")

    notify(next)
end

function stage2(sp::Spacecraft, ctrl::Controls; trigger, next)
    engine = SCH.Engine(sp.parts[:e2])
    ṁ = 2.025+7.695+0.3281
    control = SCH.Control(sp.ves)
    wait(trigger)

    # separation and ullage
    SCH.ToggleActionGroup(control, convert(UInt32, 11))
    SCH.RCS!(control, true)
    Timing.delay(sp, 8, "Separation")
    SCH.Forward!(control, F32(0.9))
    Timing.delay(sp, 2, "Ullage")
    SCH.Active!(engine, true)
    Timing.delay(sp, 1)
    SCH.Forward!(control, F32(0))
    SCH.RCS!(control, false)
    Timing.wait_for_value(sp; timeout=121, period=1, name="Stage2", anticipate=:half) do
        Modules.Engine.burn_time(sp, engine, ṁ)
    end

    notify(next)
end

function stage3(sp::Spacecraft, ctrl::Controls; trigger, next)
    engine = SCH.Engine(sp.parts[:e3])
    ṁ = 2.025+7.695+0.3281
    control = SCH.Control(sp.ves)
    wait(trigger)

    # stabilize vessel direction
    SCH.RCS!(control, true)
    Timing.delay(sp, 5, "Stabilize")
    SCH.ToggleActionGroup(control, convert(UInt32, 20))

    # start ullage
    SCH.Forward!(control, F32(1))
    Timing.delay(sp, 15, "Ullage")

    # ignition
    SCH.Active!(engine, true)
    Timing.delay(sp, 1)
    SCH.ToggleActionGroup(control, convert(UInt32, 21)) # separate
    SCH.Forward!(control, F32(0))

    Timing.wait_for_value(sp; timeout=62, period=1, name="Stage3", anticipate=:half) do
        Modules.Engine.burn_time(sp, engine, ṁ)
    end
    @log_mark "new stage activated"
end
