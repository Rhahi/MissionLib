@enum ActionGroup begin
    E1_IGNITE = 1
    E2_IGNITE = 2
    E3_IGNITE = 3
    LAUNCHPAD_UMBILICAL = 10
    LAUNCHPAD_RELEASE = 11
    S1_JETTISON_FAIRINGS = 20
    S1_SEP_E1_CUTOFF = 21
    S2_SEP_E2_CUTOFF = 31
    S3_SEP_S3_RETRO = 40
    PAYLOAD_SCIENCE = 42
end

struct ControlInterface
    engage::Channel{Bool}
    throttle::Channel{Float32}
    roll::Channel{Float32}
    direction::Channel{NTuple{3, Float64}}
    guidance::Channel{Tuple{Symbol, Channel}}
end

struct IgnitionFailureError <: Exception end

function register_ascent_stages(sp, conditions)
    setup_condition(:s1, :s2, :s3, :c1, :c2)
    ctrl = ascent_control(sp)
    @asyncx stage1(sp, ctrl; trigger=s1, next=c1)
    @asyncx coast1(sp, ctrl; trigger=c1, next=s2)
    @asyncx stage2(sp, ctrl; trigger=s2, next=c2)
    @asyncx coast2(sp, ctrl; trigger=c2, next=s3)
    @asyncx stage3(sp, ctrl; trigger=s3, next=d1)
end

function ascent_control(sp::Spacecraft)
    ref = Navigation.ReferenceFrame.BCBF(sp)
    control = SCH.Control(sp.ves)
    ap = SCH.AutoPilot(sp.ves)
    # SCH.TimeToPeak!(ap, (2.0, 2.0, 2.0))  # used to be (3 3 3)
    # SCH.Overshoot!(ap, (0.05, 0.05, 0.05))  # used to be (0.1)
    SCH.ReferenceFrame!(ap, ref)

    ctrl_guidance = Channel{Tuple{Symbol, Channel}}(1)
    src_engage = Channel{Bool}(1)
    src_throttle = Channel{Float32}(1)
    src_roll = Channel{Float32}(1)
    src_direction = Channel{NTuple{3, Float64}}(1)
    ascent_guidance(sp, ctrl_guidance)
    # sink_direction = Control.filter_vector_limit(sp, sol_direction; degrees_per_second=1)

    Control.sink_engage(ap, src_engage)
    Control.sink_throttle(control, src_throttle)
    Control.sink_roll(ap, src_roll)
    Control.sink_direction(sp, ap, ref, src_direction; line_length=40, line_color=:cyan)
    return ControlInterface(src_engage, src_throttle, src_roll, src_direction, ctrl_guidance)
end


function coast1(sp, ctrl; trigger, next)
    entry = wait(trigger)
    @log_dev "start coast"
    if entry ≠ :hot
        resp = Channel(1)
        put!(ctrl.guidance, (:coast, resp))
        time = take!(resp)
        @log_dev "start calculation"
        sol_direction = Control.filter_solution_to_direction(sp, resp; period=0.04)
        @log_dev "calculation done"
        Control.sink_injector(ctrl.direction, sol_direction)
        Timing.delay(sp, time - sp.system.met, "Coasting")
    end
    notify(next, entry)
end

function stage2(sp::Spacecraft, ctrl::ControlInterface; trigger, next)
    entry = wait(trigger)
    engine = SCH.Engine(sp.parts[:e2])
    ṁ = 2.025+7.695+0.3281
    control = SCH.Control(sp.ves)

    try
        if entry == :hot
            stage2_hotstage(sp, control, engine)
        else
            stage2_coldstage(sp, control, engine)
        end
    catch e
        if isa(e, IgnitionFailureError)
            notify(next)
        else
            error(e)
        end
    end

    @asyncx begin
        Telemetry.ut_periodic_stream(sp, 2) do stream
            for _ in stream
                thrust = SCH.Thrust(sp.ves)
                if thrust == 0
                    notify(next)
                    return
                end
            end
        end
    end

    Timing.wait_for_value(sp; timeout=121, period=1, name="Stage2", anticipate=:half) do
        Modules.Engine.burn_time(sp, engine, ṁ)
    end

    notify(next)
end

function stage2_hotstage(sp::Spacecraft, control::ControlInterface, engine)
    # prepare for ignition
    SCH.ToggleActionGroup(control, UInt32(S1_JETTISON_FAIRINGS))
    SCH.RCS!(control, true)
    SCH.Forward!(control, F32(0.8))

    # ignition
    ok = Modules.Engine.ignite!(sp, engine)
    !ok && error(IgnitionFailureError)
    Timing.delay(sp, 1)

    # separation
    SCH.ToggleActionGroup(control, UInt32(S1_SEP_E1_CUTOFF))
    Timing.delay(sp, 1)
    SCH.Forward!(control, F32(0))
    SCH.RCS!(control, false)
    nothing
end

function stage2_coldstage(sp::Spacecraft, control::ControlInterface, engine)
    SCH.ToggleActionGroup(control, UInt32(S1_SEP_E1_CUTOFF))
    SCH.RCS!(control, true)
    Timing.delay(sp, 8, "Separation")
    SCH.Forward!(control, F32(0.9))
    Timing.delay(sp, 2, "Ullage")
    ok = Modules.Engine.ignite!(sp, engine)
    !ok && error(IgnitionFailureError)
    Timing.delay(sp, 1)
    SCH.Forward!(control, F32(0))
    SCH.RCS!(control, false)
    nothing
end


function coast2(sp, ctrl; trigger, next)
    entry = wait(trigger)
    notify(next, entry)
end


function stage3(sp::Spacecraft, ctrl::ControlInterface; trigger, next)
    entry = wait(trigger)
    engine = SCH.Engine(sp.parts[:e3])
    ṁ = 2.025+7.695+0.3281
    control = SCH.Control(sp.ves)

    if entry == :hot
        stage3_hotstage(sp, control)
    else
        stage3_coldstage(sp, control)
    end


    Timing.wait_for_value(sp; timeout=62, period=1, name="Stage3", anticipate=:half) do
        Modules.Engine.burn_time(sp, engine, ṁ)
    end
    @log_mark "new stage activated"
end

function stage3_hotstage(sp::Spacecraft, control::ControlInterface)
    SCH.RCS!(control, true)
    SCH.Forward!(control, F32(1))
    SCH.ToggleActionGroup(control, UInt32(E3_IGNITE))
    Timing.delay(sp, 1)
    SCH.ToggleActionGroup(control, UInt32(S2_SEP_E2_CUTOFF))
    SCH.Forward!(control, F32(0))
    nothing
end

function stage3_coldstage(sp::Spacecraft, control::ControlInterface)
    # stabilize vessel direction
    SCH.RCS!(control, true)
    Timing.delay(sp, 5, "Stabilize")

    # start ullage
    SCH.Forward!(control, F32(1))
    Timing.delay(sp, 5, "Ullage")

    # ignition
    SCH.ToggleActionGroup(control, UInt32(E3_IGNITE))
    Timing.delay(sp, 1)
    SCH.ToggleActionGroup(control, UInt32(S2_SEP_E2_CUTOFF))
    SCH.Forward!(control, F32(0))
    nothing
end

function deploy(sp::Spacecraft, control::ControlInterface; trigger, next)
    @log_dev "deploy"
end
