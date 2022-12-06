function stage1_guidance(ctrl::ControlInterface)
    resp = Channel{Any}(1)
    put!(ctrl.direction, r)
    sol = query!(ctrl.guidance, :lower)
    sol_direction = Control.filter_solution_to_direction(sp, resp; period=0.04)
    Control.sink_injector(ctrl.direction, sol_direction)
    @asyncx begin
        wait(next)
        close(sol_direction)
        @log_dev "closed sol_direction"
    end
end

function stage1(sp::Spacecraft, ctrl::Controls; trigger, next)
    @log_dev "waiting..."
    wait(trigger)
    @log_dev "notified..."
    engine = SCH.Engine(sp.parts[:e1])
    control = SCH.Control(sp.ves)
    flight = SCH.Flight(sp.ves, Navigation.ReferenceFrame.BCBF(sp))
    target_thrust = SCH.Mass(sp.ves)*9.80665*1.3
    ṁ = 85.39+192.2
    r = Navigation.coordinate(sp, :BCBF) |> hat

    # start guidance


    # ignite engine
    put!(ctrl.throttle, 1)
    ok = Modules.Engine.ignite!(sp, engine; target_thrust=target_thrust)
    while !ok
        # re-ignite if it fails
        SCH.Active!(engine, false)
        Timing.delay(sp, 0.5)
        @log_attention "Failed to ignite, retrying..."
        ok = Modules.Engine.ignite!(sp, engine; target_thrust=target_thrust)
    end
    put!(ctrl.engage, true)
    Control.stage!(sp)
    Timing.delay(sp, 1)

    # queue fairing separation
    @asyncx begin
        Timing.wait_for_value(sp; timeout=120, name="Fairings") do
            30000 - Navigation.altitude(flight)
        end
        SCH.ToggleActionGroup(control, UInt32(S1_JETTISON_FAIRINGS))
    end

    # watch for loss of thrust
    @asyncx begin
        Telemetry.ut_periodic_stream(sp, 2) do stream
            for _ in stream
                thrust = SCH.Thrust(sp.ves)
                if thrust == 0
                    if Navigation.altitude(sp) < 50000
                        notify(next, :hot)
                    else
                        notify(next)
                    end
                    return
                end
            end
        end
    end

    # wait for next stage
    Timing.wait_for_value(sp; name="Stage1", anticipate=:half) do
        Modules.Engine.burn_time(sp, engine, ṁ) - 2
    end

    notify(next)
end