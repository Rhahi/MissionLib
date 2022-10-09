using LoggingExtras
using SpaceLib
using SpaceLib.Timing
using SpaceLib.Control
import KRPC.Interface.SpaceCenter.Helpers as SCH
import KRPC.Interface.SpaceCenter as SC


function stage0(sp::Spacecraft)
    # release clamps and ignite boosters
    @info "Ignition"
    stage(sp)
    delay(sp, 0.5)
    notify(sp.events[:stage1])
end


function stage1(sp::Spacecraft)
    @infov 1 "stage1 standby"
    e1 = SCH.Engine(sp.parts[:e1])
    wait(sp.events[:stage1])

    SCH.Active!(e1, true)
    delay(sp, 0.2)
    stage(sp)

    delay(sp, 38.5, log="stage1")
    notify(sp.events[:stage2])
end


function stage2(sp::Spacecraft)
    e2 = SCH.Engine(sp.parts[:e2])
    @infov 1 "stage2 standby"
    wait(sp.events[:stage2])

    SCH.Active!(e2, true)
    delay(sp, 0.5)
    stage(sp)
    delay(sp, 38.5, log="stage2")
    notify(sp.events[:deploy])
end


function deploy(sp::Spacecraft)
    wait(sp.events[:deploy])
    Timing.delay_on_bedrock_altitude(sp, 140e+3, 120, "chute")
    SCH.Arm(SCH.Parachute(sp.parts[:chute]))
    stage(sp)
end
