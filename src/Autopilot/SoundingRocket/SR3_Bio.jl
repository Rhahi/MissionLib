"""Rocket designed for biological payload contracts"""

using SpaceLib
using SpaceLib.Timing, SpaceLib.Control, SpaceLib.Modules
using Logging, ProgressLogging
import KRPC.Interface.SpaceCenter.Helpers as SCH


function setup(sp::Spacecraft)
    steps = 4
    @withprogress name="Setting up..." begin
        @logprogress 1/steps info="setup parts"
        parts = SCH.Parts(sp.ves)
        sp.parts[:e1] = SCH.WithTag(parts, "e1")[1]
        sp.parts[:e2] = SCH.WithTag(parts, "e2")[1]
        sp.parts[:chute] = SCH.WithTag(parts, "chute")[1]

        @logprogress 2/steps info="setup conditions"
        sp.events[:s1] = Condition()
        sp.events[:s2] = Condition()
        sp.events[:deploy] = Condition()

        @logprogress 3/steps info="setup throttle"
        Control.throttle(sp, 1)
        delay(sp, 1)
        @logprogress 4/steps info="done"
    end
end


function stage0(sp::Spacecraft)
    # stage(sp)
    # delay(sp, 0.7)
    stage(sp)
    delay(sp, 0.6)
    notify(sp.events[:s1])
    @info "notified s1"
end


function stage1(sp::Spacecraft)
    e1 = SCH.Engine(sp.parts[:e1])
    wait(sp.events[:s1])
    @info "begin s1"
    SCH.Active!(e1, true)
    delay(sp, 0.2)
    stage(sp)
    delay(sp, 38.5, "stage1")
    notify(sp.events[:s2])
    @info "notified s2"
end


function stage2(sp::Spacecraft)
    e2 = SCH.Engine(sp.parts[:e2])
    wait(sp.events[:s2])
    @info "begin s2"
    SCH.Active!(e2, true)
    delay(sp, 0.3)
    stage(sp)
    delay(sp, 40, "stage2")
    notify(sp.events[:deploy])
    @info "notified dep"
end


function deploy(sp::Spacecraft)
    chute = sp.parts[:chute]
    wait(sp.events[:deploy])
    delay__bedrock_altitude(sp, "radar"; target=145e+3)
    Parachute.arm(chute)
    stage(sp)
    @info "Giving a few seconds for parachute arming..."
    delay(sp, 5)
end


main("Bio"; log_path=homedir()*"/spacelib/SR", log_level=LogLevel(-1000)) do sp
    setup(sp)
    @sync begin
        @async stage0(sp)
        @async stage1(sp)
        @async stage2(sp)
        @async deploy(sp)
    end
end
