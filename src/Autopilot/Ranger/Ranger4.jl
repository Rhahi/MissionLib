using SpaceLib
using SpaceLib.Timing
using SpaceLib.Control
using Logging
using ProgressLogging
using Unitful.DefaultSymbols
using DifferentialEquations
using LinearAlgebra
using KerbalMath
using KerbalGuidance
using Plots
import KRPC.Interface.SpaceCenter.Helpers as SCH
import KRPC.Interface.SpaceCenter.RemoteTypes as SCR

include("guidance.jl")

function setup(sp::Spacecraft)
    steps = 6
    path = nothing
    @withprogress name="Setting up..." begin
        @logprogress 1/steps info="setup parts"
        parts = SCH.Parts(sp.ves)
        sp.parts[:e1] = SCH.WithTag(parts, "e1")[1]
        sp.parts[:e2] = SCH.WithTag(parts, "e2")[1]
        sp.parts[:e3] = SCH.WithTag(parts, "e3")[1]
        sp.parts[:chute] = SCH.WithTag(parts, "chute")[1]
        sp.parts[:core] = SCH.WithTag(parts, "core")[1]

        @logprogress 2/steps info="setup conditions"
        sp.events[:s1] = Condition()
        sp.events[:s2] = Condition()
        sp.events[:s3] = Condition()
        sp.events[:guidance] = Condition()
        sp.events[:guidance_end] = Condition()
        sp.events[:deploy] = Condition()

        @logprogress 3/steps info="setup throttle"
        Control.throttle!(sp, 0.8)

        @logprogress 4/steps info="setting up plot backend"
        gr()
        gui()

        @logprogress 5/steps info="setting up guidance"
        ref = Navigation.ReferenceFrame.BCBF(sp)
        path = make_path(sp, ref; show=true)
        if isnothing(path)
            @error "Failed to get guidance"
            readline(stdin)
            error("Failed to get guidance")
        end

        @logprogress 6/steps info="done"
    end
    return path
end

function stage1(sp::Spacecraft)
    e1 = SCH.Engine(sp.parts[:e1])
    _, time_spent = Modules.Engine.ignite!(sp, e1; timeout=18)
    stage!(sp)
    notify(sp.events[:guidance])
    @log_attention "Engine ignition time" time_spent
    delay(sp, 67.5, "stage1")
    notify(sp.events[:s2])
end

function stage2(sp::Spacecraft)
    e2 = SCH.Engine(sp.parts[:e2])
    control = SCH.Control(sp.ves)
    wait(sp.events[:s2])
    SCH.RCS!(SCH.Control(sp.ves), true)
    SCH.Active!(e2, true)
    @log_mark "stage 2 active"
    delay(sp, 1.5)
    SCH.ToggleActionGroup(control, convert(UInt32, 2))
    delay(sp, 1.5)
    SCH.ToggleActionGroup(control, convert(UInt32, 1))
    delay(sp, 46, "stage2")
    notify(sp.events[:s3])
end

function stage3(sp::Spacecraft)
    e3 = SCH.Engine(sp.parts[:e3])
    control = SCH.Control(sp.ves)
    wait(sp.events[:s3])
    SCH.Active!(e3, true)
    @log_mark "stage 3 active"
    delay(sp, 0.5)
    SCH.ToggleActionGroup(control, convert(UInt32, 4))
    delay(sp, 44, "stage3")
    notify(sp.events[:guidance_end])
    notify(sp.events[:deploy])
end

function deploy(sp::Spacecraft)
    chute = sp.parts[:chute]
    wait(sp.events[:deploy])
    delay__bedrock_altitude(sp, "radar"; target=141e+3)
    # control = SCH.Control(sp.ves)
    SpaceLib.Modules.Parachute.arm(chute)
    # SCH.ToggleActionGroup(control, convert(UInt32, 3))
    delay(sp, 5)
end

function end_guidance(sp::Spacecraft, sources)
    wait(sp.events[:guidance_end])
    @log_dev "ending guidance"
    close.(sources)
end

main("Test1";
    log_path=homedir()*"/spacelib/Ranger",
    log_level=LogLevel(-650),
    save_file=false
) do sp
    @sync begin
        path = setup(sp)
        sources = setup_guidance(sp, path)
        task1 = @asyncx stage1(sp)
        task2 = @asyncx stage2(sp)
        task3 = @asyncx stage3(sp)
        task4 = @asyncx deploy(sp)
        task5 = @asyncx end_guidance(sp, sources)
        wait(task4)
    end
    delay(sp, 2)
end
