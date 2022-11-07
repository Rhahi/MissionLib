function stage1(sp::Spacecraft)
    e1 = SCH.Engine(sp.parts[:e1])
    @log_dev "begin ignite"
    _, time_spent = Modules.Engine.ignite!(sp, e1; timeout=10)
    stage!(sp)
    notify(sp.events[:guidance])
    control = SCH.Control(sp.ves)
    delay(sp, 60-time_spent, "stage1")
    SCH.Roll!(control, F32(1))
    delay(sp, 8, "stage1 spin")
    notify(sp.events[:s2])
end


function stage2(sp::Spacecraft)
    e2 = SCH.Engine(sp.parts[:e2])
    wait(sp.events[:s2])
    SCH.Active!(e2, true)
    delay(sp, 2)
    control = SCH.Control(sp.ves)
    SCH.ToggleActionGroup(control, convert(UInt32, 4))
end
