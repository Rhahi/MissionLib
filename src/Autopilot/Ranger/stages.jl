function stage1(sp::Spacecraft)
    e1 = SCH.Engine(sp.parts[:e1])
    _, time_spent = Modules.Engine.ignite!(sp, e1; timeout=18)
    stage!(sp)
    notify(sp.events[:guidance])
    @log_attention "Engine ignition time" time_spent
    delay(sp, 68)
    notify(sp.events[:s2])
end


function stage2(sp::Spacecraft)
    e2 = SCH.Engine(sp.parts[:e2])
    wait(sp.events[:s2])
    SCH.Active!(e2, true)
    delay(sp, 2)
    control = SCH.Control(sp.ves)
    SCH.ToggleActionGroup(control, convert(UInt32, 4))
    @log_mark "new stage activated"
end
