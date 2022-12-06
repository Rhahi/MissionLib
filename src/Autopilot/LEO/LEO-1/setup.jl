function setup(sp::Spacecraft, parentid)
    steps = 4
    id = progress_init(parentid, "Internals")
    try
        progress_update(id, 1/steps, "Parts")
        collect_parts(sp)

        progress_update(id, 2/steps, "Ascent stage")
        register_ascent_stages(sp)
    finally
        progress_end(id)
    end
    nothing
end

function collect_parts(sp)
    parts = SCH.Parts(sp.ves)
    sp.parts[:e1] = SCH.WithTag(parts, "e1")[1]
    sp.parts[:e2] = SCH.WithTag(parts, "e2")[1]
    sp.parts[:e3] = SCH.WithTag(parts, "e3")[1]
    nothing
end

function checklist(sp)
    id = progress_init("Checklist")
    steps = 2
    try
        progress_update(id, 1/steps, "Setting up internals")
        setup(sp, id)
        progress_update(id, 2/steps, "Throttle check")
        Control.verify_throttle!(sp, 0.8)
    finally
        progress_end(id)
    end
end

function setup_condition(sp::Spacecraft, names::Symbol...)
    for name in names
        sp.events[name] = Condition()
    end
    nothing
end
