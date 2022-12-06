function setup(sp::Spacecraft)
    steps = 6
    path = nothing
    @withprogress name="Setting up..." begin
        @logprogress 1/steps info="setup parts"
        collect_parts(sp)

        @logprogress 2/steps info="setup conditions"
        collect_flags(sp)

        @logprogress 3/steps info="setup throttle"
        Control.throttle!(sp, 0.8)

        @logprogress 4/steps info="setting up plot backend"
        gr()
        gui()

        @logprogress 5/steps info="setting up guidance"
        ref = Navigation.ReferenceFrame.BCBF(sp)
        path = make_path(sp, ref; show=true)
        isnothing(path) && error("Failed to get guidance")

        @logprogress 6/steps info="done"
    end
    return path
end

function collect_parts(sp)
    parts = SCH.Parts(sp.ves)
    sp.parts[:e1] = SCH.WithTag(parts, "e1")[1]
    sp.parts[:e2] = SCH.WithTag(parts, "e2")[1]
    sp.parts[:t1] = SCH.WithTag(parts, "t1")[1]
    sp.parts[:t2] = SCH.WithTag(parts, "t2")[1]
    sp.parts[:core] = SCH.WithTag(parts, "core")[1]
    nothing
end

function collect_flags(sp)
    sp.events[:s1] = Condition()
    sp.events[:s2] = Condition()
    sp.events[:guidance] = Condition()
end

function checklist(sp::Spacecraft) end

function monitor_deviation(sp::Spacecraft, guidance) end

function abort(sp::Spacecraft) end
