using KRPC
using SpaceLib

include("./subroutines.jl")


function setup(s::Spacecraft)
    s.events["stage1"] = Condition()
    s.events["stage2"] = Condition()
    s.parts["e0"] = s.ves.parts.with_tag("e0")[1].engine
    s.parts["e1"] = s.ves.parts.with_tag("e1")[1].engine
    s.parts["e2"] = s.ves.parts.with_tag("e2")[1].engine
end


function main(s::Spacecraft)
    @info "Setting up..."
    setup(s)
    @info "Setup complete"

    @info "Schedule stage 0"
    s0 = @async stage0(sp)
    @info "Schedule stage 1"
    s1 = @async stage1(sp)

    sleep(5)
    wait(s1)
end


try
    s = connect_to_spacecraft("SR Test")
    main(s)
finally
    close(s.conn)
end
