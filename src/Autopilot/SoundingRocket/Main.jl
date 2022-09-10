using SpaceLib.KRPC

include("./subroutines.jl")


function setup(name::String)
    sp = connect(name)
    sp.events["stage1"] = Condition()
    sp.events["stage2"] = Condition()
    sp.parts["e0"] = sp.ves.parts.with_tag("e0")[1].engine
    sp.parts["e1"] = sp.ves.parts.with_tag("e1")[1].engine
    sp.parts["e2"] = sp.ves.parts.with_tag("e2")[1].engine

    sp
end


function main()
    sp = setup("Test")
    @async stage0(sp)
    @async stage1(sp)
    @async stage2(sp)
end
