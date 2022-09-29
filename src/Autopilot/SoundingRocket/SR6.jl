"""Sounding rocket 6, 2 stage early rocket with 1 SRB"""

using SpaceLib
using Logging
using ProgressLogging
import KRPC.Interface.SpaceCenter.Helpers as SCH

include("./subroutines.jl")


function setup(sp::Spacecraft)
    steps = 4
    @withprogress name="Setting up..." begin
        @logprogress 1/steps info="setup triggers"
        sp.events[:stage1] = Condition()
        sp.events[:stage2] = Condition()

        @logprogress 2/steps info="setup tagged parts"
        parts = SCH.Parts(sp.ves)
        sp.parts[:e0] = SCH.WithTag(parts, "e0")[1]
        sp.parts[:e1] = SCH.WithTag(parts, "e1")[1]
        sp.parts[:e2] = SCH.WithTag(parts, "e2")[1]

        @logprogress 3/steps info="set throttle"
        Control.throttle!(sp, 1.0)

        @logprogress 4/steps info="done"
    end
end


main("SR Test"; log_path=homedir()*"/spacelib/SR", log_level=LogLevel(-1000)) do sp
    setup(sp)
    s0 = @async stage0(sp)
    s1 = @async stage1(sp)
    s2 = @async stage2(sp)
    # dep = @async deploy(sp)
    wait(s2)
end
