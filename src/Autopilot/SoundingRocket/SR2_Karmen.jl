"""Rocket designed for the Karmen line contract"""

using SpaceLib
using SpaceLib.Timing, SpaceLib.Control
using Logging
import KRPC.Interface.SpaceCenter.Helpers as SCH


function setup(sp::Spacecraft)
    sp.events[:stage] = Condition()
    parts = SCH.Parts(sp.ves)
    sp.parts[:e1] = SCH.WithTag(parts, "e1")[1]
    sp.parts[:e2] = SCH.WithTag(parts, "e2")[1]
    Control.throttle!(sp, 1.0)
end


main("Karmen"; log_path=homedir()*"/spacelib/SR", log_level=LogLevel(-1000)) do sp
    delay(sp, 0.5, "warmup")
    setup(sp)
    e1 = SCH.Engine(sp.parts[:e1])
    e2 = SCH.Engine(sp.parts[:e2])

    stage!(sp)
    SCH.Active!(e1, true)
    delay(sp, 0.2)
    stage!(sp)
    delay(sp, 45, "stage1")
    SCH.Active!(e2, true)
    delay(sp, 0.2)
    stage!(sp)
end
