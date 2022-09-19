using SpaceLib, SpaceLib.Timing, SpaceLib.Telemetry
using KRPC
using Logging, LoggingExtras
using ProgressLogging
import KRPC.Interface.SpaceCenter as SC
import KRPC.Interface.SpaceCenter.Helpers as SCH
import KRPC.Interface.KRPC as KR



function my_delay(sp::Spacecraft)
    @debug "Setup complete"
    x = delay(sp, 1., "1")
    @info x
    x = delay(sp, 1., "1")
    @info x
    x = delay(sp, 1., "1")
    @info x
    x = delay(sp, 1., "1")
    @info x
    x = delay(sp, 0.06, "0.06")
    @info x
    x = delay(sp, 0.06, "0.06")
    @info x
    x = delay(sp, 0.06, "0.06")
    @info x
    x = delay(sp, 0.06, "0.06")
    @info x
    x = delay(sp, 0.04, "0.04")
    @info x
    x = delay(sp, 0.04, "0.04")
    @info x
    x = delay(sp, 0.04, "0.04")
    @info x
    x = delay(sp, 0.04, "0.04")
    @info x
    x = delay(sp, 0.04, "0.04")
    @info x
    x = delay(sp, 0.04, "0.04")
    @info x
    x = delay(sp, 0.04, "0.04")
    @info x
    x = delay(sp, 0.02, "0.02")
    @info x
    x = delay(sp, 0.02, "0.02")
    @info x
    x = delay(sp, 0.02, "0.02")
    @info x
    x = delay(sp, 0.02, "0.02")
    @info x
    x = delay(sp, 0.02, "0.02")
    @info x
    x = delay(sp, 0.02, "0.02")
    @info x
    x = delay(sp, 0.02, "0.02")
    @info x
end


main("SR Test"; log_path=homedir()*"/spacelib/Sleep", log_level=LogLevel(-1000)) do sp
    my_delay(sp)
end
