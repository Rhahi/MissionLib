using SpaceLib, SpaceLib.Timing, SpaceLib.Telemetry
using KRPC
using Logging, LoggingExtras
using ProgressLogging
import KRPC.Interface.SpaceCenter as SC
import KRPC.Interface.SpaceCenter.Helpers as SCH
import KRPC.Interface.KRPC as KR


main("SR Test"; log_path=homedir()*"/spacelib/Sleep", log_level=LogLevel(-1000)) do sp
    @debug "Setup complete"
    @telemetry "Telemetry test"

    @info "Starting non-async sleep"
    delay(sp, 5.)
    @info "5 second non-async delay over"

    @info "Starting KRPC sleep"
    @async delay(sp, 5.)
    @info "5 second delay over"

    @info "Starting main sleep"
    sleep(10)
    @info "Main sleep done"
end