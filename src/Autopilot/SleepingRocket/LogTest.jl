"""
use SpaceLib.Timing.delay() to test:
- progresslogging functionality
- minimum game delay resolution
- delay precision

The results were used to refine delay() methods.
"""

using SpaceLib, SpaceLib.Timing
using Logging, LoggingExtras


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
