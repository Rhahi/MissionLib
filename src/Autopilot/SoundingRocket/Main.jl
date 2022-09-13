using SpaceLib
using Logging
import KRPC.Interface.SpaceCenter.Helpers as RC

include("./subroutines.jl")


function setup(sp::Spacecraft)
    @info "Create blank conditions"
    sp.events["stage1"] = Condition()
    sp.events["stage2"] = Condition()
    parts = RC.Parts(sp.ves)
    @info "Setup tagged parts"
    sp.parts["e0"] = RC.WithTag(parts, "e0")[1]
    sp.parts["e1"] = RC.WithTag(parts, "e1")[1]
    sp.parts["e2"] = RC.WithTag(parts, "e2")[1]
    @info "Set throttle"
    Control.throttle!(sp, 1.0)
end


main("SR Test"; log_path=homedir()*"/spacelib/SR", log_level=LogLevel(-50)) do sp
    @info "Setting up..."
    Telemetry.@telemetry "telemetry test" a = 1
    setup(sp)
    @info "Setup complete"

    @info "Schedule stage 0"
    s0 = @async stage0(sp)
    @info "Schedule stage 1"
    s1 = @async stage1(sp)

    sleep(5)
    wait(s1)
end
