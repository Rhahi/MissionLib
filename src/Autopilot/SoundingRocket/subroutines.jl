using SpaceLib
using LoggingExtras
using KRPC
import KRPC.Interface.SpaceCenter.Helpers as SCH
import KRPC.Interface.SpaceCenter as SC


function stage0(sp::Spacecraft)
    # release clamps and ignite boosters
    @info "Ignition"
    Control.stage(sp)
    Timing.delay(sp, 0.5)
    notify(sp.events[:stage1])
    Timing.delay(sp, 0.05)
end


function stage1(sp::Spacecraft)
    @infov 1 "enter stage 1"
    e1 = SCH.Engine(sp.parts[:e1])  # get engine part
    wait(sp.events[:stage1])

    @infov 1 "begin stage 1"
    SCH.Active!(e1, true)  # activate engine
    Timing.delay(sp, 0.5)
    Control.stage(sp)

    Timing.delay(sp, 38.5, "s1")
    notify(sp.events[:stage2])
end


function stage2(sp::Spacecraft)
    @infov 1 "enter stage 2"
    e2 = SCH.Engine(sp.parts[:e2])
    wait(sp.events[:stage2])
    @infov 1 "begin stage 2"

    SCH.Active!(e2, true)
    Timing.delay(sp, 0.5)
    Control.stage(sp)
end


# function deploy(sp::Spacecraft)
#     delay_for_value(sp, )
# end


# """Delay with observable, simple strategy"""
# function delay_for_value(sp::Spacecraft, measurement::Request, target::Number)
#     start = kerbal(sp.conn, f())
#     add_stream(sp.conn, (f(),)) do stream
#         if start > target
#             for (value,) in stream
#                 target ≥ value && break
#                 yield()
#             end
#         else
#             for (value,) in stream
#                 target ≤ value && break
#                 yield()
#             end
#         end
#     end
#     return start
# end
