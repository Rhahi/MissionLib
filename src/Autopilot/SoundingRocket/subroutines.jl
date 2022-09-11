using SpaceLib
using LoggingExtras
import KRPC.Interface.SpaceCenter.Helpers as RC


function stage0(sp::Spacecraft)
    # release clamps and ignite boosters
    @infov 1 "Staging..."
    Control.stage(sp)
    notify(sp.events["stage1"])
end


function stage1(sp::Spacecraft)
    # get engine part
    e1 = sp.parts["e1"]

    # wait until stage 1 starts
    wait(sp.events["stage1"])

    # wait until TWR approaches 1, or timeout
    sleep(0.5)

    # activate engine
    RC.Active!(e1, true)

    sleep(0.1)
    Control.stage(sp)
end


function stage2(sp::Spacecraft)
    # # get engine part
    # e2 = sp.ves.parts.with_tags("e2")[1].engine

    # # wait until stage1 is almost burnt out


    # # activate stage 2
    # e2.active = true

    # # separate stage 1
    # Staging.stage()

    # # start monitoring nominal thrust and attitude
end
