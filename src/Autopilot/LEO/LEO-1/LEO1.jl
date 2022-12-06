"Ranger with improved software structure"

using Logging
using SpaceLib
using KerbalGuidance
using LinearAlgebra
using DifferentialEquations
using KerbalMath
using Unitful.DefaultSymbols
using RemoteLogging.Terminal
using Plots
@importkrpc

include("prototypes.jl")
include("setup.jl")
include("guidance.jl")
include("ascent.jl")

function mission(sp)
    # adjust conditions here to change starting point
    conditions = [Condition() for _ = 1:6]
    s1, s2, s3, c1, c2, d1 = conditions
    ascent(sp, conditions)
    n = 0
    while n == 0
        @log_info "notifying"
        n = notify(s1)
        sleep(1)
    end
    nothing
end

function main(sp)
    checklist(sp)
    mission(sp)
    return sp
end

activate(port_logger=50050)
title = "LEO1"
sp = connect_to_spacecraft(title)
try
    main(sp)
    wait_for_input()
finally
    close(sp)
end
