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

include("setup.jl")
include("guidance.jl")
include("ascent.jl")

function mission(sp)
    ascent(sp; trigger=nothing, next=nothing)
    nothing
end

function main(sp)
    checklist(sp)
    mission(sp)
    return sp
end

activate(port_logger=50050)
title = "Ranger5"
sp = connect_to_spacecraft(title)
try
    main(sp)
    wait_for_input()
finally
    close(sp)
end
