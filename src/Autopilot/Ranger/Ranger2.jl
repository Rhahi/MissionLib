using SpaceLib
using SpaceLib.Timing
using SpaceLib.Control
using Logging
using ProgressLogging
using Unitful.DefaultSymbols
using DifferentialEquations
using LinearAlgebra
using KerbalGuidance
using Plots
import KRPC.Interface.SpaceCenter.Helpers as SCH

include("setup.jl")
include("guidance.jl")
include("stages.jl")

main("Test1";
    log_path=homedir()*"/spacelib/Ranger",
    log_level=LogLevel(-650),
    save_file=false
) do sp
    checklist(sp)
    path = setup(sp)
    @sync begin
        @async stage1(sp)
        @async stage2(sp)
        @async setup_guidance(sp, guidance_channel)
    end
end
