using SpaceLib
using SpaceLib.Timing
using SpaceLib.Control
using Logging
using ProgressLogging
using Unitful.DefaultSymbols
using DifferentialEquations
using LinearAlgebra
using KerbalMath
using KerbalGuidance
using Plots
import KRPC.Interface.SpaceCenter.Helpers as SCH
import KRPC.Interface.SpaceCenter.RemoteTypes as SCR

include("setup.jl")
include("guidance.jl")
include("stages.jl")

main("Test1";
    log_path=homedir()*"/spacelib/Ranger",
    log_level=LogLevel(-650),
    save_file=false
) do sp
    @sync begin
        checklist(sp)
        path = setup(sp)
        sources = setup_guidance(sp, path)
        task1 = @asyncx stage1(sp)
        task2 = @asyncx stage2(sp)
        wait(task2)
        close.(sources)
    end
end
