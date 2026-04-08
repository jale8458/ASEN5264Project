# Load custom ompl planning library into CppOMPL module, if not already loaded
if !isdefined(Main, :CppOMPL)
    include(joinpath(@__DIR__, "loadCppModule.jl"))
end
(SSTResult, control, controlDurations, path) = CppOMPL.PlanWithSST("normalParking.csv", "normalParkingEndpoints.csv", 0.5, 5.0)
display(SSTResult)
display(control)
display(controlDurations)
display(path)