# Load custom ompl planning library into CppOMPL module
module CppOMPL
    using CxxWrap
    @wrapmodule(() -> joinpath(@__DIR__, "../lib", "libJlOMPL"))

    function __init__()
        @initcxx
    end
end