# # Load custom ompl planning library into CppOMPL module

# module CppOMPL
#     using CxxWrap
#     @wrapmodule(() -> joinpath(@__DIR__, "..", "lib", "JlOMPL.dll"))

#     function __init__()
#         @initcxx
#     end
# end

module CppOMPL
    using CxxWrap

    const jlompl_path = normpath(joinpath(@__DIR__, "..", "lib", "JlOMPL.dll"))
    @wrapmodule(() -> jlompl_path)

    function __init__()
        @initcxx
    end
end