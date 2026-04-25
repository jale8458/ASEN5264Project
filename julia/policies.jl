# Naive Solutions
function always_continue(mdp, s)
    return :continue_plan
end

always_continue_policy = FunctionPolicy(s -> :continue_plan)
