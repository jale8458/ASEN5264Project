#include "jl_lib.h"

std::tuple<ob::PlannerStatus, ob::PathPtr> SSTSolve(const oc::SpaceInformationPtr& si, const ob::ScopedState<ob::SE2StateSpace>& q_init, const ob::GoalPtr &q_goal, const double solveTime) {
    // Uses control problem formulation to plan with RRT in a SE2 state space
    // Inputs:
    //  si = Space information object
    //  q_init = Problem starting state
    //  q_goal = Problem goal REGION

    // Set problem definition
    ob::ProblemDefinitionPtr pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->addStartState(q_init);
    pdef->setGoal(q_goal);

    // Set planner
    ob::PlannerPtr planner(std::make_shared<oc::SST>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(solveTime);
    return std::make_tuple(solved, pdef->getSolutionPath());
}

std::tuple<bool, jlcxx::ArrayRef<double, 2>, jlcxx::ArrayRef<double, 2>> PlanWithSST(const std::string obsFile, const double solveTime) {
    // Bounds
    ob::RealVectorBounds se2Bounds = ObsSpace2D::getBoundsGeneric();
    ob::RealVectorBounds cBounds(2);
    cBounds.setLow(-3);
    cBounds.setHigh(3);
    
    // Create state space and set bounds. This is a combination of SE2 and 2 R^2 spaces (v, delta)
    ob::StateSpacePtr se2Space(std::make_shared<ob::SE2StateSpace>());
    se2Space->as<ob::SE2StateSpace>()->setBounds(se2Bounds);
    ob::StateSpacePtr sSpace = se2Space;

    // Create control space
    const int cSpaceDim = 2;
    oc::ControlSpacePtr cSpace(std::make_shared<oc::RealVectorControlSpace>(sSpace, cSpaceDim));
    cSpace->as<oc::RealVectorControlSpace>()->setBounds(cBounds);

    // Setup obstacles
    std::vector<Obs2D> obstacles;
    obstacles = ObsSpace2D::getObstaclesCSV(obsFile);

    // Setup SpaceInformation/Collision Checker
    oc::SpaceInformationPtr si = std::make_shared<oc::SpaceInformation>(sSpace, cSpace);
    std::shared_ptr<SimpleUnicycle> propagator = std::make_shared<SimpleUnicycle>(si);
    si->setStatePropagator(propagator);
    si->setStateValidityChecker(std::make_shared<PointCollChecker2D>(si, obstacles));
    //// NOTE: Hard coded
    si->setStateValidityCheckingResolution(0.001);
    si->setPropagationStepSize(0.1);
    si->setMinMaxControlDuration(1, 10);

    // Start point
    ob::ScopedState<ob::SE2StateSpace> se2Start(se2Space);
    se2Start->setX(0.0);
    se2Start->setY(7.0);
    se2Start->setYaw(0.0);
    ob::ScopedState<> start(sSpace);
    start << se2Start;

    // Goal region
    ob::ScopedState<ob::SE2StateSpace> se2Goal(se2Space);
    se2Goal->setX(5.0);
    se2Goal->setY(4.0);
    se2Goal->setYaw(-M_PI/2);
    ob::ScopedState<> goal(sSpace);
    goal << se2Goal;
    ob::GoalPtr goalRegion = std::make_shared<SE2GoalRegion>(si, goal, 0.5, 0.5);

    auto [solved, path] = SSTSolve(si, start, goalRegion, solveTime);
    return ProcessPath(solved, path, si);
}

std::tuple<bool, jlcxx::ArrayRef<double, 2>, jlcxx::ArrayRef<double, 2>> ProcessPath(const ob::PlannerStatus& solved, const ob::PathPtr& path, const oc::SpaceInformationPtr& si) {
    /// Solution processing -> Boolean & Julia matrices
    oc::PathControl* pathControl = path->as<oc::PathControl>();

    size_t n_states = pathControl->getStateCount();
    size_t n_controls = pathControl->getControlCount();
    size_t ctrl_dim = si->getControlSpace()->as<oc::RealVectorControlSpace>()->getDimension();
    size_t state_dim = si->getStateDimension();

    // Solution Status
    bool foundSolution;
    if (solved == ob::PlannerStatus::EXACT_SOLUTION || solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout<<"Exact solution found.\n";
        foundSolution = true;
    }
    else if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
        std::cout<<"Solution not found. Returning nearest path.\n";
        foundSolution = false;
    }
    else {
        throw std::runtime_error("Unknown Solution Status.");
    }

    // Allocate control and geometric return matrices. These are 3xN because for better compatability with Julia
    jl_value_t* array_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 2);
    jl_array_t* jl_path_ptr = jl_alloc_array_2d(array_type, state_dim, n_states);
    jl_array_t* jl_ctrl_ptr = jl_alloc_array_2d(array_type, ctrl_dim, n_controls);
    jlcxx::ArrayRef<double, 2> controls(jl_ctrl_ptr);
    jlcxx::ArrayRef<double, 2> pathPoints(jl_path_ptr);

    // Control Extraction
    for (size_t i = 0; i < n_controls; ++i) {
        oc::RealVectorControlSpace::ControlType* c = pathControl->getControl(i)->as<oc::RealVectorControlSpace::ControlType>();
        for (size_t j = 0; j < ctrl_dim; ++j) {
            controls[j + i * ctrl_dim] = c->values[j];
        }
    }

    // Geometric Path Extraction
    for (size_t i = 0; i < n_states; ++i) {
        ob::SE2StateSpace::StateType* se2 = pathControl->getState(i)->as<ob::SE2StateSpace::StateType>();
        // Wrap yaw to [-pi, pi] before output
        double wrappedYaw = atan2(sin(se2->getYaw()), cos(se2->getYaw()));

        pathPoints[i * 3] = se2->getX();
        pathPoints[1+i * 3] = se2->getY();
        pathPoints[2+i * 3] = wrappedYaw;
    }

    return std::make_tuple(foundSolution, controls, pathPoints);
}

/* =========== Port PlanWithSST to Julia ========== */

JLCXX_MODULE define_julia_module(jlcxx::Module& mod) {
    mod.method("PlanWithSST", &PlanWithSST);
}