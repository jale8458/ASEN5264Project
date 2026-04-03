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

SSTResult PlanWithSST(const std::string obsFile) {
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

    auto [solved, path] = SSTSolve(si, start, goalRegion);

    ///// Solution processing
    // Solution Status
    SSTResult result;
    if (solved == ob::PlannerStatus::EXACT_SOLUTION || solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout<<"Exact solution found.\n";
        result.foundSolution = true;
    }
    else if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
        std::cout<<"Solution not found. Returning nearest path.\n";
        result.foundSolution = false;
    }
    else {
        throw std::runtime_error("Unknown Solution Status.");
    }
    // Control Extraction

    // Geometric Path Extraction

}

/* =========== Port PlanWithSST to Julia ========== */

JLCXX_MODULE define_julia_module(jlcxx::Module& mod) {
    mod.method("PlanWithSST", &PlanWithSST);
}