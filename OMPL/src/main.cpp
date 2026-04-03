#include "main.h"
#include <random>


// NOTE - we're using the following namespaces:
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

std::tuple<ob::PlannerStatus, ob::PathPtr> RRTSolve(const oc::SpaceInformationPtr& si, const ob::ScopedState<ob::SE2StateSpace>& q_init, const ob::GoalPtr &q_goal) {
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
    ob::PlannerPtr planner(std::make_shared<oc::RRT>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);
    return std::make_tuple(solved, pdef->getSolutionPath());
}

int main() {

    // Some parameters
    const std::string obsFile = "normalParking.csv";
    const bool isPoint = true;
    const double carLength = 2;
    const double carWidth = 1;

    // Bounds
    ob::RealVectorBounds se2Bounds = ObsSpace2D::getBoundsGeneric();
    ob::RealVectorBounds velBounds(2);
    // Forward Velocity bounds
    velBounds.setLow(0, -5);
    velBounds.setHigh(0, 5);
    // Steer angle bounds
    velBounds.setLow(1, -M_PI/3);
    velBounds.setHigh(1, M_PI/3);
    ob::RealVectorBounds cBounds(2);
    cBounds.setLow(-3);
    cBounds.setHigh(3);
    
    // Create state space and set bounds. This is a combination of SE2 and 2 R^2 spaces (v, delta)
    ob::StateSpacePtr se2Space(std::make_shared<ob::SE2StateSpace>());
    se2Space->as<ob::SE2StateSpace>()->setBounds(se2Bounds);
    ob::StateSpacePtr velSpace = std::make_shared<ob::RealVectorStateSpace>(2);
    velSpace->as<ob::RealVectorStateSpace>()->setBounds(velBounds);
    ob::StateSpacePtr sSpace = se2Space + velSpace;

    // Create control space
    const int cSpaceDim = 2;
    oc::ControlSpacePtr cSpace(std::make_shared<oc::RealVectorControlSpace>(sSpace, cSpaceDim));
    cSpace->as<oc::RealVectorControlSpace>()->setBounds(cBounds);

    // Setup obstacles
    std::vector<Obs2D> obstacles;
    obstacles = ObsSpace2D::getObstaclesCSV(obsFile);

    // Setup SpaceInformation/Collision Checker
    oc::SpaceInformationPtr si = std::make_shared<oc::SpaceInformation>(sSpace, cSpace);
    std::shared_ptr<SimpleCarAccel> car = std::make_shared<SimpleCarAccel>(si, carLength, carWidth);
    si->setStatePropagator(car);
    si->setStateValidityChecker(std::make_shared<CarCollisionChecker2D>(si, obstacles, car->getLength(), car->getWidth()));
    //// NOTE: Hard coded
    si->setStateValidityCheckingResolution(0.001);
    si->setPropagationStepSize(0.1);
    si->setMinMaxControlDuration(1, 10);

    // Start point
    ob::ScopedState<ob::SE2StateSpace> se2Start(se2Space);
    se2Start->setX(0.0);
    se2Start->setY(7.0);
    se2Start->setYaw(0.0);
    ob::ScopedState<ob::RealVectorStateSpace> velStart(velSpace);
    velStart->values[0] = 0.0;
    velStart->values[1] = 0.0;
    ob::ScopedState<> start(sSpace);
    start << se2Start << velStart;

    // Goal region
    ob::ScopedState<ob::SE2StateSpace> se2Goal(se2Space);
    se2Goal->setX(5.0);
    se2Goal->setY(4.0);
    se2Goal->setYaw(-M_PI/2);
    ob::ScopedState<ob::RealVectorStateSpace> velGoal(velSpace);
    velGoal->values[0] = 0.0;
    velGoal->values[1] = 0.0;
    ob::ScopedState<> goal(sSpace);
    goal << se2Goal << velGoal;
    ob::GoalPtr goalRegion = std::make_shared<CompoundGoalRegion>(si, goal, 0.5, 0.5);

    auto [solved, path] = RRTSolve(si, start, goalRegion);

    // Solution Checking
    if (solved == ob::PlannerStatus::EXACT_SOLUTION || solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        // Print solution type
        if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
            std::cout<<"Exact solution found.\n";
        } else {
            std::cout<<"No solution found. Plotting closest solution. \n";
        }
        
        // Get solution path
        og::PathGeometric geometricPath = path->as<oc::PathControl>()->asGeometric();

        std::cout << "Starting visualization..." << std::endl;
        // Get obstacles
        std::vector<Obs2D> obstacles = std::static_pointer_cast<CollisionChecker2D>(si->getStateValidityChecker())->getObstacles();
        
        // Visualizer::Plot2D(q_init, &geometricPath, obstacles);
        // Visualizer::Plot2D(start, std::static_pointer_cast<CompoundGoalRegion>(goalRegion)->getGoal(), &geometricPath, obstacles);
        Visualizer::Plot2D(&geometricPath, obstacles, car->getLength(), car->getWidth());
        Visualizer::Animate2D(&geometricPath, obstacles, car->getLength(), car->getWidth(), true);
        Visualizer::showFigures();

    }
    else {
        std::cout << "No solution found" << std::endl;
    }
}