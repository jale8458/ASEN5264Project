#include "Environments.h"

std::vector<Obs2D> ObsSpace2D::getTestSpace() {
        std::vector<Obs2D> obstacles;
        Obs2D obstacle;
        obstacle.push_back(Eigen::Vector2d(-8,-8));
        obstacle.push_back(Eigen::Vector2d(8,-8));
        obstacle.push_back(Eigen::Vector2d(8,8));
        obstacle.push_back(Eigen::Vector2d(-8,8));
        obstacles.push_back(obstacle);
        return obstacles;
    }

std::vector<Obs2D> ObsSpace2D::getObstaclesCSV(const std::string& filename) {
    // Reads obstacles from "environments" folder
    std::string filepath = std::string(ENV_DIR) + filename;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filepath);
    }

    std::vector<Obs2D> obstacles;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        int obstacleNum;
        double x, y;
        char comma;
        if (ss >> obstacleNum >> comma >> x >> comma >> y) {
            if (obstacleNum >= static_cast<int>(obstacles.size())) {
                obstacles.resize(obstacleNum + 1);
            }
            obstacles[obstacleNum].push_back(Eigen::Vector2d(x,y));
        } else {
            throw ompl::Exception("Invalid obstacle specification in CSV line: " + line);
        }
    }
    return obstacles;
}

std::tuple<ob::ScopedState<>, ob::ScopedState<>> ObsSpace2D::getStartGoal(ompl::control::SpaceInformationPtr& si, const std::string& filename) {
    // Reads start and goal states from "environments" folder
    std::string filepath = std::string(ENV_DIR) + filename;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filepath);
    }

    ob::StateSpacePtr ss = si->getStateSpace();
    int ssType(ss->getType());
    ob::ScopedState<> start(ss);
    ob::ScopedState<> goal(ss);
    if (ssType == ob::StateSpaceType::STATE_SPACE_SE2) {
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;

            std::stringstream lineStream(line);
            std::string pointType;
            double x, y, theta;
            char comma;
            if (lineStream >> pointType >> x >> comma >> y >> comma >> theta) {
                if (pointType == "start,") {
                    start->as<ob::SE2StateSpace::StateType>()->setX(x);
                    start->as<ob::SE2StateSpace::StateType>()->setY(y);
                    start->as<ob::SE2StateSpace::StateType>()->setYaw(theta);
                }
                else if (pointType == "goal,") {
                    goal->as<ob::SE2StateSpace::StateType>()->setX(x);
                    goal->as<ob::SE2StateSpace::StateType>()->setY(y);
                    goal->as<ob::SE2StateSpace::StateType>()->setYaw(theta);
                }
                else {
                    throw ompl::Exception("Invalid point specification in CSV line: " + line);
                }
            } else {
                throw ompl::Exception("Invalid line specification in CSV line: " + line);
            }
        }
        return std::make_tuple(start, goal);
    } else if (ssType == ob::StateSpaceType::STATE_SPACE_UNKNOWN) {
        ob::StateSpacePtr se2Space = ss->as<ob::CompoundStateSpace>()->getSubspace(0);
        ob::StateSpacePtr velSpace = ss->as<ob::CompoundStateSpace>()->getSubspace(1);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;

            std::stringstream lineStream(line);
            std::string pointType;
            double x, y, theta, v, delta;
            char comma;
            if (lineStream >> pointType >> x >> comma >> y >> comma >> theta >> comma >> v >> comma >> delta) {
                if (pointType == "start,") {
                    ob::ScopedState<ob::SE2StateSpace> se2Start(se2Space);
                    se2Start->setX(x);
                    se2Start->setY(y);
                    se2Start->setYaw(theta);
                    ob::ScopedState<ob::RealVectorStateSpace> velStart(velSpace);
                    velStart->values[0] = v;
                    velStart->values[1] = delta;
                    start << se2Start << velStart;
                }
                else if (pointType == "goal,") {
                    ob::ScopedState<ob::SE2StateSpace> se2Goal(se2Space);
                    se2Goal->setX(x);
                    se2Goal->setY(y);
                    se2Goal->setYaw(theta);
                    ob::ScopedState<ob::RealVectorStateSpace> velGoal(velSpace);
                    velGoal->values[0] = v;
                    velGoal->values[1] = delta;
                    goal << se2Goal << velGoal;
                }
                else {
                    throw ompl::Exception("Invalid obstacle specification in CSV line: " + line);
                }
            } else {
                throw ompl::Exception("Invalid obstacle specification in CSV line: " + line);
            }
        }
        return std::make_tuple(start, goal);
    } else {
        throw ompl::Exception("Unknown state space type");
    }
}

// WORKSPACE SPECIFIC FUNCTIONS
ompl::base::RealVectorBounds ObsSpace2D::getBoundsHW2WS2() {
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-7);
    bounds.setHigh(0, 36);
    bounds.setHigh(1, 7);

    return bounds;
}
std::tuple<Eigen::Vector2d, Eigen::Vector2d> ObsSpace2D::getEndpointsHW2WS2() {
    Eigen::Vector2d q_init(0,0);
    Eigen::Vector2d q_goal(35,0);

    return std::make_tuple(q_init, q_goal);
}

ompl::base::RealVectorBounds ObsSpace2D::getBoundsGeneric() {
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(10);

    return bounds;
}
std::tuple<Eigen::Vector3d, Eigen::Vector3d> ObsSpace2D::getEndpointsGeneric() {
    Eigen::Vector3d q_init(1,1,0);
    Eigen::Vector3d q_goal(9,9,0);

    return std::make_tuple(q_init, q_goal);
}
std::tuple<Eigen::Vector2d, Eigen::Vector2d> ObsSpace2D::getEndpointsRandom(const std::tuple<Eigen::Vector2d, Eigen::Vector2d>& bounds) {
    const Eigen::Vector2d& boundsLow = std::get<0>(bounds);
    const Eigen::Vector2d& boundsHigh = std::get<1>(bounds);

    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> dx(boundsLow.x(), boundsHigh.x());
    std::uniform_real_distribution<double> dy(boundsLow.y(), boundsHigh.y());

    Eigen::Vector2d q_init, q_goal;

    q_init << dx(rng), dy(rng);
    do {
        q_goal << dx(rng), dy(rng);
    } while ((q_goal - q_init).norm() < (boundsHigh - boundsLow).norm()/2); // Insure non-trivial problems
    return std::make_tuple(q_init, q_goal);
}

// ========== SE2GoalRegion ==========
double SE2GoalRegion::distanceGoal(const ob::State *state) const {
    const auto* s = state->as<ob::SE2StateSpace::StateType>();

    double dx = s->getX() - goal_[0];
    double dy = s->getY() - goal_[1];
    double normPosDist = std::sqrt(dx*dx + dy*dy)/posTol_;

    // atan2(sin/cos) to account for angle wrapping
    double dTheta = std::abs(std::atan2(std::sin(s->getYaw() - goal_[2]), std::cos(s->getYaw() - goal_[2])));

    // If dTheta is within angTol_, then return posDist, otherwise return 1.0 + posDist (which never passes)
    if (dTheta > angTol_) {
        return 1.0 + normPosDist; 
    } else{
        return normPosDist;
    }
}

// ========== CompoundGoalRegion ==========
double CompoundGoalRegion::distanceGoal(const ob::State *state) const {
    // Get goal state
    const ob::SE2StateSpace::StateType* goalSE2 = goalSS_->as<ob::SE2StateSpace::StateType>(0);
    const double goalVel = goalSS_->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
    // Get current state
    const auto* s = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
    const double vel = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0];

    double dx = s->getX() - goalSE2->getX();
    double dy = s->getY() - goalSE2->getY();
    double normPosDist = std::sqrt(dx*dx + dy*dy)/posTol_;

    // Delta in velocity
    double dv = abs(vel - goalVel);

    // atan2(sin/cos) to account for angle wrapping
    double dTheta = std::abs(std::atan2(std::sin(s->getYaw() - goalSE2->getYaw()), std::cos(s->getYaw() - goalSE2->getYaw())));

    // If dTheta is within angTol_ AND dv is within velTol_, then return posDist, otherwise return 1.0 + posDist (which never passes)
    if (dTheta > angTol_ || dv > velTol_) {
        return 1.0 + normPosDist; 
    } else{
        return normPosDist;
    }
}