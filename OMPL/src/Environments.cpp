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
    std::string filepath = "./../environments/" + filename;
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
    double posDist = std::sqrt(dx*dx + dy*dy);

    // atan2(sin/cos) to account for angle wrapping
    double dTheta = std::abs(std::atan2(std::sin(s->getYaw() - goal_[2]), std::cos(s->getYaw() - goal_[2])));

    // Return worst-case normalized distance
    return std::max(posDist / posTol_, dTheta / angTol_);
}

// ========== CompoundGoalRegion ==========
double CompoundGoalRegion::distanceGoal(const ob::State *state) const {
    const auto* s = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
    const auto* vel = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);

    double dx = s->getX() - goalSS_->getX();
    double dy = s->getY() - goalSS_->getY();
    double posDist = std::sqrt(dx*dx + dy*dy);

    // atan2(sin/cos) to account for angle wrapping
    double dTheta = std::abs(std::atan2(std::sin(s->getYaw() - goalSS_->getYaw()), std::cos(s->getYaw() - goalSS_->getYaw())));

    // Return worst-case normalized distance
    return std::max(posDist / posTol_, dTheta / angTol_);
}