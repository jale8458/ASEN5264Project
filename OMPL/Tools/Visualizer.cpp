#include "Visualizer.h"


void Visualizer::Plot2D(const Eigen::VectorXd& q_init, const Eigen::VectorXd& q_goal, ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles) {
    // Get bounds of state space
    const auto& bounds = path->getSpaceInformation()->getStateSpace()->as<ompl::base::SE2StateSpace>()->getBounds();

    PyCaller::call("Visualizer", "plotPathWithObstacles2D", std::forward_as_tuple(path, bounds.low, bounds.high, obstacles, q_init, q_goal));
}

void Visualizer::Plot2D(const ompl::base::ScopedState<>& q_init, ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles) {
    // Get bounds of state space
    const ompl::base::RealVectorBounds& bounds = getBounds(path);
    
    PyCaller::call("Visualizer", "plotPathWithObstacles2D", std::forward_as_tuple(path, bounds.low, bounds.high, obstacles, q_init));
}

void Visualizer::Plot2D(const ompl::base::ScopedState<>& q_init, const ompl::base::ScopedState<>& q_goal, ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles) {
    // Get bounds of state space
    const ompl::base::RealVectorBounds& bounds = getBounds(path);

    PyCaller::call("Visualizer", "plotPathWithObstacles2D", std::forward_as_tuple(path, bounds.low, bounds.high, obstacles, q_init, q_goal));
}

void Visualizer::Plot2D(ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles, const double carLength, const double carWidth) {
    // Get bounds of state space
    const ompl::base::RealVectorBounds& bounds = getBounds(path);

    PyCaller::call("Visualizer", "plotCarPathWithObstacles2D", std::forward_as_tuple(path, bounds.low, bounds.high, obstacles, carLength, carWidth));
}

void Visualizer::Animate2D(ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles, const double carLength, const double carWidth, const bool saveAnimation) {
    // Get bounds of state space
    const ompl::base::RealVectorBounds& bounds = getBounds(path);

    if (saveAnimation) {
        PyCaller::call("Visualizer", "animateCarPath2D", std::forward_as_tuple(path, bounds.low, bounds.high, obstacles, carLength, carWidth, 100, "./../animation/car.mp4"));
    } else {
        PyCaller::call("Visualizer", "animateCarPath2D", std::forward_as_tuple(path, bounds.low, bounds.high, obstacles, carLength, carWidth));
    }
}

void Visualizer::boxplot(const std::vector<std::vector<double>>& data, const std::vector<std::string>& labels, const std::string& title, const std::string& ylabel,const std::string& xlabel) {
    PyCaller::call("Visualizer", "plot_boxplot", std::forward_as_tuple(data, labels, title, ylabel, xlabel));
}

void Visualizer::boxplot(const std::vector<std::vector<int>>& data, const std::vector<std::string>& labels, const std::string& title, const std::string& ylabel,const std::string& xlabel) {
    PyCaller::call("Visualizer", "plot_boxplot", std::forward_as_tuple(data, labels, title, ylabel, xlabel));
}

void Visualizer::boxplot(const std::vector<std::vector<double>>& data) {
    PyCaller::call("Visualizer", "plot_boxplot", std::forward_as_tuple(data));
}

void Visualizer::barGraph(const std::vector<double>& data, const std::vector<std::string>& labels, const std::string& title, const std::string& ylabel,const std::string& xlabel) {
    PyCaller::call("Visualizer", "plot_barGraph", std::forward_as_tuple(data, labels, title, ylabel, xlabel));
}

void Visualizer::barGraph(const std::vector<double>& data) {
    PyCaller::call("Visualizer", "plot_barGraph", std::forward_as_tuple(data));
}

void Visualizer::showFigures() {
    PyCaller::call("Visualizer", "showFigures");
}

const ompl::base::RealVectorBounds& Visualizer::getBounds(ompl::geometric::PathGeometric* path) {
    ompl::base::StateSpacePtr ss(path->getSpaceInformation()->getStateSpace());
    if (ss->getType() == ompl::base::StateSpaceType::STATE_SPACE_SE2) { // SE2
        return ss->as<ompl::base::SE2StateSpace>()->getBounds();
    }
    else if (ss->getType() == ompl::base::StateSpaceType::STATE_SPACE_UNKNOWN) { // Compound ss
        return ss->as<ompl::base::CompoundStateSpace>()->as<ompl::base::SE2StateSpace>(0)->getBounds();
    }
    else
    {
        throw std::runtime_error("Unknown State Space Type.");
    }
}