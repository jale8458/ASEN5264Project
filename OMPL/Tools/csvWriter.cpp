#include "csvWriter.h"

void csvWriter::savePathToCsv(ompl::geometric::PathGeometric& path, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    const ompl::base::StateSpacePtr ss = path.getSpaceInformation()->getStateSpace();
    const int ssType = ss->getType(); // Get the type of the state space
    const auto& states = path.getStates();
    for (const auto* state : states) {
        if (ssType == ompl::base::StateSpaceType::STATE_SPACE_SE2) { // SE2
            const ompl::base::SE2StateSpace::StateType* se2State = state->as<ompl::base::SE2StateSpace::StateType>();
            file << se2State->getX() << "," << se2State->getY() << "," << se2State->getYaw() << "\n";
        }
        else if (ssType == ompl::base::StateSpaceType::STATE_SPACE_REAL_VECTOR) { // Real Vector
            const ompl::base::RealVectorStateSpace::StateType* rvState = state->as<ompl::base::RealVectorStateSpace::StateType>();
            const double dim = ss->as<ompl::base::RealVectorStateSpace>()->getDimension();
            for (std::size_t i = 0; i < dim; ++i) {
                file << rvState->values[i];
                if (i + 1 < dim) file << ",";
            }
            file << "\n";
        }
        else {
            throw ompl::Exception("Unsupported state type in PathGeometric");
        }
    }

    file.close();
}

void csvWriter::saveObstaclesToCsv(const std::vector<Obs2D>& obstacles, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    for (int i = 0; i < obstacles.size(); ++i) {
        const Obs2D& obstacle = obstacles[i];
            for (const Eigen::Vector2d& vertex : obstacle) {
                file << i << "," << vertex[0] << "," << vertex[1] << "\n";
            }
        }

    file.close();
}

void csvWriter::saveEnvToCsv(const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& endpoints, const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& bounds, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    
    // Build iterable std::vector given tuples
    std::vector<Eigen::VectorXd> envData;
    envData.push_back(std::get<0>(endpoints));
    envData.push_back(std::get<1>(endpoints));
    envData.push_back(std::get<0>(bounds));
    envData.push_back(std::get<1>(bounds));

    for (const Eigen::VectorXd& data : envData) {
        for (std::size_t i = 0; i < data.size(); ++i) {
            file << data[i];
            if (i + 1 < data.size()) file << ",";
        }
        file << "\n";
    }

}