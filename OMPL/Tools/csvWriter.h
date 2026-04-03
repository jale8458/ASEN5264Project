#pragma once

#include <fstream>
#include <Eigen/Dense>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "CollisionCheckers.h"

namespace csvWriter {
    void savePathToCsv(ompl::geometric::PathGeometric& path, const std::string& filename);
    void saveObstaclesToCsv(const std::vector<Obs2D>& obstacles, const std::string& filename);
    void saveEnvToCsv(const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& endpoints, const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& bounds, const std::string& filename);
}