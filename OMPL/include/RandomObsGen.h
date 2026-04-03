#pragma once

#include <Eigen/Dense>
#include <vector>
#include <tuple>

#include "CollisionCheckers.h"


namespace ObsGenerator2D {
    enum class ObstacleShape { Polygon, Rectangle, Square };
    std::vector<Eigen::VectorXd> generateRandomPath(const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& endpoints, const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& bounds, const int numNodes = 5);
    std::vector<Obs2D> generateRandomObstacles(
                                                const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& endpoints,
                                                const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& bounds,
                                                int numObstacles = 100,
                                                int vertexCount = 6,
                                                double scale = 1.2,
                                                double clearance = 0.01,
                                                ObsGenerator2D::ObstacleShape shape = ObsGenerator2D::ObstacleShape::Polygon);
}