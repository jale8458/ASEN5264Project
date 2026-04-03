#include "CollisionCheckers.h"

bool PointCollChecker2D::isValid(const ompl::base::State *state) const {
    Eigen::Vector2d p;
    // Get se2 state space
    if (ssType == ompl::base::StateSpaceType::STATE_SPACE_SE2) { // SE2
        const ompl::base::SE2StateSpace::StateType* se2 = state->as<ompl::base::SE2StateSpace::StateType>();
        p = Eigen::Vector2d(se2->getX(), se2->getY());
    }
    else if (ssType == ompl::base::StateSpaceType::STATE_SPACE_UNKNOWN) { // Compound ss
        const ompl::base::SE2StateSpace::StateType* se2 = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
        p = Eigen::Vector2d(se2->getX(), se2->getY());
    }
    else
    {
        throw std::runtime_error("Unknown State Space Type.");
    }
    for (const Obs2D& obstacleVertices : obstacles) {
        bool inside = false;
        int n = obstacleVertices.size();
        // In obstacle collision checking
        for (int i = 0, j = n - 1; i < n; j = i++) {
            const Eigen::Vector2d& pi = obstacleVertices[i];
            const Eigen::Vector2d& pj = obstacleVertices[j];
            if (((pi.y() > p.y()) != (pj.y() > p.y())) &&
                (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x())) {
                inside = !inside;
            }
        }
        if (inside) {
            return false;
        }
    }
    return true;
}

bool CarCollisionChecker2D::isValid(const ompl::base::State *state) const {
    // Check if in bounds first
    if (!si_->satisfiesBounds(state)) return false;
    // Extract position and yaw
    const ompl::base::SE2StateSpace::StateType* se2;
    if (ssType == ompl::base::StateSpaceType::STATE_SPACE_SE2) {
        se2 = state->as<ompl::base::SE2StateSpace::StateType>();
    } else { // Compound
        se2 = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
    }

    const double x = se2->getX();
    const double y = se2->getY();
    const double theta = se2->getYaw();

    // Compute car corners in world frame
    // Car is centered at (x, y) with half-length L/2 and half-width W/2
    const double cl = L / 2.0;
    const double cw = W / 2.0;
    const double cosT = cos(theta);
    const double sinT = sin(theta);

    // Four corners: front-left, front-right, rear-right, rear-left
    std::vector<Eigen::Vector2d> corners = {
        {x + cl*cosT - cw*sinT,  y + cl*sinT + cw*cosT},  // front-left
        {x + cl*cosT + cw*sinT,  y + cl*sinT - cw*cosT},  // front-right
        {x - cl*cosT + cw*sinT,  y - cl*sinT - cw*cosT},  // rear-right
        {x - cl*cosT - cw*sinT,  y - cl*sinT + cw*cosT},  // rear-left
    };

    // Check each obstacle
    for (const Obs2D& obstacleVertices : obstacles) {
        // Check if any car corner is inside obstacle
        for (const Eigen::Vector2d& corner : corners) {
            bool inside = false;
            int n = obstacleVertices.size();
            for (int i = 0, j = n - 1; i < n; j = i++) {
                const Eigen::Vector2d& pi = obstacleVertices[i];
                const Eigen::Vector2d& pj = obstacleVertices[j];
                if (((pi.y() > corner.y()) != (pj.y() > corner.y())) &&
                    (corner.x() < (pj.x() - pi.x()) * (corner.y() - pi.y()) / (pj.y() - pi.y()) + pi.x())) {
                    inside = !inside;
                }
            }
            if (inside) return false;
        }
    }
    return true;
}