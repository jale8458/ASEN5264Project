#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <Eigen/Core>

using Obs2D = std::vector<Eigen::Vector2d>; // Contains vertices in CCW Order

class CollisionChecker2D : public ompl::base::StateValidityChecker {
    public:
        CollisionChecker2D(const ompl::base::SpaceInformationPtr &si, const std::vector<Obs2D>& obstacles) :
            ompl::base::StateValidityChecker(si),
            ssType(si->getStateSpace()->getType()),
            obstacles(obstacles) {}
        void setObstacles(std::vector<Obs2D> obs) {
            obstacles = obs;
        }
        const std::vector<Obs2D>& getObstacles() {
            return obstacles;
        }
    protected:
        std::vector<Obs2D> obstacles;
        int ssType;
};

class PointCollChecker2D : public CollisionChecker2D {
    public:
        PointCollChecker2D(const ompl::base::SpaceInformationPtr &si, const std::vector<Obs2D>& obstacles) :
            CollisionChecker2D(si, obstacles) {}
        bool isValid(const ompl::base::State *state) const override;
};

class CarCollisionChecker2D : public CollisionChecker2D {
    public:
        CarCollisionChecker2D(const ompl::base::SpaceInformationPtr &si, const std::vector<Obs2D>& obstacles, const double length = 0.2, const double width = 0.1) :
            CollisionChecker2D(si, obstacles),
            L(length),
            W(width) {}
        bool isValid(const ompl::base::State *state) const override;
    private:
        double L; // Car length
        double W; // Car width
};