#pragma once

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <iostream>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/ScopedState.h>

#include "CollisionCheckers.h"

namespace ob = ompl::base;
using Obs2D = std::vector<Eigen::Vector2d>; // Contains vertices in CCW Order

namespace ObsSpace2D {
    std::vector<Obs2D> getTestSpace();
    std::vector<Obs2D> getObstaclesCSV(const std::string& filename);

    // WORKSPACE SPECIFIC FUNCTIONS
    ompl::base::RealVectorBounds getBoundsHW2WS2();
    std::tuple<Eigen::Vector2d, Eigen::Vector2d> getEndpointsHW2WS2();

    ompl::base::RealVectorBounds getBoundsGeneric();
    std::tuple<Eigen::Vector2d, Eigen::Vector2d> getEndpointsRandom(const std::tuple<Eigen::Vector2d, Eigen::Vector2d>& bounds);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> getEndpointsGeneric();
}

class SE2GoalRegion : public ob::GoalRegion {
public:
    SE2GoalRegion(const ob::SpaceInformationPtr &si, const ob::ScopedState<ob::SE2StateSpace>& goal, double posTol, double angTol = M_PI/12.0, double threshold = 1.0)
        : ob::GoalRegion(si), goal_(goal), posTol_(posTol), angTol_(angTol) {setThreshold(threshold);}
    double distanceGoal(const ob::State *state) const override;
    const ob::ScopedState<ob::SE2StateSpace>& getGoal() const {return goal_;}
private:
    ob::ScopedState<ob::SE2StateSpace> goal_;
    double posTol_, angTol_;
};

class CompoundGoalRegion : public ob::GoalRegion {
public:
    CompoundGoalRegion(const ob::SpaceInformationPtr &si, const ob::ScopedState<>& goal, double posTol, double velTol, double angTol = M_PI/12.0, double threshold = 1.0)
        : ob::GoalRegion(si), goal_(goal), goalSS_(goal->as<ob::CompoundStateSpace::StateType>()), goalVel_(goal->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)), posTol_(posTol), velTol_(velTol), angTol_(angTol) {setThreshold(threshold);}
    double distanceGoal(const ob::State *state) const override;
    const ob::ScopedState<>& getGoal() const {return goal_;}
private:
    ob::ScopedState<> goal_;  // keeps memory alive
    const ob::CompoundStateSpace::StateType* goalSS_;
    const ob::RealVectorStateSpace::StateType* goalVel_;
    double posTol_, velTol_, angTol_;
};