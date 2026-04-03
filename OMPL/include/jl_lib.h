#pragma once

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/sst/SST.h>

#include <ompl/config.h>
#include <iostream>

// Julia cxx library
#include "jlcxx/jlcxx.hpp"

#include "Visualizer.h"
#include "Environments.h"
#include "RandomObsGen.h"
#include "csvWriter.h"
#include "benchmarker.h"
#include "propagators.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

struct SSTResult {
    bool foundSolution;
    std::vector<std::vector<double>> pathPoints;
    std::vector<std::vector<double>> controls;
};

std::tuple<ob::PlannerStatus, ob::PathPtr> SSTSolve(const oc::SpaceInformationPtr& si, const ob::ScopedState<ob::SE2StateSpace>& q_init, const ob::GoalPtr &q_goal, const double solveTime = 10.0);
SSTResult PlanWithSST(const std::string obsFile = "normalParking.csv");