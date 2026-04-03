#pragma once

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

// TO BE REMOVED
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>

#include <ompl/config.h>
#include <iostream>

#include "Visualizer.h"
#include "Environments.h"
#include "RandomObsGen.h"
#include "csvWriter.h"
#include "benchmarker.h"
#include "propagators.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::tuple<ob::PlannerStatus, ob::PathPtr> RRTSolve(const ob::SpaceInformationPtr& si, const ob::ScopedState<ob::SE2StateSpace>& q_init, const ob::GoalPtr &q_goal);
int plan();
bool isStateValid(const ob::State *state);