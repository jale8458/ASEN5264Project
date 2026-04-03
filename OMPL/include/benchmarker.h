#pragma once

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/PathGeometric.h>

#include <memory>
#include <vector>
#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct BenchmarkResult {
    std::vector<double> pathLengths;   // Only for EXACT solutions
    std::vector<double> solveTimes;  // Times for all runs
    std::vector<int> nodeCount;   // Number of nodes in tree
    std::vector<int> exactFlags;    // 1 if exact, 0 otherwise
};

BenchmarkResult benchmarkPlanner(ob::PlannerPtr planner, ob::ProblemDefinitionPtr pdef, int N, double maxTime = 1.0)
{
    BenchmarkResult out;

    for (int i = 0; i < N; ++i) {
        planner->clear();
        pdef->clearSolutionPaths();

        auto t0 = std::chrono::high_resolution_clock::now();
        ob::PlannerStatus status = planner->solve(maxTime);
        auto t1 = std::chrono::high_resolution_clock::now();

        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (status == ob::PlannerStatus::EXACT_SOLUTION) {
            out.exactFlags.push_back(1);

            auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
            out.solveTimes.push_back(ms);
            out.pathLengths.push_back(path->length());

            ompl::base::PlannerData pd(pdef->getSpaceInformation());
            planner->getPlannerData(pd);
            out.nodeCount.push_back(pd.numVertices());
        } else {
            out.exactFlags.push_back(0);
            // Do NOT push anything into pathLengths
        }
    }

    return out;
}
