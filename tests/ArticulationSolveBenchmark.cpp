#include <gtest/gtest.h>
#include "support/ArticulationComparison.h"
#include "ConstraintSolveOrder.h"
#include "SequentialConstraintSolve.h"
#include <chrono>
#include <iostream>
#include <algorithm>

using namespace ArticulationComparison;
using namespace OurPaintDCM::System;
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}
}

TEST(ArticulationSolveBenchmark, ColdAndRepeatedSolves) {
    for (const auto* shape : {"chain", "star", "cycle"}) {
        for (std::size_t size : {3, 32, 128, 512}) {
            std::vector<double> topology, sequential, splitCold, jointCold, splitWarm, jointWarm;
            std::size_t blocks = 0, largestTask = 0;
            for (int repeat = 0; repeat < 5; ++repeat) {
                Model split(size, shape), joint(size, shape), staged(size, shape);
                const auto& system = staged.manager.getRequirementSystem();
                auto start = Clock::now();
                auto plan = buildConstraintDecomposition(system,
                    decomposeByArticulationPoints(system.buildDependencyGraph()));
                topology.push_back(elapsed(start));
                if (plan) {
                    blocks = plan->blocks.size();
                    start = Clock::now();
                    const auto order = chooseConstraintSolveOrder(system, *plan);
                    ASSERT_TRUE(solveSequentialConstraints(system, *plan, order, {}, SolverType::LEVENBERG_MARQUARDT,
                        1e-6, [&](TaskMatrix& task) {
                            largestTask = std::max(largestTask, task.getValues().size());
                            SparseLMSolver optimizer(100, 1e-3, 1e-8, 1e-8, 1e-14);
                            optimizer.setTask(&task);
                            optimizer.optimize();
                        }));
                    sequential.push_back(elapsed(start));
                } else sequential.push_back(0);
                start = Clock::now();
                ASSERT_TRUE(split.solve());
                splitCold.push_back(elapsed(start));
                start = Clock::now();
                JointSolve baseline(joint);
                ASSERT_TRUE(baseline.solve());
                jointCold.push_back(elapsed(start));
                split.perturb(0.3);
                joint.perturb(0.3);
                start = Clock::now();
                ASSERT_TRUE(split.solve());
                splitWarm.push_back(elapsed(start));
                start = Clock::now();
                ASSERT_TRUE(baseline.solve());
                jointWarm.push_back(elapsed(start));
                ASSERT_TRUE(split.satisfied());
            }
            std::cout << "articulation_bench shape=" << shape << " points=" << size << " blocks=" << blocks
                << " full_variables=" << 2 * (size - 1) << " largest_block_variables=" << largestTask
                << " topology_ms=" << median(topology) << " sequential_ms=" << median(sequential)
                << " split_cold_ms=" << median(splitCold) << " joint_cold_ms=" << median(jointCold)
                << " split_warm_ms=" << median(splitWarm) << " joint_warm_ms=" << median(jointWarm) << '\n';
        }
    }
}

TEST(ArticulationSolveBenchmark, RejectedSplitAndJointFallback) {
    std::vector<double> splitCold, jointCold, splitWarm, jointWarm;
    for (int repeat = 0; repeat < 5; ++repeat) {
        DCMManager split, joint;
        const auto ids = intersection(split, true);
        joint.restoreSnapshot(split.snapshot());
        auto start = Clock::now();
        ASSERT_TRUE(split.solve(split.getComponentForFigure(ids[0])));
        splitCold.push_back(elapsed(start));
        start = Clock::now();
        JointSolve baseline(joint);
        ASSERT_TRUE(baseline.solve());
        jointCold.push_back(elapsed(start));
        split.updatePoint({ids[1], 3, 2});
        joint.updatePoint({ids[1], 3, 2});
        start = Clock::now();
        ASSERT_TRUE(split.solve(split.getComponentForFigure(ids[0])));
        splitWarm.push_back(elapsed(start));
        start = Clock::now();
        ASSERT_TRUE(baseline.solve());
        jointWarm.push_back(elapsed(start));
    }
    std::cout << "articulation_fallback_bench split_cold_ms=" << median(splitCold)
        << " joint_cold_ms=" << median(jointCold) << " split_warm_ms=" << median(splitWarm)
        << " joint_warm_ms=" << median(jointWarm) << '\n';
}

TEST(ArticulationSolveBenchmark, WarmDragComparedToJointSolveWithSameLocks) {
    for (std::size_t size : {128, 512}) {
        std::vector<double> splitTimes, jointTimes;
        for (int repeat = 0; repeat < 5; ++repeat) {
            Model split(size, "star", false), joint(size, "star", false);
            ASSERT_TRUE(split.solve());
            const auto variables = joint.manager.getRequirementSystem().getFigureVariables(joint.points.back());
            JointSolve baseline(joint.manager, std::unordered_set<VAR>(variables.begin(), variables.end()));
            ASSERT_TRUE(baseline.solve());
            split.manager.setSolveMode(SolveMode::DRAG);
            const double x = 5 * (size - 1) + 1;
            auto start = Clock::now();
            ASSERT_TRUE(split.manager.updatePoint({split.points.back(), x, 1}));
            splitTimes.push_back(elapsed(start));
            joint.manager.updatePoint({joint.points.back(), x, 1});
            start = Clock::now();
            ASSERT_TRUE(baseline.solve());
            jointTimes.push_back(elapsed(start));
            EXPECT_DOUBLE_EQ(*split.manager.getFigure(split.points.back())->x, x);
            EXPECT_DOUBLE_EQ(*split.manager.getFigure(split.points.back())->y, 1);
            ASSERT_TRUE(split.satisfied());
        }
        std::cout << "articulation_drag_bench points=" << size << " split_ms=" << median(splitTimes)
            << " joint_locked_ms=" << median(jointTimes) << '\n';
    }
}
