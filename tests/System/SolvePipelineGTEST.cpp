#include <gtest/gtest.h>
#include "SolvePipeline.h"
#include "RequirementSystem.h"
#include "SparseLSMTask.h"
#include "SparseSystemTask.h"
#include "sparse/SparseLevenbergMarquardtSolver.h"

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

namespace {
class SolvePipelineTest : public ::testing::Test {
protected:
    GeometryStorage storage;
    ID a = storage.createPoint(0, 0);
    ID b = storage.createPoint(3, 0);
    RequirementSystem system{&storage, std::vector<ID>{a, b}};
};

TEST_F(SolvePipelineTest, LockedCoordinatesAreConstantsDuringOptimization) {
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    auto* point = storage.get<Point2D>(a);
    auto pipeline = buildSolvePipeline(system.getFunctions(), {}, {point->ptrX(), point->ptrY()},
                                      SolverType::LEVENBERG_MARQUARDT);
    ASSERT_TRUE(pipeline.task);
    EXPECT_EQ(pipeline.variableOwners.size(), 2U);
    SparseLMSolver solver;
    solver.setTask(pipeline.task.get());
    solver.optimize();
    EXPECT_DOUBLE_EQ(point->x(), 0);
    EXPECT_DOUBLE_EQ(point->y(), 0);
    EXPECT_NEAR(storage.get<Point2D>(b)->x(), 5, 1e-4);
}

TEST_F(SolvePipelineTest, FixedAssignmentsFromOtherBlockAreAlsoExcluded) {
    system.addRequirement(RequirementDescriptor::fixPoint(a));
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    const auto fixed = collectFixedAssignments(system.getFunctions());
    const std::vector bindings{system.getFunctions().back()};
    auto pipeline = buildSolvePipeline(bindings, fixed, {}, SolverType::DOGLEG);
    ASSERT_TRUE(pipeline.task);
    EXPECT_EQ(pipeline.variableOwners.size(), 2U);
    pipeline.task->setError({5, 0});
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(a)->x(), 0);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->x(), 5);
}

TEST_F(SolvePipelineTest, OnlyPreviouslyDeterminedBoundaryIsLocked) {
    const auto c = storage.createPoint(8, 0);
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 5));
    auto* point = storage.get<Point2D>(a);
    auto pipeline = buildSolvePipeline(system.getFunctions(), {}, {point->ptrX(), point->ptrY()},
                                      SolverType::GAUSS_NEWTON);
    EXPECT_EQ(pipeline.variableOwners.size(), 4U);
    ASSERT_TRUE(pipeline.task);
    pipeline.task->setError({5, 0, 10, 0});
    EXPECT_DOUBLE_EQ(point->x(), 0);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->x(), 5);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(c)->x(), 10);
}

TEST_F(SolvePipelineTest, ConstantsRemainLiveInputsForRepeatedEvaluations) {
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    auto* point = storage.get<Point2D>(a);
    auto pipeline = buildSolvePipeline(system.getFunctions(), {}, {point->ptrX(), point->ptrY()},
                                      SolverType::LEVENBERG_MARQUARDT);
    ASSERT_TRUE(pipeline.task);
    EXPECT_DOUBLE_EQ(pipeline.task->setError({5, 0}), 0);
    point->x() = 2;
    EXPECT_GT(pipeline.task->setError({5, 0}), 0);
    EXPECT_DOUBLE_EQ(point->x(), 2);
}

TEST_F(SolvePipelineTest, AllLockedAndFixOnlyTasksNeedNoOptimizer) {
    system.addRequirement(RequirementDescriptor::fixPoint(a));
    auto fixed = collectFixedAssignments(system.getFunctions());
    auto fixOnly = buildSolvePipeline(system.getFunctions(), fixed, {}, SolverType::NEWTON);
    EXPECT_FALSE(fixOnly.hasFunctions);
    EXPECT_FALSE(fixOnly.task);
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    auto* point = storage.get<Point2D>(b);
    auto locked = buildSolvePipeline(system.getFunctions(), fixed, {point->ptrX(), point->ptrY()}, SolverType::NEWTON);
    EXPECT_TRUE(locked.hasFunctions);
    EXPECT_FALSE(locked.hasFreeVariables);
    EXPECT_FALSE(locked.task);
}

TEST_F(SolvePipelineTest, UsesExistingTaskTypeForEachSolver) {
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    for (auto solver : {SolverType::LEVENBERG_MARQUARDT, SolverType::DOGLEG,
                        SolverType::GAUSS_NEWTON, SolverType::NEWTON}) {
        auto pipeline = buildSolvePipeline(system.getFunctions(), {}, {}, solver);
        ASSERT_TRUE(pipeline.task);
        if (solver == SolverType::NEWTON) EXPECT_NE(dynamic_cast<SparseSystemTask*>(pipeline.task.get()), nullptr);
        else EXPECT_NE(dynamic_cast<SparseLSMTask*>(pipeline.task.get()), nullptr);
    }
}
} // namespace
