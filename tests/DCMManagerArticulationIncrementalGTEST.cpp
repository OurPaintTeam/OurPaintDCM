#include <gtest/gtest.h>
#include "DCMManager.h"
#include "ArticulationDecomposition.h"
#include <cmath>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;
using namespace OurPaintDCM::System;

namespace {
class ArticulationIncrementalTest : public ::testing::Test {
protected:
    DCMManager manager;
    ID a = manager.addFigure(FigureDescriptor::point(0, 0));
    ID b = manager.addFigure(FigureDescriptor::point(5, 0));
    ID c = manager.addFigure(FigureDescriptor::point(10, 0));
    ID ab = manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    ID bc = manager.addRequirement(RequirementDescriptor::pointPointDist(b, c, 5));

    bool solve(ID member) { return manager.solve(manager.getComponentForFigure(member)); }
    double distance(ID first, ID second) {
        const auto p = manager.getFigure(first).value();
        const auto q = manager.getFigure(second).value();
        return std::hypot(*p.x - *q.x, *p.y - *q.y);
    }
    void expectSatisfied() {
        for (const auto& function : manager.getRequirementSystem().getFunctions())
            EXPECT_TRUE(function->mathematical()->satisfied(1e-6));
    }
    ArticulationDecomposition topology(ID member) {
        const auto component = manager.getComponentForFigure(member).value();
        auto graph = Figures::GeometryGraphBuilder::buildObjectGraph(
            manager.getStorage(), manager.getFiguresInComponent(component));
        for (auto id : manager.getRequirementsInComponent(component)) {
            const auto requirement = manager.getRequirement(id).value();
            for (std::size_t i = 0; i < requirement.objectIds.size(); ++i)
                for (std::size_t j = i + 1; j < requirement.objectIds.size(); ++j)
                    graph.addEdge(requirement.objectIds[i], requirement.objectIds[j], id);
        }
        return decomposeByArticulationPoints(graph);
    }
};

TEST_F(ArticulationIncrementalTest, ChangedDistancesRefreshBindingsAndKeepOtherComponentUntouched) {
    const auto circle = manager.addFigure(FigureDescriptor::circle(100, 100, 2));
    manager.addRequirement(RequirementDescriptor::circleRadius(circle, 7));
    for (auto mode : {SolveMode::LOCAL, SolveMode::DRAG}) {
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(a));
        for (double length : {7.0, 3.0, 5.0}) {
            manager.updateRequirementParam(bc, length);
            ASSERT_TRUE(solve(a));
            ASSERT_TRUE(solve(a));
            EXPECT_NEAR(distance(a, b), 5, 1e-6);
            EXPECT_NEAR(distance(b, c), length, 1e-6);
            EXPECT_DOUBLE_EQ(*manager.getFigure(circle)->radius, 2);
            EXPECT_EQ(topology(a).articulationPoints, (std::vector<ID>{b}));
        }
    }
}

TEST_F(ArticulationIncrementalTest, AddingCycleRemovesArticulationAndRemovalRestoresIt) {
    manager.setSolveMode(SolveMode::LOCAL);
    ASSERT_TRUE(solve(a));
    EXPECT_EQ(topology(a).articulationPoints, (std::vector<ID>{b}));
    const auto ac = manager.addRequirement(RequirementDescriptor::pointPointDist(a, c, 10));
    ASSERT_TRUE(solve(a));
    EXPECT_TRUE(topology(a).articulationPoints.empty());
    EXPECT_EQ(manager.getComponentCount(), 1U);
    manager.removeRequirement(ac);
    ASSERT_TRUE(solve(a));
    EXPECT_EQ(topology(a).articulationPoints, (std::vector<ID>{b}));
    manager.updateRequirementParam(bc, 8);
    ASSERT_TRUE(solve(a));
    EXPECT_NEAR(distance(a, b), 5, 1e-6);
    EXPECT_NEAR(distance(b, c), 8, 1e-6);
}

TEST_F(ArticulationIncrementalTest, SplitAndMergeRefreshComponentIdsAndCachedPlans) {
    manager.setSolveMode(SolveMode::LOCAL);
    ASSERT_TRUE(solve(a));
    manager.removeRequirement(bc);
    ASSERT_EQ(manager.getComponentCount(), 2U);
    ASSERT_TRUE(manager.updatePoint({c, 20, 4}));
    ASSERT_TRUE(solve(a));
    EXPECT_DOUBLE_EQ(*manager.getFigure(c)->x, 20);
    EXPECT_DOUBLE_EQ(*manager.getFigure(c)->y, 4);
    manager.addRequirement(RequirementDescriptor::pointPointDist(b, c, 8));
    ASSERT_EQ(manager.getComponentCount(), 1U);
    ASSERT_TRUE(solve(c));
    EXPECT_NEAR(distance(a, b), 5, 1e-6);
    EXPECT_NEAR(distance(b, c), 8, 1e-6);
}

TEST_F(ArticulationIncrementalTest, CoincidenceWeightChangesRebindSharedCoordinatePointers) {
    const auto alias = manager.addFigure(FigureDescriptor::point(10, 0));
    auto descriptor = RequirementDescriptor::pointOnPoint(c, alias);
    descriptor.weight = 0;
    const auto coincidence = manager.addRequirement(descriptor);
    const auto tail = manager.addFigure(FigureDescriptor::point(15, 0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(alias, tail, 5));
    manager.setSolveMode(SolveMode::DRAG);
    ASSERT_TRUE(solve(a));
    for (double weight : {1.0, 0.0, 1.0}) {
        manager.updateRequirementWeight(coincidence, weight);
        ASSERT_TRUE(manager.updatePoint({alias, 12, 3}));
        ASSERT_TRUE(solve(alias));
        EXPECT_NEAR(distance(a, b), 5, 1e-6);
        EXPECT_NEAR(distance(b, c), 5, 1e-6);
        EXPECT_NEAR(distance(alias, tail), 5, 1e-6);
        if (weight != 0) {
            EXPECT_EQ(manager.getFigure(c)->x, manager.getFigure(alias)->x);
            EXPECT_EQ(manager.getFigure(c)->y, manager.getFigure(alias)->y);
        }
    }
}

TEST_F(ArticulationIncrementalTest, ImplicitArcRowCountChangesDoNotLeaveStaleFunctionIndices) {
    const auto center = manager.addFigure(FigureDescriptor::point(5, 5));
    const auto arc = manager.addFigure(FigureDescriptor::arc(a, c, center));
    const auto perpendicular = manager.addRequirement(RequirementDescriptor::arcCenterOnPerpendicular(arc));
    manager.setSolveMode(SolveMode::LOCAL);
    ASSERT_TRUE(solve(a));
    for (double weight : {0.0, 1.0, 0.0}) {
        manager.updateRequirementWeight(perpendicular, weight);
        ASSERT_TRUE(solve(a));
        expectSatisfied();
    }
}

TEST_F(ArticulationIncrementalTest, DeletedGeometryAndReusedIdsDoNotReuseOldParametersOrEditMarks) {
    manager.setSolveMode(SolveMode::LOCAL);
    ASSERT_TRUE(solve(a));
    ASSERT_TRUE(manager.updatePoint({c, 12, 1}));
    manager.removeFigure(c);
    auto replacement = FigureDescriptor::point(10, 0);
    replacement.id = c;
    ASSERT_EQ(manager.addFigure(replacement), c);
    manager.addRequirement(RequirementDescriptor::pointPointDist(b, c, 5));
    ASSERT_TRUE(solve(a));
    EXPECT_NEAR(distance(a, b), 5, 1e-6);
    EXPECT_NEAR(distance(b, c), 5, 1e-6);
    EXPECT_EQ(topology(a).articulationPoints, (std::vector<ID>{b}));
}

TEST_F(ArticulationIncrementalTest, SolverChangesKeepPlanUsableAndDoNotRetainFrozenValues) {
    manager.setSolveMode(SolveMode::LOCAL);
    for (auto solver : {SolverType::LEVENBERG_MARQUARDT, SolverType::DOGLEG,
                        SolverType::GAUSS_NEWTON, SolverType::NEWTON}) {
        manager.setSolver(solver);
        ASSERT_TRUE(manager.updatePoint({c, 12, 2}));
        ASSERT_TRUE(solve(a));
        EXPECT_NEAR(distance(a, b), 5, 1e-6);
        EXPECT_NEAR(distance(b, c), 5, 1e-6);
    }
}

TEST_F(ArticulationIncrementalTest, SnapshotRestorationDropsPlansEvenWhenIdsAreReused) {
    manager.setSolveMode(SolveMode::LOCAL);
    ASSERT_TRUE(solve(a));
    const auto state = manager.snapshot();
    manager.updateRequirementParam(bc, 8);
    ASSERT_TRUE(solve(a));
    manager.restoreSnapshot(state);
    ASSERT_TRUE(solve(a));
    EXPECT_NEAR(distance(a, b), 5, 1e-6);
    EXPECT_NEAR(distance(b, c), 5, 1e-6);
}

TEST_F(ArticulationIncrementalTest, RejectedParameterChangeLeavesCachedPlanUsable) {
    manager.setSolveMode(SolveMode::LOCAL);
    ASSERT_TRUE(solve(a));
    EXPECT_THROW(manager.updateRequirementParam(bc, -1), std::invalid_argument);
    ASSERT_TRUE(solve(a));
    expectSatisfied();
}
} // namespace
