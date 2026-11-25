#include <gtest/gtest.h>
#include "DCMManager.h"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {

class DCMManagerComponentSolveTest : public ::testing::Test {
protected:
    DCMManager manager;

    void expectPoint(ID id, double x, double y) const {
        const auto point = manager.getFigure(id);
        ASSERT_TRUE(point && point->x && point->y);
        EXPECT_DOUBLE_EQ(*point->x, x);
        EXPECT_DOUBLE_EQ(*point->y, y);
    }

    void expectDistance(ID first, ID second, double distance) const {
        const auto a = manager.getFigure(first);
        const auto b = manager.getFigure(second);
        ASSERT_TRUE(a && b && a->x && a->y && b->x && b->y);
        EXPECT_NEAR(std::hypot(*a->x - *b->x, *a->y - *b->y), distance, 1e-6);
    }

    void expectUnchanged(const std::vector<FigureDescriptor>& before) const {
        for (const auto& original : before) {
            ASSERT_TRUE(original.id);
            SCOPED_TRACE(original.id->id);
            const auto current = manager.getFigure(*original.id);
            ASSERT_TRUE(current);
            EXPECT_EQ(current->pointIds, original.pointIds);
            EXPECT_EQ(current->coords, original.coords);
            EXPECT_EQ(current->x, original.x);
            EXPECT_EQ(current->y, original.y);
            EXPECT_EQ(current->radius, original.radius);
        }
    }

    void expectComponent(ID member, std::vector<ID> figures, std::vector<ID> requirements) const {
        const auto component = manager.getComponentForFigure(member);
        ASSERT_TRUE(component);
        for (const auto id : figures) EXPECT_EQ(manager.getComponentForFigure(id), component);
        auto actualFigures = manager.getFiguresInComponent(*component);
        auto actualRequirements = manager.getRequirementsInComponent(*component);
        std::sort(figures.begin(), figures.end());
        std::sort(requirements.begin(), requirements.end());
        std::sort(actualFigures.begin(), actualFigures.end());
        std::sort(actualRequirements.begin(), actualRequirements.end());
        EXPECT_EQ(actualFigures, figures);
        EXPECT_EQ(actualRequirements, requirements);
    }
};

TEST_F(DCMManagerComponentSolveTest, TriangleWithoutArticulationsSolvesInEveryMode) {
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto b = manager.addFigure(FigureDescriptor::point(3, 0));
    const auto c = manager.addFigure(FigureDescriptor::point(1, 3));
    manager.addRequirement(RequirementDescriptor::fixPoint(a));
    manager.addRequirement(RequirementDescriptor::fixPoint(b));
    manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 3));
    manager.addRequirement(RequirementDescriptor::pointPointDist(a, c, 5));
    manager.addRequirement(RequirementDescriptor::pointPointDist(b, c, 4));
    const auto initial = manager.snapshot();

    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        SCOPED_TRACE(static_cast<int>(mode));
        manager.restoreSnapshot(initial);
        manager.setSolveMode(mode);
        const auto component = manager.getComponentForFigure(a);
        ASSERT_TRUE(component);
        ASSERT_EQ(manager.getComponentCount(), 1U);
        for (int attempt = 0; attempt < 2; ++attempt) {
            ASSERT_TRUE(manager.solve(*component, 1e-6));
            expectPoint(a, 0, 0);
            expectPoint(b, 3, 0);
            expectDistance(a, b, 3);
            expectDistance(a, c, 5);
            expectDistance(b, c, 4);
        }
    }
}

TEST_F(DCMManagerComponentSolveTest, ScopedSolvesKeepUnselectedComponentsAfterParameterChanges) {
    const auto circle = manager.addFigure(FigureDescriptor::circle(100, 100, 3));
    manager.addRequirement(RequirementDescriptor::circleRadius(circle, 7));
    const auto p = manager.addFigure(FigureDescriptor::point(200, 0));
    const auto q = manager.addFigure(FigureDescriptor::point(205, 0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p));
    manager.addRequirement(RequirementDescriptor::fixPoint(q));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p, q, 10));
    // One unselected component is solvable but unsatisfied, another is contradictory.
    const auto untouched = manager.getAllFigures();
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto b = manager.addFigure(FigureDescriptor::point(10, 0));
    const auto fixed = manager.addRequirement(RequirementDescriptor::fixPoint(a));
    const auto distance = manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto component = manager.getComponentForFigure(a);
    ASSERT_TRUE(component);

    for (const auto mode : {SolveMode::LOCAL, SolveMode::DRAG}) {
        SCOPED_TRACE(static_cast<int>(mode));
        manager.setSolveMode(mode);
        for (const double target : {12.0, 6.0}) {
            manager.updateRequirementParam(distance, target);
            for (int attempt = 0; attempt < 2; ++attempt) {
                ASSERT_TRUE(manager.solve(*component, 1e-6));
                expectPoint(a, 0, 0);
                expectDistance(a, b, target);
                expectUnchanged(untouched);
                EXPECT_EQ(manager.getComponentCount(), 3U);
                EXPECT_EQ(manager.getComponentForFigure(a), component);
                expectComponent(a, {a, b}, {fixed, distance});
            }
        }
    }
}

TEST_F(DCMManagerComponentSolveTest, GlobalSolveWithComponentIdStillSolvesAllComponents) {
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto b = manager.addFigure(FigureDescriptor::point(8, 0));
    manager.addRequirement(RequirementDescriptor::fixPoint(a));
    manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto circle = manager.addFigure(FigureDescriptor::circle(100, 100, 3));
    manager.addRequirement(RequirementDescriptor::circleRadius(circle, 7));
    const auto component = manager.getComponentForFigure(a);
    ASSERT_TRUE(component);
    manager.setSolveMode(SolveMode::GLOBAL);

    for (const double radius : {3.0, 4.0}) {
        ASSERT_TRUE(manager.updateCircle({circle, radius}));
        ASSERT_TRUE(manager.solve(*component, 1e-6));
        expectPoint(a, 0, 0);
        expectDistance(a, b, 10);
        const auto result = manager.getFigure(circle);
        ASSERT_TRUE(result && result->radius);
        EXPECT_NEAR(*result->radius, 7, 1e-6);
        EXPECT_EQ(manager.getComponentCount(), 2U);
    }
}

TEST_F(DCMManagerComponentSolveTest, BatchDragSolvesOnlyTouchedComponents) {
    const auto circle = manager.addFigure(FigureDescriptor::circle(200, 200, 3));
    manager.addRequirement(RequirementDescriptor::circleRadius(circle, 7));
    const auto untouched = manager.getAllFigures();
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto b = manager.addFigure(FigureDescriptor::point(10, 0));
    const auto c = manager.addFigure(FigureDescriptor::point(100, 100));
    const auto d = manager.addFigure(FigureDescriptor::point(110, 100));
    const auto ab = manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto cd = manager.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    manager.setSolveMode(SolveMode::DRAG);

    for (const double offset : {2.0, 4.0}) {
        ASSERT_TRUE(manager.updatePoints({{a, offset, 3}, {c, 100 + offset, 103}}));
        expectPoint(a, offset, 3);
        expectPoint(c, 100 + offset, 103);
        expectDistance(a, b, 10);
        expectDistance(c, d, 10);
        expectUnchanged(untouched);
        EXPECT_EQ(manager.getComponentCount(), 3U);
        expectComponent(a, {a, b}, {ab});
        expectComponent(c, {c, d}, {cd});
    }
}

TEST_F(DCMManagerComponentSolveTest, CachedComponentsFollowMergeAndSplitOnConstraintChanges) {
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto b = manager.addFigure(FigureDescriptor::point(10, 0));
    const auto c = manager.addFigure(FigureDescriptor::point(20, 0));
    const auto d = manager.addFigure(FigureDescriptor::point(30, 0));
    const auto ab = manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto cd = manager.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    manager.setSolveMode(SolveMode::DRAG);
    ASSERT_TRUE(manager.updatePoints({{a, 2, 0}, {c, 22, 0}}));

    const auto bridge = manager.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    ASSERT_EQ(manager.getComponentCount(), 1U);
    expectComponent(a, {a, b, c, d}, {ab, cd, bridge});
    ASSERT_TRUE(manager.updatePoint({a, 4, 0}));
    expectPoint(a, 4, 0);
    expectDistance(a, b, 10);
    expectDistance(b, c, 10);
    expectDistance(c, d, 10);

    manager.removeRequirement(bridge);
    ASSERT_EQ(manager.getComponentCount(), 2U);
    expectComponent(a, {a, b}, {ab});
    expectComponent(c, {c, d}, {cd});
    ASSERT_NE(manager.getComponentForFigure(a), manager.getComponentForFigure(c));
    // Leave the separated component unsatisfied to detect accidental solving of it.
    manager.updateRequirementParam(cd, 20);
    const auto cBefore = manager.getFigure(c);
    const auto dBefore = manager.getFigure(d);
    ASSERT_TRUE(cBefore && dBefore);
    for (const double x : {6.0, 8.0}) {
        ASSERT_TRUE(manager.updatePoint({a, x, 0}));
        expectPoint(a, x, 0);
        expectDistance(a, b, 10);
        expectUnchanged({*cBefore, *dBefore});
        expectComponent(a, {a, b}, {ab});
        expectComponent(c, {c, d}, {cd});
    }
}

TEST_F(DCMManagerComponentSolveTest, ManualContradictorySolveKeepsFiniteCandidateInEveryMode) {
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto b = manager.addFigure(FigureDescriptor::point(20, 0));
    manager.addRequirement(RequirementDescriptor::fixPoint(a));
    manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto initial = manager.snapshot();

    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        SCOPED_TRACE(static_cast<int>(mode));
        manager.restoreSnapshot(initial);
        manager.setSolveMode(mode);
        const auto component = manager.getComponentForFigure(a);
        ASSERT_TRUE(component);
        for (int attempt = 0; attempt < 2; ++attempt) {
            EXPECT_FALSE(manager.solve(*component, 1e-6));
            expectPoint(a, 0, 0);
            const auto candidate = manager.getFigure(b);
            ASSERT_TRUE(candidate && candidate->x && candidate->y);
            ASSERT_TRUE(std::isfinite(*candidate->x) && std::isfinite(*candidate->y));
            // Ordinary solve retains a finite candidate even when constraints conflict.
            const double distance = std::hypot(*candidate->x, *candidate->y);
            EXPECT_GT(distance, 5);
            EXPECT_LT(distance, 10);
        }
    }
}

TEST_F(DCMManagerComponentSolveTest, FailedContradictoryDragRestoresBatchAndCanRecover) {
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto b = manager.addFigure(FigureDescriptor::point(10, 0));
    const auto c = manager.addFigure(FigureDescriptor::point(100, 0));
    const auto d = manager.addFigure(FigureDescriptor::point(110, 0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto conflict = manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    manager.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    const auto circle = manager.addFigure(FigureDescriptor::circle(200, 200, 3));
    manager.addRequirement(RequirementDescriptor::circleRadius(circle, 7));
    manager.setSolveMode(SolveMode::DRAG);
    const auto before = manager.getAllFigures();

    for (int attempt = 0; attempt < 2; ++attempt) {
        EXPECT_FALSE(manager.updatePoints({{a, 2, 0}, {c, 102, 0}}));
        expectUnchanged(before);
        EXPECT_EQ(manager.getComponentCount(), 3U);
    }

    manager.removeRequirement(conflict);
    ASSERT_TRUE(manager.updatePoints({{a, 2, 0}, {c, 102, 0}}));
    expectPoint(a, 2, 0);
    expectPoint(c, 102, 0);
    expectDistance(a, b, 10);
    expectDistance(c, d, 10);
    const auto unchangedCircle = manager.getFigure(circle);
    ASSERT_TRUE(unchangedCircle && unchangedCircle->radius);
    EXPECT_DOUBLE_EQ(*unchangedCircle->radius, 3);
}

} // namespace
