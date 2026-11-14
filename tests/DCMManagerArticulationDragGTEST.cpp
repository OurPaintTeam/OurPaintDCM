#include <gtest/gtest.h>
#include "DCMManager.h"
#include <algorithm>
#include <cmath>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
class ArticulationDragTest : public ::testing::Test {
protected:
    DCMManager manager;
    ID a = manager.addFigure(FigureDescriptor::point(0, 0));
    ID b = manager.addFigure(FigureDescriptor::point(10, 0));
    ID c = manager.addFigure(FigureDescriptor::point(20, 0));

    void SetUp() override {
        manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
        manager.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
        manager.setSolveMode(SolveMode::DRAG);
    }

    void expectRootProjection(double oldX, double oldY, double x, double y) {
        const double scale = 10 / std::hypot(oldX - x, oldY - y);
        const auto middle = manager.getFigure(b).value();
        EXPECT_NEAR(*middle.x, x + (oldX - x) * scale, 1e-6);
        EXPECT_NEAR(*middle.y, y + (oldY - y) * scale, 1e-6);
        for (auto endpoint : {a, c}) {
            const auto point = manager.getFigure(endpoint).value();
            EXPECT_NEAR(std::hypot(*point.x - *middle.x, *point.y - *middle.y), 10, 1e-6);
        }
    }
};

TEST_F(ArticulationDragTest, PointApisStartWithEditedBranchAndDoNotKeepPreviousEdit) {
    const auto initial = manager.snapshot();
    for (int api = 0; api < 4; ++api) {
        SCOPED_TRACE(api);
        manager.restoreSnapshot(initial);
        const auto circle = manager.addFigure(FigureDescriptor::circle(100, 100, 2));
        manager.addRequirement(RequirementDescriptor::circleRadius(circle, 7));
        for (auto endpoint : {c, a, c}) {
            const auto before = manager.getFigure(b).value();
            const double x = endpoint == c ? 20 : 0;
            const double y = endpoint == c ? 10 : -10;
            switch (api) {
                case 0: ASSERT_TRUE(manager.updatePoint({endpoint, x, y})); break;
                case 1: ASSERT_TRUE(manager.updatePoints({{endpoint, x, y}})); break;
                case 2: ASSERT_TRUE(manager.updateFigure(FigureUpdateDescriptor::point(endpoint, x, y))); break;
                case 3: ASSERT_TRUE(manager.updateFigures({FigureUpdateDescriptor::point(endpoint, x, y)})); break;
            }
            EXPECT_DOUBLE_EQ(*manager.getFigure(endpoint)->x, x);
            EXPECT_DOUBLE_EQ(*manager.getFigure(endpoint)->y, y);
            expectRootProjection(*before.x, *before.y, x, y);
            EXPECT_DOUBLE_EQ(*manager.getFigure(circle)->radius, 2);
        }
    }
}

TEST_F(ArticulationDragTest, CoincidentDraggedPointSelectsBranchUsingRepresentativeParameters) {
    const auto alias = manager.addFigure(FigureDescriptor::point(20, 0));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(c, alias));
    ASSERT_TRUE(manager.updatePoint({alias, 20, 10}));
    EXPECT_DOUBLE_EQ(*manager.getFigure(alias)->y, 10);
    EXPECT_DOUBLE_EQ(*manager.getFigure(c)->y, 10);
    expectRootProjection(10, 0, 20, 10);
}

TEST_F(ArticulationDragTest, PointTargetsAlsoSelectEditedBranch) {
    const std::vector<PointTarget> targets{{c, 20, 10}};
    const auto result = manager.solveWithTargets(targets);
    ASSERT_TRUE(result.converged);
    EXPECT_TRUE(result.targetsSatisfied);
    EXPECT_FALSE(result.usedFallback);
    expectRootProjection(10, 0, 20, 10);
    EXPECT_EQ(manager.getSolveMode(), SolveMode::DRAG);
}

TEST_F(ArticulationDragTest, ManualModesRememberEditsUntilSelectedComponentIsSolved) {
    const auto initial = manager.snapshot();
    for (auto mode : {SolveMode::LOCAL, SolveMode::GLOBAL}) {
        manager.restoreSnapshot(initial);
        manager.setSolveMode(mode);
        for (auto endpoint : {c, a, c}) {
            const auto before = manager.getFigure(b).value();
            const double x = endpoint == c ? 20 : 0;
            const double y = endpoint == c ? 10 : -10;
            ASSERT_TRUE(manager.updatePoint({endpoint, x, y}));
            EXPECT_EQ(manager.getFigure(b)->x, before.x);
            EXPECT_EQ(manager.getFigure(b)->y, before.y);
            const double length = std::hypot(x - *before.x, y - *before.y);
            const double shift = (length - 10) / (2 * length);
            ASSERT_TRUE(manager.solve(manager.getComponentForFigure(endpoint)));
            EXPECT_NEAR(*manager.getFigure(b)->x, *before.x + (x - *before.x) * shift, 1e-6);
            EXPECT_NEAR(*manager.getFigure(b)->y, *before.y + (y - *before.y) * shift, 1e-6);
        }
    }
}

TEST_F(ArticulationDragTest, BatchChoosesEditedBranchForEachTouchedComponent) {
    const auto p = manager.addFigure(FigureDescriptor::point(100, 0));
    const auto q = manager.addFigure(FigureDescriptor::point(110, 0));
    const auto r = manager.addFigure(FigureDescriptor::point(120, 0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p, q, 10));
    manager.addRequirement(RequirementDescriptor::pointPointDist(q, r, 10));
    const auto initial = manager.snapshot();
    for (bool reverse : {false, true}) {
        manager.restoreSnapshot(initial);
        std::vector<PointUpdateDescriptor> updates{{c, 20, 10}, {p, 100, 10}};
        if (reverse) std::reverse(updates.begin(), updates.end());
        ASSERT_TRUE(manager.updatePoints(updates));
        expectRootProjection(10, 0, 20, 10);
        EXPECT_NEAR(*manager.getFigure(q)->x, 100 + 10 / std::sqrt(2.0), 1e-6);
        EXPECT_NEAR(*manager.getFigure(q)->y, 10 - 10 / std::sqrt(2.0), 1e-6);
        EXPECT_DOUBLE_EQ(*manager.getFigure(p)->x, 100);
        EXPECT_DOUBLE_EQ(*manager.getFigure(p)->y, 10);
        EXPECT_EQ(manager.getComponentCount(), 2U);
    }
}

TEST_F(ArticulationDragTest, TypedAndUnifiedLineUpdatesIdentifyChangedEndpoint) {
    const auto end = manager.addFigure(FigureDescriptor::point(30, 0));
    const auto line = manager.addFigure(FigureDescriptor::line(c, end));
    const auto initial = manager.snapshot();
    for (int api = 0; api < 4; ++api) {
        SCOPED_TRACE(api);
        manager.restoreSnapshot(initial);
        switch (api) {
            case 0: ASSERT_TRUE(manager.updateLine({line, 20, 10, 30, 10})); break;
            case 1: ASSERT_TRUE(manager.updateLines({{line, 20, 10, 30, 10}})); break;
            case 2: ASSERT_TRUE(manager.updateFigure(FigureUpdateDescriptor::line(line, 20, 10, 30, 10))); break;
            case 3: ASSERT_TRUE(manager.updateFigures({FigureUpdateDescriptor::line(line, 20, 10, 30, 10)})); break;
        }
        expectRootProjection(10, 0, 20, 10);
        EXPECT_EQ(manager.getFigure(line)->coords, (std::vector<double>{20, 10, 30, 10}));
    }
}

TEST_F(ArticulationDragTest, CircleCenterUpdatesIdentifyChangedBranch) {
    const auto circle = manager.addFigure(FigureDescriptor::circle(c, 2));
    const auto initial = manager.snapshot();
    for (int api = 0; api < 4; ++api) {
        manager.restoreSnapshot(initial);
        switch (api) {
            case 0: ASSERT_TRUE(manager.updateCircle(CircleUpdateDescriptor::center(circle, 20, 10))); break;
            case 1: ASSERT_TRUE(manager.updateCircles({CircleUpdateDescriptor::center(circle, 20, 10)})); break;
            case 2: ASSERT_TRUE(manager.updateFigure(FigureUpdateDescriptor::circleCenter(circle, 20, 10))); break;
            case 3: ASSERT_TRUE(manager.updateFigures({FigureUpdateDescriptor::circleCenter(circle, 20, 10)})); break;
        }
        expectRootProjection(10, 0, 20, 10);
        EXPECT_DOUBLE_EQ(*manager.getFigure(circle)->radius, 2);
    }
}

TEST_F(ArticulationDragTest, ArcAndCurveUpdatesPropagateEditedDefiningPoints) {
    const auto end = manager.addFigure(FigureDescriptor::point(30, 0));
    const auto center = manager.addFigure(FigureDescriptor::point(25, 5));
    const auto arc = manager.addFigure(FigureDescriptor::arc(c, end, center));
    const auto curve = manager.addFigure(FigureDescriptor::curve(1, std::vector<ID>{c, end}, {1, 1}, {0, 0, 1, 1}));
    const auto initial = manager.snapshot();
    for (int api = 0; api < 6; ++api) {
        SCOPED_TRACE(api);
        manager.restoreSnapshot(initial);
        switch (api) {
            case 0: ASSERT_TRUE(manager.updateArc({arc, 20, 10, 30, 10, 25, 15})); break;
            case 1: ASSERT_TRUE(manager.updateArcs({{arc, 20, 10, 30, 10, 25, 15}})); break;
            case 2: ASSERT_TRUE(manager.updateFigure(FigureUpdateDescriptor::arc(arc, 20, 10, 30, 10, 25, 15))); break;
            case 3: ASSERT_TRUE(manager.updateFigures({FigureUpdateDescriptor::arc(arc, 20, 10, 30, 10, 25, 15)})); break;
            // Keep the defining arc valid while editing the shared curve controls.
            case 4: ASSERT_TRUE(manager.updateFigures({FigureUpdateDescriptor::curve(curve, {20, 10, 30, 10}),
                                                      FigureUpdateDescriptor::point(center, 25, 15)})); break;
            case 5: ASSERT_TRUE(manager.updateFigures({FigureUpdateDescriptor::point(center, 25, 15),
                                                      FigureUpdateDescriptor::curve(curve, {20, 10, 30, 10})})); break;
        }
        expectRootProjection(10, 0, 20, 10);
        EXPECT_EQ(manager.getFigure(curve)->coords, (std::vector<double>{20, 10, 30, 10}));
    }
}

TEST(ArticulationDragRadiusTest, RadiusOnlyUpdatesPropagateAcrossSharedCircleNode) {
    for (int api = 0; api < 4; ++api) {
        DCMManager manager;
        const auto a = manager.addFigure(FigureDescriptor::circle(0, 0, 2));
        const auto b = manager.addFigure(FigureDescriptor::circle(10, 0, 2));
        const auto c = manager.addFigure(FigureDescriptor::circle(20, 0, 2));
        manager.addRequirement(RequirementDescriptor::equalRadius(a, b));
        manager.addRequirement(RequirementDescriptor::equalRadius(b, c));
        manager.setSolveMode(SolveMode::DRAG);
        for (double radius : {5.0, 3.0}) {
            switch (api) {
                case 0: ASSERT_TRUE(manager.updateCircle({c, radius})); break;
                case 1: ASSERT_TRUE(manager.updateCircles({{c, radius}})); break;
                case 2: ASSERT_TRUE(manager.updateFigure(FigureUpdateDescriptor::circleRadius(c, radius))); break;
                case 3: ASSERT_TRUE(manager.updateFigures({FigureUpdateDescriptor::circleRadius(c, radius)})); break;
            }
            for (auto id : {a, b, c}) EXPECT_NEAR(*manager.getFigure(id)->radius, radius, 1e-6);
        }
    }
}
} // namespace
