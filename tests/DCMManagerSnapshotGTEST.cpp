#include <gtest/gtest.h>

#include "DCMManager.h"

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerSnapshotTest, RestoresCascadeDeletedGeometryConstraintsAndIds) {
    DCMManager manager;

    const ID lineId = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 10.0, 0.0));
    const auto line = manager.getFigure(lineId);
    ASSERT_TRUE(line.has_value());
    ASSERT_EQ(line->pointIds.size(), 2U);

    const ID firstPointId = line->pointIds[0];
    const ID secondPointId = line->pointIds[1];
    const ID fixedPointRequirement = manager.addRequirement(RequirementDescriptor::fixPoint(firstPointId));
    const ID horizontalRequirement = manager.addRequirement(RequirementDescriptor::horizontal(lineId));
    const auto state = manager.snapshot();

    manager.removeFigure(lineId, true);
    EXPECT_EQ(manager.figureCount(), 0U);
    EXPECT_EQ(manager.requirementCount(), 0U);

    manager.restoreSnapshot(state);

    EXPECT_EQ(manager.figureCount(), 3U);
    EXPECT_EQ(manager.requirementCount(), 2U);
    EXPECT_TRUE(manager.hasFigure(firstPointId));
    EXPECT_TRUE(manager.hasFigure(secondPointId));
    EXPECT_TRUE(manager.hasFigure(lineId));
    EXPECT_TRUE(manager.hasRequirement(fixedPointRequirement));
    EXPECT_TRUE(manager.hasRequirement(horizontalRequirement));
    EXPECT_EQ(manager.getRequirementSystem().getRequirementCount(), 2U);

    const auto restoredLine = manager.getFigure(lineId);
    ASSERT_TRUE(restoredLine.has_value());
    EXPECT_EQ(restoredLine->pointIds, line->pointIds);
    EXPECT_DOUBLE_EQ(restoredLine->coords[0], 0.0);
    EXPECT_DOUBLE_EQ(restoredLine->coords[1], 0.0);
    EXPECT_DOUBLE_EQ(restoredLine->coords[2], 10.0);
    EXPECT_DOUBLE_EQ(restoredLine->coords[3], 0.0);

    const auto restoredFix = manager.getRequirement(fixedPointRequirement);
    ASSERT_TRUE(restoredFix.has_value());
    EXPECT_EQ(restoredFix->objectIds, std::vector<ID>{firstPointId});

    const ID nextId = manager.addFigure(FigureDescriptor::point(20.0, 20.0));
    EXPECT_EQ(nextId, ID(4));
}

TEST(DCMManagerSnapshotTest, RoundTripsIndependentSnapshots) {
    DCMManager manager;
    const ID pointId = manager.addFigure(FigureDescriptor::point(1.0, 2.0));
    manager.setSolveMode(SolveMode::DRAG);
    const auto beforeMove = manager.snapshot();

    manager.updatePoint(PointUpdateDescriptor(pointId, 10.0, 20.0));
    manager.setSolveMode(SolveMode::LOCAL);
    const auto afterMove = manager.snapshot();

    manager.restoreSnapshot(beforeMove);
    const auto restoredBefore = manager.getFigure(pointId);
    ASSERT_TRUE(restoredBefore.has_value());
    EXPECT_DOUBLE_EQ(restoredBefore->x.value(), 1.0);
    EXPECT_DOUBLE_EQ(restoredBefore->y.value(), 2.0);
    EXPECT_EQ(manager.getSolveMode(), SolveMode::DRAG);

    manager.restoreSnapshot(afterMove);
    const auto restoredAfter = manager.getFigure(pointId);
    ASSERT_TRUE(restoredAfter.has_value());
    EXPECT_DOUBLE_EQ(restoredAfter->x.value(), 10.0);
    EXPECT_DOUBLE_EQ(restoredAfter->y.value(), 20.0);
    EXPECT_EQ(manager.getSolveMode(), SolveMode::LOCAL);
}

TEST(DCMManagerSnapshotTest, PreservesFixedRequirementTargets) {
    DCMManager manager;
    const ID pointId = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(pointId));

    manager.updatePoint(PointUpdateDescriptor(pointId, 5.0, 7.0));
    const auto state = manager.snapshot();

    manager.clear();
    manager.restoreSnapshot(state);
    EXPECT_TRUE(manager.solve());

    const auto restoredPoint = manager.getFigure(pointId);
    ASSERT_TRUE(restoredPoint.has_value());
    EXPECT_NEAR(restoredPoint->x.value(), 0.0, 1e-6);
    EXPECT_NEAR(restoredPoint->y.value(), 0.0, 1e-6);
}
