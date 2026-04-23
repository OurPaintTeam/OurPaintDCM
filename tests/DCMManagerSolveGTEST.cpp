#include <gtest/gtest.h>
#include "DCMManager.h"
#include <cmath>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

class DCMManagerSolveTest : public ::testing::Test {
protected:
    DCMManager manager;
};

TEST_F(DCMManagerSolveTest, GlobalSolve_PointPointDist) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(3.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));
    manager.setSolveMode(SolveMode::GLOBAL);

    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    double dx = d2->x.value() - d1->x.value();
    double dy = d2->y.value() - d1->y.value();
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 5.0, 0.1);
}

TEST_F(DCMManagerSolveTest, GlobalSolve_Horizontal) {
    auto line = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 5.0, 3.0));
    auto lineDesc = manager.getFigure(line);
    ASSERT_TRUE(lineDesc.has_value());
    auto p1 = lineDesc->pointIds[0];
    auto p2 = lineDesc->pointIds[1];

    manager.addRequirement(RequirementDescriptor::horizontal(line));
    manager.setSolveMode(SolveMode::GLOBAL);

    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    EXPECT_NEAR(d1->y.value(), d2->y.value(), 0.1);
}

TEST_F(DCMManagerSolveTest, LocalSolve_SingleComponent) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(1.0, 0.0));
    auto p3 = manager.addFigure(FigureDescriptor::point(100.0, 100.0));
    auto p4 = manager.addFigure(FigureDescriptor::point(103.0, 100.0));

    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p3, p4, 20.0));

    EXPECT_EQ(manager.getComponentCount(), 2);

    auto comp12 = manager.getComponentForFigure(p1);
    ASSERT_TRUE(comp12.has_value());

    auto d3before = manager.getFigure(p3);
    auto d4before = manager.getFigure(p4);

    manager.setSolveMode(SolveMode::LOCAL);
    bool converged = manager.solve(comp12.value());
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    double dx = d2->x.value() - d1->x.value();
    double dy = d2->y.value() - d1->y.value();
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 10.0, 0.1);

    auto d3after = manager.getFigure(p3);
    auto d4after = manager.getFigure(p4);
    ASSERT_TRUE(d3after.has_value() && d4after.has_value());
    EXPECT_DOUBLE_EQ(d3after->x.value(), d3before->x.value());
    EXPECT_DOUBLE_EQ(d3after->y.value(), d3before->y.value());
    EXPECT_DOUBLE_EQ(d4after->x.value(), d4before->x.value());
    EXPECT_DOUBLE_EQ(d4after->y.value(), d4before->y.value());
}

TEST_F(DCMManagerSolveTest, DragMode_AutoSolveOnUpdate) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));

    manager.setSolveMode(SolveMode::DRAG);
    manager.updatePoint(PointUpdateDescriptor(p1, 2.0, 0.0));

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    double dx = d2->x.value() - d1->x.value();
    double dy = d2->y.value() - d1->y.value();
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 5.0, 0.5);
}

TEST_F(DCMManagerSolveTest, DragMode_UpdateFixedPoint_RevertsToFixedCoordinates) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));

    manager.setSolveMode(SolveMode::DRAG);
    manager.updatePoint(PointUpdateDescriptor(p1, 2.0, 3.0));

    auto d1 = manager.getFigure(p1);
    ASSERT_TRUE(d1.has_value());
    EXPECT_NEAR(d1->x.value(), 0.0, 1e-6);
    EXPECT_NEAR(d1->y.value(), 0.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, DragMode_UpdateFixedCircleRadius_Ignored) {
    auto circle = manager.addFigure(FigureDescriptor::circle(0.0, 0.0, 10.0));
    manager.addRequirement(RequirementDescriptor::fixCircle(circle));
    manager.setSolveMode(SolveMode::DRAG);

    manager.updateCircle(CircleUpdateDescriptor(circle, 25.0));

    auto c = manager.getFigure(circle);
    ASSERT_TRUE(c.has_value());
    ASSERT_TRUE(c->radius.has_value());
    EXPECT_NEAR(c->radius.value(), 10.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, FixPointUsesCoordinatesCapturedAtConstraintCreation) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(20.0, 0.0));

    manager.updatePoint(PointUpdateDescriptor(p1, 5.0, 5.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    manager.setSolveMode(SolveMode::GLOBAL);

    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());
    EXPECT_NEAR(d1->x.value(), 5.0, 1e-6);
    EXPECT_NEAR(d1->y.value(), 5.0, 1e-6);

    const double dx = d2->x.value() - d1->x.value();
    const double dy = d2->y.value() - d1->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 10.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, GlobalSolve_PointOnPointUsesSingleRepresentativePoint) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    auto p3 = manager.addFigure(FigureDescriptor::point(14.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointOnPoint(p1, p2));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p3, 5.0));
    manager.setSolveMode(SolveMode::GLOBAL);

    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    auto d3 = manager.getFigure(p3);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value());
    EXPECT_NEAR(d1->x.value(), d2->x.value(), 1e-6);
    EXPECT_NEAR(d1->y.value(), d2->y.value(), 1e-6);

    const double dx = d3->x.value() - d2->x.value();
    const double dy = d3->y.value() - d2->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 5.0, 1e-4);
}

TEST_F(DCMManagerSolveTest, DragMode_UpdateAliasedPointKeepsDraggedCoordinates) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    auto p3 = manager.addFigure(FigureDescriptor::point(20.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointOnPoint(p1, p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p2, p3, 5.0));
    manager.setSolveMode(SolveMode::DRAG);

    manager.updatePoint(PointUpdateDescriptor(p1, 100.0, 0.0));

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    auto d3 = manager.getFigure(p3);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value());
    EXPECT_NEAR(d1->x.value(), 100.0, 1e-6);
    EXPECT_NEAR(d2->x.value(), 100.0, 1e-6);
    EXPECT_NEAR(d1->y.value(), 0.0, 1e-6);
    EXPECT_NEAR(d2->y.value(), 0.0, 1e-6);

    const double dx = d3->x.value() - d2->x.value();
    const double dy = d3->y.value() - d2->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 5.0, 1e-4);
}

TEST_F(DCMManagerSolveTest, DragMode_MovingOtherPointDoesNotMoveFixedPoint) {
    auto p1 = manager.addFigure(FigureDescriptor::point(5.85786437626905, 5.85786437626905));
    auto p2 = manager.addFigure(FigureDescriptor::point(20.0, 20.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 20.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.setSolveMode(SolveMode::DRAG);

    manager.updatePoint(PointUpdateDescriptor(p1, -10.0, -10.0));

    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d2.has_value());
    EXPECT_NEAR(d2->x.value(), 20.0, 1e-9);
    EXPECT_NEAR(d2->y.value(), 20.0, 1e-9);
}

TEST_F(DCMManagerSolveTest, DragMode_FallbackWithoutLocksWhenNoDofLeft) {
    auto pFixed = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto pDrag = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(pFixed, pDrag, 10.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(pFixed));
    manager.setSolveMode(SolveMode::DRAG);

    // Impossible to keep this exact drag position and the fixed distance simultaneously
    // if dragged vars are hard-locked. Solver should fallback and project to feasible state.
    manager.updatePoint(PointUpdateDescriptor(pDrag, 20.0, 0.0));

    auto dFixed = manager.getFigure(pFixed);
    auto dDrag = manager.getFigure(pDrag);
    ASSERT_TRUE(dFixed.has_value() && dDrag.has_value());
    EXPECT_NEAR(dFixed->x.value(), 0.0, 1e-9);
    EXPECT_NEAR(dFixed->y.value(), 0.0, 1e-9);

    const double dx = dDrag->x.value() - dFixed->x.value();
    const double dy = dDrag->y.value() - dFixed->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 10.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, SolveUsesUpdatedRequirementParameterAfterCacheInvalidation) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    const auto req = manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));

    manager.setSolveMode(SolveMode::DRAG);
    manager.updatePoint(PointUpdateDescriptor(p1, 2.0, 0.0));

    manager.updateRequirementParam(req, 20.0);
    const auto component = manager.getComponentForFigure(p1);
    ASSERT_TRUE(component.has_value());

    manager.setSolveMode(SolveMode::LOCAL);
    const bool converged = manager.solve(component.value());
    EXPECT_TRUE(converged);

    const auto d1 = manager.getFigure(p1);
    const auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    const double dx = d2->x.value() - d1->x.value();
    const double dy = d2->y.value() - d1->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 20.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, DragMode_RemovedRequirementDoesNotAffectSeparatedComponent) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    auto p3 = manager.addFigure(FigureDescriptor::point(20.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    const auto req = manager.addRequirement(RequirementDescriptor::pointPointDist(p2, p3, 10.0));

    manager.setSolveMode(SolveMode::DRAG);
    manager.updatePoint(PointUpdateDescriptor(p2, 12.0, 0.0));

    const auto beforeRemovalP3 = manager.getFigure(p3);
    ASSERT_TRUE(beforeRemovalP3.has_value());

    manager.removeRequirement(req);
    EXPECT_EQ(manager.getComponentCount(), 2U);

    manager.updatePoint(PointUpdateDescriptor(p2, 30.0, 0.0));

    const auto d1 = manager.getFigure(p1);
    const auto d2 = manager.getFigure(p2);
    const auto d3 = manager.getFigure(p3);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value());

    const double dx = d2->x.value() - d1->x.value();
    const double dy = d2->y.value() - d1->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 10.0, 1e-6);
    EXPECT_NEAR(d3->x.value(), beforeRemovalP3->x.value(), 1e-9);
    EXPECT_NEAR(d3->y.value(), beforeRemovalP3->y.value(), 1e-9);
}

TEST_F(DCMManagerSolveTest, GlobalSolve_Rectangle) {
    auto l1 = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 100.0, 2.0));
    auto l2 = manager.addFigure(FigureDescriptor::line(100.0, 2.0, 102.0, 52.0));
    auto l3 = manager.addFigure(FigureDescriptor::line(102.0, 52.0, -1.0, 49.0));
    auto l4 = manager.addFigure(FigureDescriptor::line(-1.0, 49.0, 0.0, 0.0));

    auto dL1 = manager.getFigure(l1);
    auto dL2 = manager.getFigure(l2);
    ASSERT_TRUE(dL1.has_value() && dL2.has_value());
    auto p1 = dL1->pointIds[0];
    auto p2 = dL1->pointIds[1];
    auto p3 = dL2->pointIds[1];

    manager.addRequirement(RequirementDescriptor::horizontal(l1));
    manager.addRequirement(RequirementDescriptor::horizontal(l3));
    manager.addRequirement(RequirementDescriptor::vertical(l2));
    manager.addRequirement(RequirementDescriptor::vertical(l4));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 100.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p2, p3, 50.0));

    manager.setSolveMode(SolveMode::GLOBAL);
    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    auto d3 = manager.getFigure(p3);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value());

    EXPECT_NEAR(d1->y.value(), d2->y.value(), 0.5);

    double dx12 = d2->x.value() - d1->x.value();
    double dy12 = d2->y.value() - d1->y.value();
    EXPECT_NEAR(std::sqrt(dx12 * dx12 + dy12 * dy12), 100.0, 1.0);

    double dx23 = d3->x.value() - d2->x.value();
    double dy23 = d3->y.value() - d2->y.value();
    EXPECT_NEAR(std::sqrt(dx23 * dx23 + dy23 * dy23), 50.0, 1.0);
}

TEST_F(DCMManagerSolveTest, GlobalSolve_ClosedPolylineViaPointOnPointAliases) {
    auto l1 = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 10.0, 0.0));
    auto l2 = manager.addFigure(FigureDescriptor::line(12.0, 0.0, 12.0, 8.0));
    auto l3 = manager.addFigure(FigureDescriptor::line(12.0, 10.0, 0.0, 10.0));
    auto l4 = manager.addFigure(FigureDescriptor::line(-2.0, 10.0, -2.0, 0.0));

    auto d1 = manager.getFigure(l1);
    auto d2 = manager.getFigure(l2);
    auto d3 = manager.getFigure(l3);
    auto d4 = manager.getFigure(l4);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value() && d4.has_value());

    manager.addRequirement(RequirementDescriptor::pointOnPoint(d1->pointIds[1], d2->pointIds[0]));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(d2->pointIds[1], d3->pointIds[0]));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(d3->pointIds[1], d4->pointIds[0]));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(d4->pointIds[1], d1->pointIds[0]));
    manager.addRequirement(RequirementDescriptor::horizontal(l1));
    manager.addRequirement(RequirementDescriptor::vertical(l2));
    manager.addRequirement(RequirementDescriptor::horizontal(l3));
    manager.addRequirement(RequirementDescriptor::vertical(l4));

    manager.setSolveMode(SolveMode::GLOBAL);
    const bool converged = manager.solve();
    EXPECT_TRUE(converged);

    d1 = manager.getFigure(l1);
    d2 = manager.getFigure(l2);
    d3 = manager.getFigure(l3);
    d4 = manager.getFigure(l4);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value() && d4.has_value());

    auto p11 = manager.getFigure(d1->pointIds[0]);
    auto p12 = manager.getFigure(d1->pointIds[1]);
    auto p21 = manager.getFigure(d2->pointIds[0]);
    auto p22 = manager.getFigure(d2->pointIds[1]);
    auto p31 = manager.getFigure(d3->pointIds[0]);
    auto p32 = manager.getFigure(d3->pointIds[1]);
    auto p41 = manager.getFigure(d4->pointIds[0]);
    auto p42 = manager.getFigure(d4->pointIds[1]);
    ASSERT_TRUE(p11.has_value() && p12.has_value() && p21.has_value() && p22.has_value()
        && p31.has_value() && p32.has_value() && p41.has_value() && p42.has_value());

    EXPECT_NEAR(p12->x.value(), p21->x.value(), 1e-6);
    EXPECT_NEAR(p12->y.value(), p21->y.value(), 1e-6);
    EXPECT_NEAR(p22->x.value(), p31->x.value(), 1e-6);
    EXPECT_NEAR(p22->y.value(), p31->y.value(), 1e-6);
    EXPECT_NEAR(p32->x.value(), p41->x.value(), 1e-6);
    EXPECT_NEAR(p32->y.value(), p41->y.value(), 1e-6);
    EXPECT_NEAR(p42->x.value(), p11->x.value(), 1e-6);
    EXPECT_NEAR(p42->y.value(), p11->y.value(), 1e-6);
}

TEST_F(DCMManagerSolveTest, SolveEmptySystem) {
    manager.setSolveMode(SolveMode::GLOBAL);
    EXPECT_TRUE(manager.solve());
}

TEST_F(DCMManagerSolveTest, LocalSolve_ThrowsWithoutComponentId) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));

    manager.setSolveMode(SolveMode::LOCAL);
    EXPECT_THROW(manager.solve(), std::runtime_error);
}
