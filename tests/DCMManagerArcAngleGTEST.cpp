#include "DCMManager.h"
#include "RequirementFunctionFactory.h"
#include <gtest/gtest.h>
#include <limits>
#include <numbers>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
constexpr double pi = std::numbers::pi;
double sweep(const DCMManager& m, ID arc) {
    const auto p = m.getFigure(arc)->coords;
    const double ax = p[0]-p[4], ay = p[1]-p[5], bx = p[2]-p[4], by = p[3]-p[5];
    const double angle = std::atan2(ax*by - ay*bx, ax*bx + ay*by);
    return angle < 0 ? angle + 2*pi : angle;
}
void expectArc(const DCMManager& m, ID arc, double angle, double radius) {
    const auto p = m.getFigure(arc)->coords;
    EXPECT_NEAR(sweep(m, arc), angle, 1e-6);
    EXPECT_NEAR(std::hypot(p[0]-p[4], p[1]-p[5]), radius, 1e-6);
    EXPECT_NEAR(std::hypot(p[2]-p[4], p[3]-p[5]), radius, 1e-6);
}
}

TEST(DCMManagerArcAngle, MinorMajorAndSemicircleWorkInAllModesAndDrag) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (double target : {0.1, pi/2, pi, 3*pi/2, 2*pi - 0.1}) {
            DCMManager m;
            const auto arc = m.addFigure(FigureDescriptor::arc(5, 0, 0, 5, 0, 0));
            const auto points = m.getFigure(arc)->pointIds;
            m.addRequirement(RequirementDescriptor::fixPoint(points[2]));
            m.addRequirement(RequirementDescriptor::arcRadius(arc, 5));
            m.addRequirement(RequirementDescriptor::arcAngle(arc, target));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(arc)));
            expectArc(m, arc, target, 5);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 1u);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updatePoint({points[1], -9, 4}));
                expectArc(m, arc, target, 5);
            }
            EXPECT_EQ(m.getFigure(points[2])->coords, (std::vector<double>{0, 0}));
        }
    }
}

TEST(DCMManagerArcAngle, FixedStartAndCenterDetermineTheSecondEndpoint) {
    for (double target : {pi/2, pi, 3*pi/2}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(5, 0, 4, 3, 0, 0));
        const auto points = m.getFigure(arc)->pointIds;
        m.addRequirement(RequirementDescriptor::fixPoint(points[0]));
        m.addRequirement(RequirementDescriptor::fixPoint(points[2]));
        m.addRequirement(RequirementDescriptor::arcAngle(arc, target));
        ASSERT_TRUE(m.solve());
        expectArc(m, arc, target, 5);
        EXPECT_NEAR(*m.getFigure(points[1])->x, 5 * std::cos(target), 1e-6);
        EXPECT_NEAR(*m.getFigure(points[1])->y, 5 * std::sin(target), 1e-6);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
    }
}

TEST(DCMManagerArcAngle, FixedEndpointsMoveCenterThroughTheSemicircle) {
    for (double target : {pi/2, pi, 3*pi/2}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(-5, 0, 5, 0, 0, 3));
        const auto points = m.getFigure(arc)->pointIds;
        m.addRequirement(RequirementDescriptor::fixPoint(points[0]));
        m.addRequirement(RequirementDescriptor::fixPoint(points[1]));
        m.addRequirement(RequirementDescriptor::arcAngle(arc, target));
        ASSERT_TRUE(m.solve());
        expectArc(m, arc, target, 5 / std::sin(target / 2));
        EXPECT_NEAR(*m.getFigure(points[2])->x, 0, 1e-6);
        EXPECT_NEAR(*m.getFigure(points[2])->y, 5 / std::tan(target / 2), 1e-6);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
    }
}

TEST(DCMManagerArcAngle, FreeArcKeepsFourDegreesOfFreedomAndCanBeScaled) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    m.addRequirement(RequirementDescriptor::arcAngle(arc, 3*pi/2));
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(sweep(m, arc), 3*pi/2, 1e-6);
    EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 4u);
    auto p = m.getFigure(arc)->coords;
    const double radius = std::hypot(p[0]-p[4], p[1]-p[5]);
    for (std::size_t i = 0; i < p.size(); i += 2) { p[i] = 2*p[i]+2; p[i+1] = 2*p[i+1]+3; }
    m.setSolveMode(SolveMode::DRAG);
    ASSERT_TRUE(m.updateFigure(FigureUpdateDescriptor::arc(arc, p[0], p[1], p[2], p[3], p[4], p[5])));
    expectArc(m, arc, 3*pi/2, 2*radius);
}

TEST(DCMManagerArcAngle, ParameterWeightSnapshotAndRemoval) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(5, 0, 0, 5, 0, 0));
    const auto points = m.getFigure(arc)->pointIds;
    m.addRequirement(RequirementDescriptor::fixPoint(points[0]));
    m.addRequirement(RequirementDescriptor::fixPoint(points[2]));
    const auto id = m.addRequirement(RequirementDescriptor::arcAngle(arc, pi/2));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, 3*pi/2);
    m.updateRequirementWeight(id, 2);
    ASSERT_TRUE(m.solve());
    expectArc(m, arc, 3*pi/2, 5);
    const auto saved = m.snapshot();
    m.updateRequirementWeight(id, 0);
    ASSERT_TRUE(m.updatePoint({points[1], 0, 5}));
    ASSERT_TRUE(m.solve());
    expectArc(m, arc, pi/2, 5);
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    expectArc(m, arc, 3*pi/2, 5);
    EXPECT_DOUBLE_EQ(*m.getRequirement(id)->param, 3*pi/2);
    EXPECT_DOUBLE_EQ(m.getRequirement(id)->weight, 2);
    m.removeRequirement(id);
    ASSERT_TRUE(m.updatePoint({points[1], 0, 5}));
    ASSERT_TRUE(m.solve());
    expectArc(m, arc, pi/2, 5);
}

TEST(DCMManagerArcAngle, PointAliasesUseTheSameAngularVariables) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(5, 0, 0, 5, 0, 0));
    const auto points = m.getFigure(arc)->pointIds;
    const auto endpoint = m.addFigure(FigureDescriptor::point(5, 0));
    const auto center = m.addFigure(FigureDescriptor::point(0, 0));
    m.addRequirement(RequirementDescriptor::pointOnPoint(points[0], endpoint));
    m.addRequirement(RequirementDescriptor::pointOnPoint(points[2], center));
    m.addRequirement(RequirementDescriptor::fixPoint(endpoint));
    m.addRequirement(RequirementDescriptor::fixPoint(center));
    m.addRequirement(RequirementDescriptor::arcAngle(arc, 3*pi/2));
    ASSERT_TRUE(m.solve());
    expectArc(m, arc, 3*pi/2, 5);
    EXPECT_EQ(m.getFigure(points[0])->coords, m.getFigure(endpoint)->coords);
    EXPECT_EQ(m.getFigure(points[2])->coords, m.getFigure(center)->coords);
}

TEST(DCMManagerArcAngle, ConflictingFixedArcRejectsDragWithoutMovingGeometry) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(5, 0, 0, 5, 0, 0));
    const auto before = m.getFigure(arc)->coords;
    m.addRequirement(RequirementDescriptor::fixArc(arc));
    const auto id = m.addRequirement(RequirementDescriptor::arcAngle(arc, pi/2));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, 3*pi/2);
    EXPECT_FALSE(m.solve());
    EXPECT_EQ(m.getFigure(arc)->coords, before);
    m.setSolveMode(SolveMode::DRAG);
    EXPECT_FALSE(m.updateArc({arc, 5, 0, 0, -5, 0, 0}));
    EXPECT_EQ(m.getFigure(arc)->coords, before);
}

TEST(DCMManagerArcAngle, InvalidAnglesTypesAndDegenerateArcsAreRejected) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(5, 0, 0, 5, 0, 0));
    const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
    for (double bad : {0.0, -1.0, 2*pi, 2*pi+1, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcAngle(arc, bad)), std::invalid_argument);
    auto descriptor = RequirementDescriptor::arcAngle(arc, pi);
    descriptor.param.reset();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    descriptor.param = pi;
    descriptor.objectIds.clear();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcAngle(circle, pi)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcAngle(ID(999), pi)), std::runtime_error);
    EXPECT_EQ(m.requirementCount(), 0u);
    const auto id = m.addRequirement(RequirementDescriptor::arcAngle(arc, pi/2));
    EXPECT_THROW(m.updateRequirementParam(id, 2*pi), std::invalid_argument);
    EXPECT_DOUBLE_EQ(*m.getRequirement(id)->param, pi/2);
    DCMManager collapsed;
    const auto invalid = collapsed.addFigure(FigureDescriptor::arc(5, 0, 5, 0, 0, 0));
    collapsed.addRequirement(RequirementDescriptor::arcAngle(invalid, pi));
    EXPECT_FALSE(collapsed.solve());
}

TEST(DCMManagerArcAngle, FactoryAndSystemBindLiveCounterclockwiseAngle) {
    Figures::GeometryStorage storage;
    const auto a = storage.createPoint(5, 0), b = storage.createPoint(0, 5), c = storage.createPoint(0, 0);
    const auto arc = *storage.createArc(a, b, c);
    auto* geometry = storage.get<Figures::Arc2D>(arc);
    const auto row = Function::RequirementFunctionFactory::createArcAngle(geometry, pi/2);
    EXPECT_EQ(row->getVarCount(), 6u);
    EXPECT_NE(dynamic_cast<ArcAngleError*>(row->mathematical().get()), nullptr);
    EXPECT_TRUE(row->mathematical()->satisfied(1e-12));
    geometry->p2->y() = -5;
    EXPECT_NEAR(std::abs(row->mathematical()->evaluate()), pi, 1e-12);
    System::RequirementSystem system(&storage);
    system.addArcAngle(arc, 3*pi/2);
    ASSERT_EQ(system.getFunctions().size(), 2u);
    EXPECT_EQ(system.getFunctions()[0]->getType(), RequirementType::ET_ARCANGLE);
    EXPECT_TRUE(system.getFunctions()[0]->mathematical()->satisfied(1e-12));
    geometry->p2->x() = -5; geometry->p2->y() = 0;
    EXPECT_NEAR(system.getFunctions()[0]->mathematical()->evaluate(), -pi/2, 1e-12);
}
