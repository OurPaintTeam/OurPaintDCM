#include "DCMManager.h"
#include "RequirementFunctionFactory.h"
#include <gtest/gtest.h>
#include <limits>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
double radius(const DCMManager& m, ID arc) {
    const auto a = m.getFigure(arc)->coords;
    const double r = std::hypot(a[0]-a[4], a[1]-a[5]);
    EXPECT_NEAR(std::hypot(a[2]-a[4], a[3]-a[5]), r, 1e-6);
    return r;
}
double gap(const DCMManager& m, ID point, ID arc) {
    const auto p = *m.getFigure(point);
    const auto a = m.getFigure(arc)->coords;
    return std::hypot(*p.x-a[4], *p.y-a[5]) - radius(m, arc);
}
}

TEST(DCMManagerPointArcDistance, SignedGapsAndDragMatchCircleInAllModes) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (double target : {-3.0, 0.0, 2.0}) {
            DCMManager m;
            const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
            const auto point = m.addFigure(FigureDescriptor::point(0, -8));
            const auto before = m.getFigure(arc)->coords;
            m.addRequirement(RequirementDescriptor::fixArc(arc));
            m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, target));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(point)));
            EXPECT_NEAR(gap(m, point, arc), target, 1e-6);
            EXPECT_LT(*m.getFigure(point)->y, 0);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 1u);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updatePoint({point, 20, -20}));
                EXPECT_NEAR(gap(m, point, arc), target, 1e-6);
            }
            EXPECT_EQ(m.getFigure(arc)->coords, before);
        }
    }
}

TEST(DCMManagerPointArcDistance, FixedPointsOutsideAngularSectorAreAccepted) {
    for (double target : {-3.0, 0.0, 2.0}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto point = m.addFigure(FigureDescriptor::point(0, -(5+target)));
        m.addRequirement(RequirementDescriptor::fixArc(arc));
        m.addRequirement(RequirementDescriptor::fixPoint(point));
        m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, target));
        EXPECT_TRUE(m.solve());
    }
}

TEST(DCMManagerPointArcDistance, PointCanStartAtCenter) {
    for (double target : {-5.0, -3.0, 2.0}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto point = m.addFigure(FigureDescriptor::point(0, 0));
        m.addRequirement(RequirementDescriptor::fixArc(arc));
        m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, target));
        ASSERT_TRUE(m.solve());
        EXPECT_NEAR(gap(m, point, arc), target, 1e-6);
    }
}

TEST(DCMManagerPointArcDistance, DerivedRadiusAndCenterRemainFree) {
    for (double target : {-2.0, 4.0}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto point = m.addFigure(FigureDescriptor::point(10, 0));
        m.addRequirement(RequirementDescriptor::fixPoint(m.getFigure(arc)->pointIds[2]));
        m.addRequirement(RequirementDescriptor::fixPoint(point));
        m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, target));
        ASSERT_TRUE(m.solve());
        EXPECT_NEAR(radius(m, arc), 10-target, 1e-6);
    }
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto point = m.addFigure(FigureDescriptor::point(10, 0));
    m.addRequirement(RequirementDescriptor::fixPoint(point));
    m.addRequirement(RequirementDescriptor::arcRadius(arc, 5));
    m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, -2));
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(gap(m, point, arc), -2, 1e-6);
    EXPECT_NEAR(radius(m, arc), 5, 1e-6);
}

TEST(DCMManagerPointArcDistance, TargetsWeightsSnapshotsAndRemoval) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto point = m.addFigure(FigureDescriptor::point(0, -8));
    m.addRequirement(RequirementDescriptor::fixArc(arc));
    const auto id = m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, 3));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, -2);
    m.updateRequirementWeight(id, 2);
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(gap(m, point, arc), -2, 1e-6);
    const auto saved = m.snapshot();
    m.updateRequirementWeight(id, 0);
    ASSERT_TRUE(m.updatePoint({point, 11, 12}));
    ASSERT_TRUE(m.solve());
    EXPECT_EQ(m.getFigure(point)->coords, (std::vector<double>{11, 12}));
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(gap(m, point, arc), -2, 1e-6);
    EXPECT_EQ(m.getRequirement(id)->type, RequirementType::ET_POINTARCDIST);
    EXPECT_DOUBLE_EQ(*m.getRequirement(id)->param, -2);
    EXPECT_DOUBLE_EQ(m.getRequirement(id)->weight, 2);
    m.removeRequirement(id);
    EXPECT_NE(m.getComponentForFigure(point), m.getComponentForFigure(arc));
}

TEST(DCMManagerPointArcDistance, CenterAliasesKeepTheRadiusLive) {
    for (bool alias : {false, true}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto center = m.getFigure(arc)->pointIds[2];
        const auto point = alias ? m.addFigure(FigureDescriptor::point(0, 0)) : center;
        if (alias) m.addRequirement(RequirementDescriptor::pointOnPoint(point, center));
        const auto id = m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, -5));
        ASSERT_TRUE(m.solve());
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 4u);
        m.updateRequirementParam(id, -7);
        ASSERT_TRUE(m.solve());
        EXPECT_NEAR(radius(m, arc), 7, 1e-6);
        EXPECT_EQ(m.getFigure(point)->coords, m.getFigure(center)->coords);
    }
}

TEST(DCMManagerPointArcDistance, PointAtTheFirstEndpointCanOnlyHaveZeroGap) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto point = m.getFigure(arc)->pointIds[0];
    const auto id = m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, 0));
    ASSERT_TRUE(m.solve());
    const auto before = m.getFigure(arc)->coords;
    m.updateRequirementParam(id, 1);
    m.setSolveMode(SolveMode::DRAG);
    EXPECT_FALSE(m.updatePoint({point, 6, 8}));
    EXPECT_EQ(m.getFigure(arc)->coords, before);
}

TEST(DCMManagerPointArcDistance, ImpossibleInteriorGapRejectsDragAndPreservesGeometry) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto point = m.addFigure(FigureDescriptor::point(2, 0));
    const auto before = m.getFigure(arc)->coords;
    m.addRequirement(RequirementDescriptor::fixArc(arc));
    const auto id = m.addRequirement(RequirementDescriptor::pointArcDist(point, arc, -3));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, -6);
    m.setSolveMode(SolveMode::DRAG);
    EXPECT_FALSE(m.updatePoint({point, 10, 20}));
    EXPECT_EQ(m.getFigure(point)->coords, (std::vector<double>{2, 0}));
    EXPECT_EQ(m.getFigure(arc)->coords, before);
}

TEST(DCMManagerPointArcDistance, InvalidTargetsTypesAndZeroRadiusAreRejected) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto point = m.addFigure(FigureDescriptor::point(2, 0));
    const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
    auto descriptor = RequirementDescriptor::pointArcDist(point, arc, -3);
    EXPECT_TRUE(descriptor.validate());
    for (double bad : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        descriptor.param = bad;
        EXPECT_THROW(m.addRequirement(descriptor), std::invalid_argument);
    }
    descriptor.param.reset();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    descriptor.param = -3;
    descriptor.objectIds.pop_back();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointArcDist(arc, point, 0)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointArcDist(point, circle, 0)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointArcDist(point, ID(999), 0)), std::runtime_error);
    EXPECT_EQ(m.requirementCount(), 0u);
    DCMManager invalid;
    const auto zero = invalid.addFigure(FigureDescriptor::arc(0, 0, 1, 0, 0, 0));
    const auto p = invalid.addFigure(FigureDescriptor::point(2, 0));
    invalid.addRequirement(RequirementDescriptor::pointArcDist(p, zero, 0));
    EXPECT_FALSE(invalid.solve());
}

TEST(DCMManagerPointArcDistance, FactoryAndSystemBindTheDerivedRadius) {
    Figures::GeometryStorage storage;
    const auto a = storage.createPoint(3, 4), b = storage.createPoint(-3, 4), c = storage.createPoint(0, 0);
    const auto point = storage.createPoint(0, -8), arc = *storage.createArc(a, b, c);
    auto* geometry = storage.get<Figures::Arc2D>(arc);
    const auto row = Function::RequirementFunctionFactory::createPointArcDist(
        storage.get<Figures::Point2D>(point), geometry, 3);
    EXPECT_EQ(row->getVarCount(), 6u);
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), 0);
    geometry->p1->x() = 6; geometry->p1->y() = 8;
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), -5);
    System::RequirementSystem system(&storage);
    system.addPointArcDist(point, arc, -2);
    ASSERT_EQ(system.getFunctions().size(), 2u);
    EXPECT_EQ(system.getFunctions()[0]->getType(), RequirementType::ET_POINTARCDIST);
    EXPECT_DOUBLE_EQ(system.getFunctions()[0]->mathematical()->evaluate(), 0);
    geometry->p1->x() = 3; geometry->p1->y() = 4;
    EXPECT_DOUBLE_EQ(system.getFunctions()[0]->mathematical()->evaluate(), 5);
}
