#include "DCMManager.h"
#include "RequirementFunctionFactory.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <limits>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
double gap(const DCMManager& m, ID line, ID arc) {
    const auto l = m.getFigure(line)->coords, a = m.getFigure(arc)->coords;
    const double dx = l[2]-l[0], dy = l[3]-l[1], length2 = dx*dx+dy*dy;
    const double t = length2 == 0 ? 0 : std::clamp(((a[4]-l[0])*dx+(a[5]-l[1])*dy)/length2, 0.0, 1.0);
    const double radius = std::hypot(a[0]-a[4], a[1]-a[5]);
    EXPECT_NEAR(std::hypot(a[2]-a[4], a[3]-a[5]), radius, 1e-6);
    return std::hypot(l[0]+t*dx-a[4], l[1]+t*dy-a[5])-radius;
}
}

TEST(DCMManagerLineArcDistance, SolvesAndDragsInAllModes) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (double target : {0.0, 2.0}) {
            DCMManager m;
            const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
            const auto line = m.addFigure(FigureDescriptor::line(-8, -10, 8, -10));
            const auto before = m.getFigure(arc)->coords;
            m.addRequirement(RequirementDescriptor::fixArc(arc));
            m.addRequirement(RequirementDescriptor::lineArcDist(line, arc, target));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
            EXPECT_NEAR(gap(m, line, arc), target, 1e-6);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 3u);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updateLine({line, -8, -20, 8, -20}));
                EXPECT_NEAR(gap(m, line, arc), target, 1e-6);
            }
            EXPECT_EQ(m.getFigure(arc)->coords, before);
        }
    }
}

TEST(DCMManagerLineArcDistance, ClampsToEndpointsAcceptsCollapsedSegmentsAndIgnoresAngularSector) {
    for (auto coords : {std::array<double, 4>{-8, -7, 8, -7}, {6, -8, 12, -16},
                        {12, -16, 6, -8}, {0, -7, 0, -7}}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto line = m.addFigure(FigureDescriptor::line(coords[0], coords[1], coords[2], coords[3]));
        m.addRequirement(RequirementDescriptor::fixArc(arc));
        m.addRequirement(RequirementDescriptor::fixLine(line));
        m.addRequirement(RequirementDescriptor::lineArcDist(line, arc, gap(m, line, arc)));
        EXPECT_TRUE(m.solve());
    }
}

TEST(DCMManagerLineArcDistance, RadiusAndCenterRemainFree) {
    for (bool fixCenter : {true, false}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto line = m.addFigure(FigureDescriptor::line(-8, -10, 8, -10));
        m.addRequirement(RequirementDescriptor::fixLine(line));
        if (fixCenter) m.addRequirement(RequirementDescriptor::fixPoint(m.getFigure(arc)->pointIds[2]));
        else m.addRequirement(RequirementDescriptor::arcRadius(arc, 5));
        m.addRequirement(RequirementDescriptor::lineArcDist(line, arc, 2));
        ASSERT_TRUE(m.solve());
        EXPECT_NEAR(gap(m, line, arc), 2, 1e-6);
        const auto a = m.getFigure(arc)->coords;
        EXPECT_NEAR(std::hypot(a[0]-a[4], a[1]-a[5]), fixCenter ? 8 : 5, 1e-6);
    }
}

TEST(DCMManagerLineArcDistance, TargetsWeightsSnapshotsAndRemoval) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto line = m.addFigure(FigureDescriptor::line(-8, -8, 8, -8));
    m.addRequirement(RequirementDescriptor::fixArc(arc));
    const auto id = m.addRequirement(RequirementDescriptor::lineArcDist(line, arc, 3));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, 2);
    m.updateRequirementWeight(id, 2);
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(gap(m, line, arc), 2, 1e-6);
    const auto saved = m.snapshot();
    m.updateRequirementWeight(id, 0);
    ASSERT_TRUE(m.updateLine({line, -8, -20, 8, -20}));
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(gap(m, line, arc), 15, 1e-6);
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(gap(m, line, arc), 2, 1e-6);
    EXPECT_EQ(m.getRequirement(id)->type, RequirementType::ET_LINEARCDIST);
    EXPECT_DOUBLE_EQ(*m.getRequirement(id)->param, 2);
    EXPECT_DOUBLE_EQ(m.getRequirement(id)->weight, 2);
    m.removeRequirement(id);
    EXPECT_NE(m.getComponentForFigure(line), m.getComponentForFigure(arc));
}

TEST(DCMManagerLineArcDistance, SharedOrAliasedArcEndpointRejectsImpossibleDrag) {
    for (bool alias : {false, true}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto first = m.getFigure(arc)->pointIds[0];
        const auto point = alias ? m.addFigure(FigureDescriptor::point(3, 4)) : first;
        if (alias) m.addRequirement(RequirementDescriptor::pointOnPoint(point, first));
        const auto line = m.addFigure(FigureDescriptor::line(point, point));
        const auto id = m.addRequirement(RequirementDescriptor::lineArcDist(line, arc, 0));
        ASSERT_TRUE(m.solve());
        const auto before = m.getFigure(arc)->coords;
        m.updateRequirementParam(id, 1);
        m.setSolveMode(SolveMode::DRAG);
        EXPECT_FALSE(m.updatePoint({point, 6, 8}));
        EXPECT_EQ(m.getFigure(arc)->coords, before);
        EXPECT_EQ(m.getFigure(point)->coords, (std::vector<double>{3, 4}));
    }
}

TEST(DCMManagerLineArcDistance, FixedGeometryReportsConflictingGapAndPreservesCoordinates) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto line = m.addFigure(FigureDescriptor::line(-8, -7, 8, -7));
    const auto beforeLine = m.getFigure(line)->coords, beforeArc = m.getFigure(arc)->coords;
    m.addRequirement(RequirementDescriptor::fixArc(arc));
    m.addRequirement(RequirementDescriptor::fixLine(line));
    const auto id = m.addRequirement(RequirementDescriptor::lineArcDist(line, arc, 2));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, 3);
    m.setSolveMode(SolveMode::DRAG);
    EXPECT_FALSE(m.updateLine({line, -8, -20, 8, -20}));
    EXPECT_EQ(m.getFigure(line)->coords, beforeLine);
    EXPECT_EQ(m.getFigure(arc)->coords, beforeArc);
}

TEST(DCMManagerLineArcDistance, InvalidTargetsTypesAndZeroRadiusAreRejected) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto line = m.addFigure(FigureDescriptor::line(-8, -7, 8, -7));
    const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
    auto descriptor = RequirementDescriptor::lineArcDist(line, arc, 2);
    EXPECT_TRUE(descriptor.validate());
    for (double bad : {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        descriptor.param = bad;
        EXPECT_THROW(m.addRequirement(descriptor), std::invalid_argument);
    }
    descriptor.param.reset();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    descriptor.param = 2;
    descriptor.objectIds.pop_back();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::lineArcDist(arc, line, 0)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::lineArcDist(line, circle, 0)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::lineArcDist(line, ID(999), 0)), std::runtime_error);
    EXPECT_EQ(m.requirementCount(), 0u);
    DCMManager invalid;
    const auto zero = invalid.addFigure(FigureDescriptor::arc(0, 0, 1, 0, 0, 0));
    const auto segment = invalid.addFigure(FigureDescriptor::line(-8, -7, 8, -7));
    invalid.addRequirement(RequirementDescriptor::lineArcDist(segment, zero, 0));
    EXPECT_FALSE(invalid.solve());
}

TEST(DCMManagerLineArcDistance, FactoryAndSystemBindTheDerivedRadius) {
    Figures::GeometryStorage storage;
    const auto a = storage.createPoint(3, 4), b = storage.createPoint(-3, 4), c = storage.createPoint(0, 0);
    const auto p = storage.createPoint(-8, -8), q = storage.createPoint(8, -8);
    const auto line = *storage.createLine(p, q), arc = *storage.createArc(a, b, c);
    auto* geometry = storage.get<Figures::Arc2D>(arc);
    const auto row = Function::RequirementFunctionFactory::createLineArcDist(
        storage.get<Figures::Line2D>(line), geometry, 3);
    EXPECT_EQ(row->getVarCount(), 8u);
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), 0);
    geometry->p1->x() = 6; geometry->p1->y() = 8;
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), -5);
    System::RequirementSystem system(&storage);
    system.addLineArcDist(line, arc, 2);
    ASSERT_EQ(system.getFunctions().size(), 2u);
    EXPECT_EQ(system.getFunctions()[0]->getType(), RequirementType::ET_LINEARCDIST);
    EXPECT_DOUBLE_EQ(system.getFunctions()[0]->mathematical()->evaluate(), -4);
    geometry->p1->x() = 3; geometry->p1->y() = 4;
    EXPECT_DOUBLE_EQ(system.getFunctions()[0]->mathematical()->evaluate(), 1);
}
