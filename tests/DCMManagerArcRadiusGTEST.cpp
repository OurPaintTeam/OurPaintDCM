#include "DCMManager.h"
#include "RequirementFunctionFactory.h"
#include <gtest/gtest.h>
#include <limits>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
void expectRadius(const DCMManager& m, ID arc, double radius) {
    const auto d = m.getFigure(arc)->coords;
    EXPECT_NEAR(std::hypot(d[0] - d[4], d[1] - d[5]), radius, 1e-6);
    EXPECT_NEAR(std::hypot(d[2] - d[4], d[3] - d[5]), radius, 1e-6);
    EXPECT_GT(std::hypot(d[0] - d[2], d[1] - d[3]), 0);
}
}

TEST(DCMManagerArcRadius, BothEndpointsKeepRadiusInEverySolveModeAndDrag) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto points = m.getFigure(arc)->pointIds;
        m.addRequirement(RequirementDescriptor::fixPoint(points[2]));
        m.addRequirement(RequirementDescriptor::arcRadius(arc, 10));
        m.setSolveMode(mode);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(arc)));
        expectRadius(m, arc, 10);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 2u);
        if (mode == SolveMode::DRAG) {
            ASSERT_TRUE(m.updatePoint({points[0], 20, 20}));
            expectRadius(m, arc, 10);
        }
        EXPECT_EQ(m.getFigure(points[2])->coords, (std::vector<double>{0, 0}));
    }
}

TEST(DCMManagerArcRadius, FixedEndpointsAllowCenterToMove) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(0, 0, 6, 0, 3, 4));
    const auto points = m.getFigure(arc)->pointIds;
    m.addRequirement(RequirementDescriptor::fixPoint(points[0]));
    m.addRequirement(RequirementDescriptor::fixPoint(points[1]));
    m.addRequirement(RequirementDescriptor::arcRadius(arc, 7));
    ASSERT_TRUE(m.solve());
    expectRadius(m, arc, 7);
    EXPECT_NEAR(*m.getFigure(points[2])->x, 3, 1e-6);
    EXPECT_EQ(m.getFigure(points[0])->coords, (std::vector<double>{0, 0}));
    EXPECT_EQ(m.getFigure(points[1])->coords, (std::vector<double>{6, 0}));
    EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
}

TEST(DCMManagerArcRadius, FreeArcKeepsTranslationAndAngularFreedom) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(1, 0, 0, 1, 0, 0));
    m.addRequirement(RequirementDescriptor::arcRadius(arc, 5));
    ASSERT_TRUE(m.solve());
    expectRadius(m, arc, 5);
    EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 4u);
    auto d = m.getFigure(arc)->coords;
    for (std::size_t i = 0; i < d.size(); i += 2) { d[i] += 2; d[i + 1] += 3; }
    m.setSolveMode(SolveMode::DRAG);
    ASSERT_TRUE(m.updateFigure(FigureUpdateDescriptor::arc(arc, d[0], d[1], d[2], d[3], d[4], d[5])));
    EXPECT_EQ(m.getFigure(arc)->coords, d);
    expectRadius(m, arc, 5);
}

TEST(DCMManagerArcRadius, ParameterWeightSnapshotAndRemoval) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    m.addRequirement(RequirementDescriptor::fixPoint(m.getFigure(arc)->pointIds[2]));
    const auto id = m.addRequirement(RequirementDescriptor::arcRadius(arc, 5));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, 8);
    m.updateRequirementWeight(id, 2);
    ASSERT_TRUE(m.solve());
    expectRadius(m, arc, 8);
    const auto saved = m.snapshot();
    m.updateRequirementWeight(id, 0);
    ASSERT_TRUE(m.updateArc({arc, 3, 0, 0, 3, 0, 0}));
    ASSERT_TRUE(m.solve());
    expectRadius(m, arc, 3);
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    expectRadius(m, arc, 8);
    EXPECT_DOUBLE_EQ(*m.getRequirement(id)->param, 8);
    EXPECT_DOUBLE_EQ(m.getRequirement(id)->weight, 2);
    m.removeRequirement(id);
    ASSERT_TRUE(m.updateArc({arc, 3, 0, 0, 3, 0, 0}));
    ASSERT_TRUE(m.solve());
    expectRadius(m, arc, 3);
    EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 3u);
}

TEST(DCMManagerArcRadius, ConflictingFixedArcReportsFailureWithoutMovingGeometry) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto before = m.getFigure(arc)->coords;
    m.addRequirement(RequirementDescriptor::fixArc(arc));
    const auto id = m.addRequirement(RequirementDescriptor::arcRadius(arc, 5));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, 10);
    EXPECT_FALSE(m.solve());
    EXPECT_EQ(m.getFigure(arc)->coords, before);
    m.setSolveMode(SolveMode::DRAG);
    EXPECT_FALSE(m.updateArc({arc, 6, 8, -6, 8, 0, 0}));
    EXPECT_EQ(m.getFigure(arc)->coords, before);
}

TEST(DCMManagerArcRadius, CoincidentEndpointAndCenterAliasesUseRepresentatives) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto points = m.getFigure(arc)->pointIds;
    const auto endpoint = m.addFigure(FigureDescriptor::point(3, 4));
    const auto center = m.addFigure(FigureDescriptor::point(0, 0));
    m.addRequirement(RequirementDescriptor::pointOnPoint(points[0], endpoint));
    m.addRequirement(RequirementDescriptor::pointOnPoint(points[2], center));
    m.addRequirement(RequirementDescriptor::fixPoint(center));
    m.addRequirement(RequirementDescriptor::arcRadius(arc, 10));
    ASSERT_TRUE(m.solve());
    expectRadius(m, arc, 10);
    EXPECT_EQ(m.getFigure(points[0])->coords, m.getFigure(endpoint)->coords);
    EXPECT_EQ(m.getFigure(points[2])->coords, m.getFigure(center)->coords);
    EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 2u);
}

TEST(DCMManagerArcRadius, RejectsInvalidRadiusTypesAndCollapsedChord) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
    const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
    for (double bad : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcRadius(arc, bad)), std::invalid_argument);
    }
    auto descriptor = RequirementDescriptor::arcRadius(arc, 5);
    descriptor.param.reset();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    descriptor.param = 5;
    descriptor.objectIds.clear();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcRadius(circle, 5)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcRadius(ID(999), 5)), std::runtime_error);
    EXPECT_EQ(m.requirementCount(), 0u);
    const auto id = m.addRequirement(RequirementDescriptor::arcRadius(arc, 5));
    EXPECT_THROW(m.updateRequirementParam(id, 0), std::invalid_argument);
    EXPECT_DOUBLE_EQ(*m.getRequirement(id)->param, 5);
    DCMManager collapsed;
    const auto invalid = collapsed.addFigure(FigureDescriptor::arc(3, 4, 3, 4, 0, 0));
    collapsed.addRequirement(RequirementDescriptor::arcRadius(invalid, 5));
    EXPECT_FALSE(collapsed.solve());
}

TEST(DCMManagerArcRadius, FactoryAndSystemReuseLivePointDistance) {
    Figures::GeometryStorage storage;
    const auto a = storage.createPoint(3, 4), b = storage.createPoint(-3, 4), c = storage.createPoint(0, 0);
    const auto arc = *storage.createArc(a, b, c);
    auto* geometry = storage.get<Figures::Arc2D>(arc);
    const auto row = Function::RequirementFunctionFactory::createArcRadius(geometry, 5);
    EXPECT_EQ(row->getVars(), (std::vector<VAR>{geometry->p1->ptrX(), geometry->p1->ptrY(),
        geometry->p_center->ptrX(), geometry->p_center->ptrY()}));
    EXPECT_NE(dynamic_cast<PointPointDistanceError*>(row->mathematical().get()), nullptr);
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), 0);
    geometry->p1->x() = 6;
    geometry->p1->y() = 8;
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), 5);
    EXPECT_THROW(Function::RequirementFunctionFactory::createArcRadius(geometry, 0), std::invalid_argument);
    System::RequirementSystem system(&storage);
    system.addArcRadius(arc, 10);
    ASSERT_EQ(system.getFunctions().size(), 2u);
    EXPECT_EQ(system.getFunctions()[0]->getType(), RequirementType::ET_ARCRADIUS);
    EXPECT_DOUBLE_EQ(system.getFunctions()[0]->mathematical()->evaluate(), 0);
    EXPECT_NE(system.getFunctions()[1]->mathematical()->evaluate(), 0);
}
