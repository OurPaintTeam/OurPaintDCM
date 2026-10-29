#include "DCMManager.h"
#include "RequirementFunctionFactory.h"
#include <gtest/gtest.h>
#include <limits>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerPointCircleDistance, SignedGapsAndDragInAllModes) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (double target : {-3.0, 0.0, 2.0}) {
            DCMManager m;
            const auto circle = m.addFigure(FigureDescriptor::circle(2, 3, 5));
            const auto point = m.addFigure(FigureDescriptor::point(3, 5));
            m.addRequirement(RequirementDescriptor::fixCircle(circle));
            m.addRequirement(RequirementDescriptor::pointCircleDist(point, circle, target));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(point)));
            auto p = *m.getFigure(point);
            EXPECT_NEAR(std::hypot(*p.x - 2, *p.y - 3) - 5, target, 1e-6);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 1u);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updatePoint({point, 20, 20}));
                p = *m.getFigure(point);
                EXPECT_NEAR(std::hypot(*p.x - 2, *p.y - 3) - 5, target, 1e-6);
            }
            EXPECT_EQ(m.getFigure(circle)->coords, (std::vector<double>{2, 3}));
            EXPECT_DOUBLE_EQ(*m.getFigure(circle)->radius, 5);
        }
    }
}

TEST(DCMManagerPointCircleDistance, PointCanStartAtCenter) {
    for (double target : {-5.0, -3.0, 2.0}) {
        DCMManager m;
        const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
        const auto point = m.addFigure(FigureDescriptor::point(0, 0));
        m.addRequirement(RequirementDescriptor::fixCircle(circle));
        m.addRequirement(RequirementDescriptor::pointCircleDist(point, circle, target));
        ASSERT_TRUE(m.solve());
        const auto p = *m.getFigure(point);
        EXPECT_NEAR(std::hypot(*p.x, *p.y), 5 + target, 1e-6);
    }
}

TEST(DCMManagerPointCircleDistance, RadiusAndCenterRemainLiveVariables) {
    for (double target : {-2.0, 4.0}) {
        DCMManager m;
        const auto center = m.addFigure(FigureDescriptor::point(0, 0));
        const auto circle = m.addFigure(FigureDescriptor::circle(center, 5));
        const auto point = m.addFigure(FigureDescriptor::point(10, 0));
        m.addRequirement(RequirementDescriptor::fixPoint(center));
        m.addRequirement(RequirementDescriptor::fixPoint(point));
        m.addRequirement(RequirementDescriptor::pointCircleDist(point, circle, target));
        ASSERT_TRUE(m.solve());
        EXPECT_NEAR(*m.getFigure(circle)->radius, 10 - target, 1e-6);
    }
    DCMManager m;
    const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
    const auto point = m.addFigure(FigureDescriptor::point(10, 0));
    m.addRequirement(RequirementDescriptor::fixPoint(point));
    m.addRequirement(RequirementDescriptor::circleRadius(circle, 5));
    m.addRequirement(RequirementDescriptor::pointCircleDist(point, circle, -2));
    ASSERT_TRUE(m.solve());
    const auto c = m.getFigure(circle)->coords;
    EXPECT_NEAR(std::hypot(10 - c[0], c[1]), 3, 1e-6);
    EXPECT_NEAR(*m.getFigure(circle)->radius, 5, 1e-6);
}

TEST(DCMManagerPointCircleDistance, TargetsWeightsSnapshotsAndRemoval) {
    DCMManager m;
    const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
    const auto point = m.addFigure(FigureDescriptor::point(8, 0));
    m.addRequirement(RequirementDescriptor::fixCircle(circle));
    const auto id = m.addRequirement(RequirementDescriptor::pointCircleDist(point, circle, 3));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, -2);
    m.updateRequirementWeight(id, 2);
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(*m.getFigure(point)->x, 3, 1e-6);
    const auto saved = m.snapshot();
    m.updateRequirementWeight(id, 0);
    ASSERT_TRUE(m.updatePoint({point, 11, 12}));
    ASSERT_TRUE(m.solve());
    EXPECT_EQ(m.getFigure(point)->coords, (std::vector<double>{11, 12}));
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    EXPECT_DOUBLE_EQ(*m.getRequirement(id)->param, -2);
    EXPECT_DOUBLE_EQ(m.getRequirement(id)->weight, 2);
    EXPECT_NEAR(*m.getFigure(point)->x, 3, 1e-6);
    m.removeRequirement(id);
    EXPECT_NE(m.getComponentForFigure(point), m.getComponentForFigure(circle));
}

TEST(DCMManagerPointCircleDistance, CenterAliasesCancelTheirCoordinateDerivatives) {
    for (bool alias : {false, true}) {
        DCMManager m;
        const auto center = m.addFigure(FigureDescriptor::point(2, 3));
        const auto point = alias ? m.addFigure(FigureDescriptor::point(9, 10)) : center;
        const auto circle = m.addFigure(FigureDescriptor::circle(center, 5));
        if (alias) m.addRequirement(RequirementDescriptor::pointOnPoint(point, center));
        const auto id = m.addRequirement(RequirementDescriptor::pointCircleDist(point, circle, -5));
        ASSERT_TRUE(m.solve());
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 2u);
        m.updateRequirementParam(id, -7);
        ASSERT_TRUE(m.solve());
        EXPECT_NEAR(*m.getFigure(circle)->radius, 7, 1e-6);
    }
}

TEST(DCMManagerPointCircleDistance, ImpossibleInteriorGapRejectsDragAndPreservesGeometry) {
    DCMManager m;
    const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
    const auto point = m.addFigure(FigureDescriptor::point(2, 0));
    m.addRequirement(RequirementDescriptor::fixCircle(circle));
    const auto id = m.addRequirement(RequirementDescriptor::pointCircleDist(point, circle, -3));
    ASSERT_TRUE(m.solve());
    m.updateRequirementParam(id, -6);
    m.setSolveMode(SolveMode::DRAG);
    EXPECT_FALSE(m.updatePoint({point, 10, 20}));
    EXPECT_EQ(m.getFigure(point)->coords, (std::vector<double>{2, 0}));
    EXPECT_DOUBLE_EQ(*m.getFigure(circle)->radius, 5);
    m.updateRequirementParam(id, -3);
    EXPECT_TRUE(m.solve());
}

TEST(DCMManagerPointCircleDistance, InvalidTargetsTypesAndObjectCountsAreRejected) {
    DCMManager m;
    const auto point = m.addFigure(FigureDescriptor::point(2, 0));
    const auto circle = m.addFigure(FigureDescriptor::circle(0, 0, 5));
    auto descriptor = RequirementDescriptor::pointCircleDist(point, circle, -3);
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
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointCircleDist(circle, point, 0)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointCircleDist(point, point, 0)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointCircleDist(point, ID(999), 0)), std::runtime_error);
    EXPECT_EQ(m.requirementCount(), 0u);
}

TEST(DCMManagerPointCircleDistance, FactoryAndSystemBindLiveRadiusAndSignedTarget) {
    Figures::GeometryStorage storage;
    const auto center = storage.createPoint(1, 2), point = storage.createPoint(4, 6);
    const auto circle = *storage.createCircle(center, 5);
    auto* c = storage.get<Figures::Circle2D>(circle);
    auto* p = storage.get<Figures::Point2D>(point);
    const auto row = Function::RequirementFunctionFactory::createPointCircleDist(p, c, -2);
    EXPECT_EQ(row->getVarCount(), 5u);
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), 2);
    c->radius = 7;
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), 0);
    System::RequirementSystem system(&storage);
    system.addPointCircleDist(point, circle, -2);
    ASSERT_EQ(system.getFunctions().size(), 1u);
    EXPECT_EQ(system.getFunctions()[0]->getType(), RequirementType::ET_POINTCIRCLEDIST);
    c->radius = 8;
    EXPECT_DOUBLE_EQ(system.getFunctions()[0]->mathematical()->evaluate(), -1);
}
