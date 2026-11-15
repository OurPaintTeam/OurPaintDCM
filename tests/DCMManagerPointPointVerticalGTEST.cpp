#include "DCMManager.h"
#include "RequirementFunctionFactory.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerPointPointVertical, SolvesBothOrdersAndKeepsVerticalFreedom) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (bool reversed : {false, true}) {
            DCMManager m;
            const auto a = m.addFigure(FigureDescriptor::point(3, 2));
            const auto p = m.addFigure(FigureDescriptor::point(17, -5));
            m.addRequirement(RequirementDescriptor::fixPoint(a));
            const auto id = m.addRequirement(reversed
                ? RequirementDescriptor::pointPointVertical(p, a)
                : RequirementDescriptor::pointPointVertical(a, p));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(p)));
            EXPECT_DOUBLE_EQ(*m.getFigure(p)->y, -5);
            EXPECT_NEAR(*m.getFigure(p)->x, 3, 1e-6);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 1u);
            EXPECT_EQ(m.figureCount(), 2u);
            EXPECT_FALSE(m.getRequirement(id)->param);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updatePoint({p, -9, 8}));
                EXPECT_NEAR(*m.getFigure(p)->x, 3, 1e-6);
                EXPECT_DOUBLE_EQ(*m.getFigure(p)->y, 8);
            }
        }
    }
}

TEST(DCMManagerPointPointVertical, WeightsSnapshotsAndRemoval) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(3, 2));
    const auto p = m.addFigure(FigureDescriptor::point(17, 5));
    m.addRequirement(RequirementDescriptor::fixPoint(a));
    const auto id = m.addRequirement(RequirementDescriptor::pointPointVertical(a, p));
    m.updateRequirementWeight(id, 2);
    ASSERT_TRUE(m.solve());
    const auto saved = m.snapshot();
    m.updateRequirementWeight(id, 0);
    ASSERT_TRUE(m.updatePoint({p, 9, 8}));
    ASSERT_TRUE(m.solve());
    EXPECT_DOUBLE_EQ(*m.getFigure(p)->x, 9);
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(*m.getFigure(p)->x, 3, 1e-6);
    EXPECT_DOUBLE_EQ(m.getRequirement(id)->weight, 2);
    m.removeRequirement(id);
    EXPECT_NE(m.getComponentForFigure(a), m.getComponentForFigure(p));
}

TEST(DCMManagerPointPointVertical, CoincidentPointsRepeatedIdAndAliasesAreValid) {
    for (int variant = 0; variant < 3; ++variant) {
        DCMManager m;
        const auto a = m.addFigure(FigureDescriptor::point(3, 2));
        const auto p = variant == 0 ? a : m.addFigure(FigureDescriptor::point(3, 2));
        if (variant == 2) m.addRequirement(RequirementDescriptor::pointOnPoint(a, p));
        m.addRequirement(RequirementDescriptor::pointPointVertical(a, p));
        ASSERT_TRUE(m.solve());
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, variant == 1 ? 3u : 2u);
    }
}

TEST(DCMManagerPointPointVertical, IncompatibleFixedPointsAndInvalidDescriptorsFail) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(3, 2));
    const auto p = m.addFigure(FigureDescriptor::point(17, 5));
    const auto line = m.addFigure(FigureDescriptor::line(a, p));
    m.addRequirement(RequirementDescriptor::fixPoint(a));
    m.addRequirement(RequirementDescriptor::fixPoint(p));
    auto descriptor = RequirementDescriptor::pointPointVertical(a, p);
    const auto id = m.addRequirement(descriptor);
    EXPECT_FALSE(m.solve());
    EXPECT_DOUBLE_EQ(*m.getFigure(a)->x, 3);
    EXPECT_DOUBLE_EQ(*m.getFigure(p)->x, 17);
    EXPECT_THROW(m.updateRequirementParam(id, 0), std::runtime_error);
    descriptor.param = 0;
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    descriptor.param.reset();
    descriptor.objectIds.pop_back();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    const auto count = m.requirementCount();
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointPointVertical(a, line)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointPointVertical(a, ID(999))), std::runtime_error);
    EXPECT_EQ(m.requirementCount(), count);
}

TEST(DCMManagerPointPointVertical, DragKeepsConstraintsWithAnotherFreePoint) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(0, 0));
    const auto p = m.addFigure(FigureDescriptor::point(0, 10));
    const auto q = m.addFigure(FigureDescriptor::point(0, 20));
    m.addRequirement(RequirementDescriptor::fixPoint(a));
    m.addRequirement(RequirementDescriptor::pointPointVertical(a, p));
    m.addRequirement(RequirementDescriptor::pointPointDist(a, p, 10));
    m.addRequirement(RequirementDescriptor::pointPointDist(p, q, 10));
    m.setSolveMode(SolveMode::DRAG);
    ASSERT_TRUE(m.updatePoint({p, 4, 20}));
    const auto point = *m.getFigure(p), other = *m.getFigure(q);
    EXPECT_NEAR(*point.x, 0, 1e-6);
    EXPECT_NEAR(std::hypot(*point.y, *point.x), 10, 1e-6);
    EXPECT_NEAR(std::hypot(*other.y - *point.y, *other.x - *point.x), 10, 1e-6);
    EXPECT_DOUBLE_EQ(*m.getFigure(a)->y, 0);
    EXPECT_DOUBLE_EQ(*m.getFigure(a)->x, 0);
}

TEST(DCMManagerPointPointVertical, BothAlignmentsCanMakePointsCoincident) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(1, 2));
    const auto p = m.addFigure(FigureDescriptor::point(9, 10));
    m.addRequirement(RequirementDescriptor::fixPoint(a));
    m.addRequirement(RequirementDescriptor::pointPointHorizontal(a, p));
    m.addRequirement(RequirementDescriptor::pointPointVertical(a, p));
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(*m.getFigure(p)->x, 1, 1e-6);
    EXPECT_NEAR(*m.getFigure(p)->y, 2, 1e-6);
    EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
    m.setSolveMode(SolveMode::DRAG);
    ASSERT_TRUE(m.updatePoint({p, 5, 6}));
    EXPECT_NEAR(*m.getFigure(p)->x, 1, 1e-6);
    EXPECT_NEAR(*m.getFigure(p)->y, 2, 1e-6);
}

TEST(DCMManagerPointPointVertical, FactoryAndSystemUseLiveXCoordinates) {
    Figures::GeometryStorage storage;
    const auto a = storage.createPoint(3, 2), b = storage.createPoint(17, 5);
    auto* first = storage.get<Figures::Point2D>(a);
    auto* second = storage.get<Figures::Point2D>(b);
    const auto row = Function::RequirementFunctionFactory::createPointPointVertical(first, second);
    EXPECT_EQ(row->getVars(), (std::vector<VAR>{first->ptrX(), second->ptrX()}));
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), -14);
    second->x() = 3;
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), 0);
    System::RequirementSystem system(&storage);
    system.addPointPointVertical(a, b);
    ASSERT_EQ(system.getFunctions().size(), 1u);
    EXPECT_EQ(system.getFunctions()[0]->getType(), RequirementType::ET_POINTPOINTVERTICAL);
    second->x() = 5;
    EXPECT_DOUBLE_EQ(system.getFunctions()[0]->mathematical()->evaluate(), -2);
}
