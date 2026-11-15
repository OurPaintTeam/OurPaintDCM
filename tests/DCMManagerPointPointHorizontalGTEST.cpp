#include "DCMManager.h"
#include "RequirementFunctionFactory.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerPointPointHorizontal, SolvesBothOrdersAndKeepsHorizontalFreedom) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (bool reversed : {false, true}) {
            DCMManager m;
            const auto a = m.addFigure(FigureDescriptor::point(2, 3));
            const auto p = m.addFigure(FigureDescriptor::point(-5, 17));
            m.addRequirement(RequirementDescriptor::fixPoint(a));
            const auto id = m.addRequirement(reversed
                ? RequirementDescriptor::pointPointHorizontal(p, a)
                : RequirementDescriptor::pointPointHorizontal(a, p));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(p)));
            EXPECT_DOUBLE_EQ(*m.getFigure(p)->x, -5);
            EXPECT_NEAR(*m.getFigure(p)->y, 3, 1e-6);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 1u);
            EXPECT_EQ(m.figureCount(), 2u);
            EXPECT_FALSE(m.getRequirement(id)->param);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updatePoint({p, 8, -9}));
                EXPECT_NEAR(*m.getFigure(p)->y, 3, 1e-6);
                EXPECT_DOUBLE_EQ(*m.getFigure(p)->x, 8);
            }
        }
    }
}

TEST(DCMManagerPointPointHorizontal, WeightsSnapshotsAndRemoval) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(2, 3));
    const auto p = m.addFigure(FigureDescriptor::point(5, 17));
    m.addRequirement(RequirementDescriptor::fixPoint(a));
    const auto id = m.addRequirement(RequirementDescriptor::pointPointHorizontal(a, p));
    m.updateRequirementWeight(id, 2);
    ASSERT_TRUE(m.solve());
    const auto saved = m.snapshot();
    m.updateRequirementWeight(id, 0);
    ASSERT_TRUE(m.updatePoint({p, 8, 9}));
    ASSERT_TRUE(m.solve());
    EXPECT_DOUBLE_EQ(*m.getFigure(p)->y, 9);
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(*m.getFigure(p)->y, 3, 1e-6);
    EXPECT_DOUBLE_EQ(m.getRequirement(id)->weight, 2);
    m.removeRequirement(id);
    EXPECT_NE(m.getComponentForFigure(a), m.getComponentForFigure(p));
}

TEST(DCMManagerPointPointHorizontal, CoincidentPointsRepeatedIdAndAliasesAreValid) {
    for (int variant = 0; variant < 3; ++variant) {
        DCMManager m;
        const auto a = m.addFigure(FigureDescriptor::point(2, 3));
        const auto p = variant == 0 ? a : m.addFigure(FigureDescriptor::point(2, 3));
        if (variant == 2) m.addRequirement(RequirementDescriptor::pointOnPoint(a, p));
        m.addRequirement(RequirementDescriptor::pointPointHorizontal(a, p));
        ASSERT_TRUE(m.solve());
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, variant == 1 ? 3u : 2u);
    }
}

TEST(DCMManagerPointPointHorizontal, IncompatibleFixedPointsAndInvalidDescriptorsFail) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(2, 3));
    const auto p = m.addFigure(FigureDescriptor::point(5, 17));
    const auto line = m.addFigure(FigureDescriptor::line(a, p));
    m.addRequirement(RequirementDescriptor::fixPoint(a));
    m.addRequirement(RequirementDescriptor::fixPoint(p));
    auto descriptor = RequirementDescriptor::pointPointHorizontal(a, p);
    const auto id = m.addRequirement(descriptor);
    EXPECT_FALSE(m.solve());
    EXPECT_DOUBLE_EQ(*m.getFigure(a)->y, 3);
    EXPECT_DOUBLE_EQ(*m.getFigure(p)->y, 17);
    EXPECT_THROW(m.updateRequirementParam(id, 0), std::runtime_error);
    descriptor.param = 0;
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    descriptor.param.reset();
    descriptor.objectIds.pop_back();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    const auto count = m.requirementCount();
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointPointHorizontal(a, line)), std::runtime_error);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointPointHorizontal(a, ID(999))), std::runtime_error);
    EXPECT_EQ(m.requirementCount(), count);
}

TEST(DCMManagerPointPointHorizontal, DragKeepsConstraintsWithAnotherFreePoint) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(0, 0));
    const auto p = m.addFigure(FigureDescriptor::point(10, 0));
    const auto q = m.addFigure(FigureDescriptor::point(20, 0));
    m.addRequirement(RequirementDescriptor::fixPoint(a));
    m.addRequirement(RequirementDescriptor::pointPointHorizontal(a, p));
    m.addRequirement(RequirementDescriptor::pointPointDist(a, p, 10));
    m.addRequirement(RequirementDescriptor::pointPointDist(p, q, 10));
    m.setSolveMode(SolveMode::DRAG);
    ASSERT_TRUE(m.updatePoint({p, 20, 4}));
    const auto point = *m.getFigure(p), other = *m.getFigure(q);
    EXPECT_NEAR(*point.y, 0, 1e-6);
    EXPECT_NEAR(std::hypot(*point.x, *point.y), 10, 1e-6);
    EXPECT_NEAR(std::hypot(*other.x - *point.x, *other.y - *point.y), 10, 1e-6);
    EXPECT_DOUBLE_EQ(*m.getFigure(a)->x, 0);
    EXPECT_DOUBLE_EQ(*m.getFigure(a)->y, 0);
}

TEST(DCMManagerPointPointHorizontal, FactoryAndSystemUseLiveYCoordinates) {
    Figures::GeometryStorage storage;
    const auto a = storage.createPoint(2, 3), b = storage.createPoint(5, 17);
    auto* first = storage.get<Figures::Point2D>(a);
    auto* second = storage.get<Figures::Point2D>(b);
    const auto row = Function::RequirementFunctionFactory::createPointPointHorizontal(first, second);
    EXPECT_EQ(row->getVars(), (std::vector<VAR>{first->ptrY(), second->ptrY()}));
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), -14);
    second->y() = 3;
    EXPECT_DOUBLE_EQ(row->mathematical()->evaluate(), 0);
    System::RequirementSystem system(&storage);
    system.addPointPointHorizontal(a, b);
    ASSERT_EQ(system.getFunctions().size(), 1u);
    EXPECT_EQ(system.getFunctions()[0]->getType(), RequirementType::ET_POINTPOINTHORIZONTAL);
    second->y() = 5;
    EXPECT_DOUBLE_EQ(system.getFunctions()[0]->mathematical()->evaluate(), -2);
}
