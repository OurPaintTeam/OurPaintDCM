#include <gtest/gtest.h>
#include "ConstraintSolveOrder.h"
#include "RequirementSystem.h"
#include <algorithm>
#include <array>
#include <limits>

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

namespace {

class ConstraintSolveOrderTest : public ::testing::Test {
protected:
    GeometryStorage storage;
    ID a = storage.createPoint(0, 0);
    ID b = storage.createPoint(10, 0);
    ID c = storage.createPoint(20, 0);
    ID d = storage.createPoint(30, 0);

    ConstraintDecomposition chain(const RequirementSystem& system) {
        auto plan = buildConstraintDecomposition(system,
            decomposeByArticulationPoints(system.buildDependencyGraph()));
        EXPECT_TRUE(plan);
        return plan.value();
    }
};

TEST_F(ConstraintSolveOrderTest, FixedGeometryWinsOverEditedGeometry) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    system.addRequirement(RequirementDescriptor::fixPoint(c));
    const auto plan = chain(system);
    const auto order = chooseConstraintSolveOrder(system, plan, std::array{a});
    EXPECT_EQ(order, (std::vector<std::size_t>{1, 0}));
}

TEST_F(ConstraintSolveOrderTest, EditedGeometrySelectsEitherEndWithoutChangingPlan) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    const auto plan = chain(system);
    EXPECT_EQ(chooseConstraintSolveOrder(system, plan, std::array{d}),
              (std::vector<std::size_t>{2, 1, 0}));
    EXPECT_EQ(chooseConstraintSolveOrder(system, plan, std::array{a}),
              (std::vector<std::size_t>{0, 1, 2}));
}

TEST_F(ConstraintSolveOrderTest, FewerDegreesOfFreedomWinsBeforeStructure) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointHorizontal(b, c));
    system.addRequirement(RequirementDescriptor::pointPointVertical(b, c));
    const auto plan = chain(system);
    EXPECT_EQ(chooseConstraintSolveOrder(system, plan).front(), 1U);
}

TEST_F(ConstraintSolveOrderTest, CentralBlockWinsWhenDegreesOfFreedomAreEqual) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    const auto plan = chain(system);
    EXPECT_EQ(chooseConstraintSolveOrder(system, plan), (std::vector<std::size_t>{1, 0, 2}));
}

TEST_F(ConstraintSolveOrderTest, DisabledFixationDoesNotBecomeAnAnchor) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    auto fix = RequirementDescriptor::fixPoint(a);
    fix.weight = 0;
    system.addRequirement(fix);
    EXPECT_EQ(chooseConstraintSolveOrder(system, chain(system), std::array{c}).front(), 1U);
}

TEST_F(ConstraintSolveOrderTest, EditedCoincidentPointUsesRepresentativeCoordinates) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    system.addRequirement(RequirementDescriptor::pointOnPoint(c, d));
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    const auto plan = chain(system);
    const auto order = chooseConstraintSolveOrder(system, plan, std::array{d});
    // The empty coincidence connector is traversed, but the numerical block is the root.
    EXPECT_EQ(plan.blocks[order.front()].figures, (std::vector<ID>{b, c}));
    EXPECT_EQ(order.size(), plan.blocks.size());
}

TEST_F(ConstraintSolveOrderTest, EmptyConnectorLinksNumericalBlocksAndEveryBlockIsVisitedOnce) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointOnPoint(b, c));
    system.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    const auto plan = chain(system);
    const auto order = chooseConstraintSolveOrder(system, plan, std::array{d});
    EXPECT_EQ(order, (std::vector<std::size_t>{2, 1, 0}));
    EXPECT_TRUE(plan.blocks[order[1]].functionIndices.empty());
}

TEST_F(ConstraintSolveOrderTest, TieBreakUsesFigureIdsRatherThanBlockInsertionOrder) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    auto plan = chain(system);
    const auto first = chooseConstraintSolveOrder(system, plan);
    const auto expected = plan.blocks[first.front()].figures;
    std::reverse(plan.blocks.begin(), plan.blocks.end());
    const auto second = chooseConstraintSolveOrder(system, plan);
    EXPECT_EQ(plan.blocks[second.front()].figures, expected);
    EXPECT_EQ(chooseConstraintSolveOrder(system, plan), second);
}

TEST_F(ConstraintSolveOrderTest, EmptyPlanProducesEmptyOrder) {
    RequirementSystem system(&storage, std::vector<ID>{});
    EXPECT_TRUE(chooseConstraintSolveOrder(system, {}).empty());
}

TEST_F(ConstraintSolveOrderTest, LargerBlockWinsWhenFreedomAndCentralityAreEqual) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    system.addRequirement(RequirementDescriptor::fixPoint(a));
    system.addRequirement(RequirementDescriptor::fixPoint(c));
    system.addRequirement(RequirementDescriptor::fixPoint(d));
    ConstraintDecomposition plan;
    plan.blocks = {{{a, b}, {}, {0, 1}}, {{b, c, d}, {}, {2, 3, 4, 5}}};
    plan.articulationPoints = {b};
    EXPECT_EQ(chooseConstraintSolveOrder(system, plan).front(), 1U);
}

TEST_F(ConstraintSolveOrderTest, UndefinedLinearizationLosesToDefinedCandidate) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    const auto plan = chain(system);
    storage.get<Point2D>(a)->x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(chooseConstraintSolveOrder(system, plan).front(), 1U);
}

TEST_F(ConstraintSolveOrderTest, EditedCircleIsRecognizedThroughItsDefiningPoint) {
    const auto circle = storage.createCircle(c, 5);
    ASSERT_TRUE(circle);
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    const auto plan = chain(system);
    EXPECT_EQ(chooseConstraintSolveOrder(system, plan, std::array{*circle}).front(), 1U);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(c)->x(), 20);
    EXPECT_DOUBLE_EQ(storage.get<Circle2D>(*circle)->radius, 5);
}

TEST_F(ConstraintSolveOrderTest, StarVisitsAllBranchesOnce) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    for (auto point : {b, c, d}) system.addRequirement(RequirementDescriptor::pointPointDist(a, point, 10));
    const auto plan = chain(system);
    auto order = chooseConstraintSolveOrder(system, plan, std::array{d});
    ASSERT_EQ(order.front(), 2U);
    std::sort(order.begin(), order.end());
    EXPECT_EQ(order, (std::vector<std::size_t>{0, 1, 2}));
}

} // namespace
