#include <gtest/gtest.h>
#include "ConstraintDecomposition.h"
#include "RequirementSystem.h"
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <vector>

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;
using OurPaintDCM::Function::RequirementFunctionFactory;

namespace {

std::optional<ConstraintDecomposition> planFor(const RequirementSystem& system) {
    return buildConstraintDecomposition(system, decomposeByArticulationPoints(system.buildDependencyGraph()));
}

void expectPointers(std::vector<VAR> actual, std::vector<VAR> expected) {
    std::sort(actual.begin(), actual.end(), std::less<VAR>{});
    std::sort(expected.begin(), expected.end(), std::less<VAR>{});
    EXPECT_EQ(actual, expected);
}

void expectExactOwnership(const RequirementSystem& system, const ConstraintDecomposition& plan) {
    std::vector<unsigned> functionCounts(system.getFunctions().size());
    std::unordered_map<ID, std::size_t> owners;
    for (std::size_t i = 0; i < plan.blocks.size(); ++i) {
        for (const auto requirement : plan.blocks[i].requirements)
            EXPECT_TRUE(owners.emplace(requirement, i).second);
    }
    EXPECT_EQ(owners.size(), system.getRequirements().size());
    for (const auto& requirement : system.getRequirements()) EXPECT_TRUE(owners.contains(requirement.id));
    for (std::size_t i = 0; i < plan.blocks.size(); ++i) {
        for (const auto function : plan.blocks[i].functionIndices) {
            ASSERT_LT(function, functionCounts.size());
            ++functionCounts[function];
            if (const auto requirement = system.getFunctions()[function]->requirementId()) {
                ASSERT_TRUE(owners.contains(*requirement));
                EXPECT_EQ(owners.at(*requirement), i);
            }
        }
    }
    for (const auto count : functionCounts) EXPECT_EQ(count, 1U);
}

class ConstraintDecompositionTest : public ::testing::Test {
protected:
    GeometryStorage storage;
    ID a = storage.createPoint(0, 0);
    ID b = storage.createPoint(10, 0);
    ID c = storage.createPoint(20, 0);
    ID d = storage.createPoint(30, 0);
};

TEST_F(ConstraintDecompositionTest, ChainPreservesFunctionsAndUsesLiveSharedParameters) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    const auto ab = system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto bc = system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    const auto fixed = system.addRequirement(RequirementDescriptor::fixPoint(a));
    const auto functions = system.getFunctions();
    const auto plan = planFor(system);
    ASSERT_TRUE(plan);
    ASSERT_EQ(plan->blocks.size(), 2U);
    EXPECT_EQ(plan->blocks[0].requirements, (std::vector<ID>{ab, fixed}));
    EXPECT_EQ(plan->blocks[1].requirements, (std::vector<ID>{bc}));
    EXPECT_EQ(plan->articulationPoints, (std::vector<ID>{b}));
    expectExactOwnership(system, *plan);
    EXPECT_EQ(system.getFunctions(), functions);
    auto* point = storage.get<Point2D>(b);
    expectPointers(plan->sharedVariables, {point->ptrX(), point->ptrY()});
    EXPECT_DOUBLE_EQ(point->x(), 10);
    EXPECT_DOUBLE_EQ(point->y(), 0);
    const auto sharedX = std::find(plan->sharedVariables.begin(), plan->sharedVariables.end(), point->ptrX());
    ASSERT_NE(sharedX, plan->sharedVariables.end());
    point->x() = 12;
    EXPECT_DOUBLE_EQ(**sharedX, 12);
}

TEST_F(ConstraintDecompositionTest, OnePointCanSeparateThreeNumericalBlocks) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    for (auto point : {b, c, d}) system.addRequirement(RequirementDescriptor::pointPointDist(a, point, 10));
    const auto plan = planFor(system);
    ASSERT_TRUE(plan);
    ASSERT_EQ(plan->blocks.size(), 3U);
    for (const auto& block : plan->blocks) EXPECT_EQ(block.functionIndices.size(), 1U);
    expectPointers(plan->sharedVariables, system.getFigureVariables(a));
    expectExactOwnership(system, *plan);
}

TEST_F(ConstraintDecompositionTest, ParallelRequirementsMultipleRowsAndDisabledFunctionsStayPresent) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    system.addRequirement(RequirementDescriptor::symmetricAboutPoint(a, b, c));
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    auto disabled = RequirementDescriptor::pointPointHorizontal(a, b);
    disabled.weight = 0;
    system.addRequirement(disabled);
    system.addRequirement(RequirementDescriptor::fixPoint(c));
    system.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    const auto plan = planFor(system);
    ASSERT_TRUE(plan);
    ASSERT_EQ(plan->blocks.size(), 2U);
    EXPECT_EQ(plan->blocks[0].functionIndices.size(), 6U);
    EXPECT_EQ(plan->blocks[1].functionIndices.size(), 1U);
    expectExactOwnership(system, *plan);
    const auto disabledRows = std::count_if(system.getFunctions().begin(), system.getFunctions().end(),
        [](const auto& function) { return function->getWeight() == 0; });
    EXPECT_EQ(disabledRows, 1);
}

TEST_F(ConstraintDecompositionTest, LineArticulationIncludesBothEndpointsAndRetainsConnectorBlock) {
    const auto line = storage.createLine(a, b);
    ASSERT_TRUE(line);
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, *line});
    system.addRequirement(RequirementDescriptor::horizontal(*line));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    const auto plan = planFor(system);
    ASSERT_TRUE(plan);
    ASSERT_EQ(plan->blocks.size(), 3U);
    EXPECT_NE(std::find(plan->articulationPoints.begin(), plan->articulationPoints.end(), *line),
              plan->articulationPoints.end());
    expectPointers(plan->sharedVariables, system.getFigureVariables(*line));
    EXPECT_EQ(std::count_if(plan->blocks.begin(), plan->blocks.end(),
        [](const auto& block) { return block.functionIndices.empty(); }), 1);
    expectExactOwnership(system, *plan);
}

TEST_F(ConstraintDecompositionTest, ImplicitArcFunctionWithoutRequirementIdHasOneOwner) {
    const auto arc = storage.createArc(a, c, b);
    const auto circle = storage.createCircle(b, 5);
    ASSERT_TRUE(arc && circle);
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, *arc, *circle});
    system.addRequirement(RequirementDescriptor::circleRadius(*circle, 6));
    const auto plan = planFor(system);
    ASSERT_TRUE(plan);
    ASSERT_EQ(system.getFunctions().size(), 2U);
    EXPECT_EQ(std::count_if(system.getFunctions().begin(), system.getFunctions().end(),
        [](const auto& function) { return !function->requirementId(); }), 1);
    expectExactOwnership(system, *plan);
    expectPointers(plan->sharedVariables, system.getFigureVariables(*arc));
}

TEST_F(ConstraintDecompositionTest, CircleFixationKeepsAllThreeRowsTogether) {
    const auto circle = storage.createCircle(a, 5);
    ASSERT_TRUE(circle);
    RequirementSystem system(&storage, std::vector<ID>{a, b, *circle});
    system.addRequirement(RequirementDescriptor::fixCircle(*circle));
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto plan = planFor(system);
    ASSERT_TRUE(plan);
    expectExactOwnership(system, *plan);
    expectPointers(plan->sharedVariables, system.getFigureVariables(a));
    EXPECT_EQ(std::count_if(plan->blocks.begin(), plan->blocks.end(),
        [](const auto& block) { return block.functionIndices.size() == 3; }), 1);
}

TEST_F(ConstraintDecompositionTest, CurveArticulationUsesUniqueControlCoordinates) {
    const auto curve = storage.createCurve(2, {a, b, a}, {1, 1, 1}, {0, 0, 0, 1, 1, 1});
    ASSERT_TRUE(curve);
    RequirementSystem system(&storage, std::vector<ID>{a, b, *curve});
    system.addRequirement(RequirementDescriptor::fixPoint(a));
    system.addRequirement(RequirementDescriptor::fixPoint(b));
    const auto plan = planFor(system);
    ASSERT_TRUE(plan);
    EXPECT_EQ(plan->sharedVariables.size(), 4U);
    expectPointers(plan->sharedVariables, system.getFigureVariables(*curve));
    expectExactOwnership(system, *plan);
}

TEST_F(ConstraintDecompositionTest, CoincidenceWithoutFunctionsIsRetainedAndSharesRepresentative) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto coincidence = system.addRequirement(RequirementDescriptor::pointOnPoint(b, c));
    system.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    const auto plan = planFor(system);
    ASSERT_TRUE(plan);
    ASSERT_EQ(plan->blocks.size(), 3U);
    EXPECT_EQ(plan->blocks[1].requirements, (std::vector<ID>{coincidence}));
    EXPECT_TRUE(plan->blocks[1].functionIndices.empty());
    expectPointers(plan->sharedVariables, system.getFigureVariables(c));
    EXPECT_EQ(plan->sharedVariables.size(), 2U);
    expectExactOwnership(system, *plan);
}

TEST_F(ConstraintDecompositionTest, RejectsFunctionSpanningSeparateBlocks) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    ASSERT_TRUE(planFor(system));
    system.addFunction(RequirementFunctionFactory::createPointPointDist(
        storage.get<Point2D>(a), storage.get<Point2D>(c), 20));
    EXPECT_FALSE(planFor(system));
}

TEST_F(ConstraintDecompositionTest, RejectsRequirementOutsideSelectedScope) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(c, d, 10));
    EXPECT_FALSE(planFor(system));
}

TEST_F(ConstraintDecompositionTest, RejectsUnrepresentedFunctionParameter) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    system.addFunction(RequirementFunctionFactory::bind<FixCoordinateError>(
        RequirementType::ET_FIXPOINT, {storage.get<Point2D>(d)->ptrX()}, 30));
    EXPECT_FALSE(planFor(system));
}

TEST_F(ConstraintDecompositionTest, RejectsSharedParametersOutsideArticulationState) {
    const auto first = storage.createLine(a, b);
    const auto second = storage.createLine(b, c);
    ASSERT_TRUE(first && second);
    RequirementSystem system(&storage, std::vector<ID>{a, b, c, d, *first, *second});
    system.addRequirement(RequirementDescriptor::horizontal(*first));
    system.addRequirement(RequirementDescriptor::vertical(*second));
    // This proposed boundary misses the actual shared endpoint b.
    ArticulationDecomposition unsafe{{{d, *first}, {d, *second}}, {d}};
    EXPECT_FALSE(buildConstraintDecomposition(system, std::move(unsafe)));
}

TEST_F(ConstraintDecompositionTest, ParameterlessSeparatorDoesNotInventFixedCoordinates) {
    RequirementSystem system(&storage, std::vector<ID>{a, b});
    system.addRequirement(RequirementDescriptor::fixPoint(a));
    system.addRequirement(RequirementDescriptor::fixPoint(b));
    const ID connector(999);
    ArticulationDecomposition topology{{{a, connector}, {b, connector}}, {connector}};
    const auto plan = buildConstraintDecomposition(system, std::move(topology));
    ASSERT_TRUE(plan);
    EXPECT_TRUE(plan->sharedVariables.empty());
    expectExactOwnership(system, *plan);
}

TEST_F(ConstraintDecompositionTest, NoArticulationsOrOnlyOneNumericalBlockUsesOrdinarySolve) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, c});
    EXPECT_FALSE(planFor(system));
    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 10));
    system.addRequirement(RequirementDescriptor::pointPointDist(a, c, 20));
    EXPECT_FALSE(planFor(system));

    const auto line = storage.createLine(a, b);
    ASSERT_TRUE(line);
    RequirementSystem single(&storage, std::vector<ID>{a, b, *line});
    single.addRequirement(RequirementDescriptor::horizontal(*line));
    EXPECT_FALSE(planFor(single));
}

TEST_F(ConstraintDecompositionTest, LargeCurveUsesOneSharedParameterListForAllBlocks) {
    constexpr std::size_t count = 5000;
    std::vector<ID> controls;
    for (std::size_t i = 0; i < count; ++i) controls.push_back(storage.createPoint(i, 0));
    std::vector<double> knots(count + 2);
    for (std::size_t i = 1; i <= count; ++i) knots[i] = i - 1;
    knots.back() = count - 1;
    const auto curve = storage.createCurve(1, controls, std::vector<double>(count, 1), knots);
    ASSERT_TRUE(curve);
    RequirementSystem system(&storage, std::vector<ID>{*curve});
    system.addRequirement(RequirementDescriptor::fixPoint(controls.front()));
    system.addRequirement(RequirementDescriptor::fixPoint(controls.back()));
    ArticulationDecomposition topology;
    topology.articulationPoints.push_back(*curve);
    for (auto control : controls) topology.blocks.push_back({control, *curve});

    const auto plan = buildConstraintDecomposition(system, std::move(topology));
    ASSERT_TRUE(plan);
    EXPECT_EQ(plan->blocks.size(), count);
    EXPECT_EQ(plan->sharedVariables.size(), 2 * count);
    expectExactOwnership(system, *plan);
    expectPointers(plan->sharedVariables, system.getFigureVariables(*curve));
}

} // namespace
