#include <gtest/gtest.h>
#include "GeometryGraphBuilder.h"
#include "RequirementSystem.h"
#include <algorithm>
#include <vector>

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

namespace {

class RequirementSystemGraphTest : public ::testing::Test {
protected:
    GeometryStorage storage;
    ID a, b, center, line, circle, arc, curve, outsidePoint, outsideLine;

    void SetUp() override {
        a = storage.createPoint(0, 0);
        b = storage.createPoint(10, 0);
        center = storage.createPoint(5, 5);
        const auto l = storage.createLine(a, b);
        const auto c = storage.createCircle(center, 5);
        const auto ar = storage.createArc(a, b, center);
        const auto cv = storage.createCurve(2, {a, b, a}, {1, 1, 1}, {0, 0, 0, 1, 1, 1});
        ASSERT_TRUE(l && c && ar && cv);
        line = *l;
        circle = *c;
        arc = *ar;
        curve = *cv;
        outsidePoint = storage.createPoint(100, 100);
        const auto outside = storage.createLine(center, outsidePoint);
        ASSERT_TRUE(outside);
        outsideLine = *outside;
    }

    void expectVertices(const ObjectGraph& graph, std::vector<ID> expected) const {
        auto actual = graph.getVertices();
        std::sort(actual.begin(), actual.end());
        std::sort(expected.begin(), expected.end());
        EXPECT_EQ(actual, expected);
    }
};

TEST_F(RequirementSystemGraphTest, ScopedGraphIncludesDefiningPointsForEveryFigureType) {
    RequirementSystem system(&storage, std::vector<ID>{line, circle, arc, curve});
    const auto functionsBefore = system.getFunctions();
    const auto graph = system.buildDependencyGraph();

    expectVertices(graph, {a, b, center, line, circle, arc, curve});
    EXPECT_EQ(graph.edgeCount(), 8U);
    EXPECT_TRUE(graph.hasEdge(line, a));
    EXPECT_TRUE(graph.hasEdge(line, b));
    EXPECT_TRUE(graph.hasEdge(circle, center));
    EXPECT_TRUE(graph.hasEdge(arc, a));
    EXPECT_TRUE(graph.hasEdge(arc, b));
    EXPECT_TRUE(graph.hasEdge(arc, center));
    EXPECT_TRUE(graph.hasEdge(curve, a));
    EXPECT_TRUE(graph.hasEdge(curve, b));
    EXPECT_FALSE(graph.hasVertex(outsideLine));
    EXPECT_FALSE(graph.hasVertex(outsidePoint));
    // The implicit arc equation stays in the existing system, not in the graph.
    ASSERT_EQ(functionsBefore.size(), 1U);
    EXPECT_EQ(functionsBefore.front()->getType(), RequirementType::ET_ARCCENTERONPERPENDICULAR);
    EXPECT_EQ(system.getFunctions(), functionsBefore);
}

TEST_F(RequirementSystemGraphTest, PointScopeDoesNotExpandToIncidentFigures) {
    RequirementSystem system(&storage, std::vector<ID>{center});
    const auto graph = system.buildDependencyGraph();
    expectVertices(graph, {center});
    EXPECT_EQ(graph.edgeCount(), 0U);
}

TEST_F(RequirementSystemGraphTest, EmptyScopeDoesNotBuildWholeStorageGraph) {
    RequirementSystem scoped(&storage, std::vector<ID>{});
    EXPECT_EQ(scoped.buildDependencyGraph().vertexCount(), 0U);
    RequirementSystem whole(&storage);
    EXPECT_EQ(whole.buildDependencyGraph().vertexCount(), storage.size());
    EXPECT_TRUE(whole.buildDependencyGraph().hasEdge(outsideLine, outsidePoint));
}

TEST_F(RequirementSystemGraphTest, ConstraintTopologyKeepsAllFunctionsInTheirOriginalSystem) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, center});
    const auto symmetry = system.addRequirement(RequirementDescriptor::symmetricAboutPoint(a, b, center));
    const auto distance = system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto fixed = system.addRequirement(RequirementDescriptor::fixPoint(center));
    const auto functionsBefore = system.getFunctions();
    const auto graph = system.buildDependencyGraph();

    expectVertices(graph, {a, b, center});
    EXPECT_EQ(graph.edgeCount(), 3U);
    EXPECT_TRUE(graph.hasEdge(a, b));
    EXPECT_TRUE(graph.hasEdge(a, center));
    EXPECT_TRUE(graph.hasEdge(b, center));
    EXPECT_FALSE(graph.hasEdge(center, center));
    EXPECT_EQ(system.getFunctions(), functionsBefore);
    const auto countRows = [&](ID requirement) {
        return std::count_if(functionsBefore.begin(), functionsBefore.end(), [&](const auto& function) {
            return function->requirementId() == requirement;
        });
    };
    EXPECT_EQ(countRows(symmetry), 2);
    EXPECT_EQ(countRows(distance), 1);
    EXPECT_EQ(countRows(fixed), 2);
}

TEST_F(RequirementSystemGraphTest, FigureParametersReferenceExistingGeometryWithoutDuplicates) {
    RequirementSystem system(&storage, std::vector<ID>{line, circle, arc, curve});
    auto* p = storage.get<Point2D>(a);
    auto* q = storage.get<Point2D>(b);
    auto* c = storage.get<Point2D>(center);
    auto* radius = storage.get<Circle2D>(circle)->ptrRadius();

    EXPECT_EQ(system.getFigureVariables(a), (std::vector<VAR>{p->ptrX(), p->ptrY()}));
    const std::vector<VAR> endpoints{p->ptrX(), p->ptrY(), q->ptrX(), q->ptrY()};
    EXPECT_EQ(system.getFigureVariables(line), endpoints);
    EXPECT_EQ(system.getFigureVariables(curve), endpoints);
    EXPECT_EQ(system.getFigureVariables(circle), (std::vector<VAR>{c->ptrX(), c->ptrY(), radius}));
    EXPECT_EQ(system.getFigureVariables(arc),
              (std::vector<VAR>{p->ptrX(), p->ptrY(), q->ptrX(), q->ptrY(), c->ptrX(), c->ptrY()}));

    const auto parameters = system.getFigureVariables(circle);
    c->x() = 25;
    *radius = 7;
    EXPECT_DOUBLE_EQ(*parameters[0], 25);
    EXPECT_DOUBLE_EQ(*parameters[2], 7);
    EXPECT_TRUE(system.getFigureVariables(ID(999)).empty());
}

TEST_F(RequirementSystemGraphTest, CoincidentNodesUseSameParametersAsConstraintFunctions) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, line, curve, center});
    system.addRequirement(RequirementDescriptor::pointOnPoint(a, b));
    const auto distance = system.addRequirement(RequirementDescriptor::pointPointDist(a, center, 5));
    auto* representative = storage.get<Point2D>(b);
    auto* c = storage.get<Point2D>(center);
    const std::vector<VAR> shared{representative->ptrX(), representative->ptrY()};

    const auto graph = system.buildDependencyGraph();
    expectVertices(graph, {a, b, line, curve, center});
    EXPECT_TRUE(graph.hasEdge(a, b));
    EXPECT_EQ(system.getFigureVariables(a), shared);
    EXPECT_EQ(system.getFigureVariables(b), shared);
    EXPECT_EQ(system.getFigureVariables(line), shared);
    EXPECT_EQ(system.getFigureVariables(curve), shared);
    ASSERT_EQ(system.getFunctions().size(), 1U);
    EXPECT_EQ(system.getFunctions().front()->requirementId(), distance);
    EXPECT_EQ(system.getFunctions().front()->getVars(),
              (std::vector<VAR>{representative->ptrX(), representative->ptrY(), c->ptrX(), c->ptrY()}));

    system.clear();
    EXPECT_FALSE(system.buildDependencyGraph().hasEdge(a, b));
    EXPECT_NE(system.getFigureVariables(a), system.getFigureVariables(b));
    EXPECT_EQ(system.getFigureVariables(line).size(), 4U);
}

TEST_F(RequirementSystemGraphTest, RebuildingGraphReflectsConstraintChangesWithoutMutatingEarlierGraph) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, center});
    const auto before = system.buildDependencyGraph();
    EXPECT_EQ(before.edgeCount(), 0U);

    system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 10));
    const auto after = system.buildDependencyGraph();
    EXPECT_TRUE(after.hasEdge(a, b));
    EXPECT_FALSE(before.hasEdge(a, b));
    expectVertices(after, {a, b, center});

    system.clear();
    EXPECT_EQ(system.buildDependencyGraph().edgeCount(), 0U);
}

TEST_F(RequirementSystemGraphTest, DisabledCoincidenceDoesNotAliasParameters) {
    RequirementSystem system(&storage, std::vector<ID>{a, b, line});
    auto coincidence = RequirementDescriptor::pointOnPoint(a, b);
    coincidence.weight = 0;
    system.addRequirement(coincidence);

    EXPECT_NE(system.getFigureVariables(a), system.getFigureVariables(b));
    EXPECT_EQ(system.getFigureVariables(line).size(), 4U);
    // Topology retains disabled requirement edges just like the existing full graph.
    EXPECT_TRUE(system.buildDependencyGraph().hasEdge(a, b));
}

TEST_F(RequirementSystemGraphTest, ScopedBuilderIgnoresRepeatedAndRemovedIds) {
    const auto removed = storage.createPoint(200, 200);
    ASSERT_EQ(storage.remove(removed), RemoveResult::Ok);
    const std::vector<ID> scope{line, line, a, removed};
    const auto graph = GeometryGraphBuilder::buildObjectGraph(storage, scope);
    expectVertices(graph, {line, a, b});
    EXPECT_EQ(graph.edgeCount(), 2U);
}

} // namespace
