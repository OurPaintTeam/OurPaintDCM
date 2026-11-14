#include <gtest/gtest.h>
#include "ArticulationDecomposition.h"
#include "RequirementSystem.h"
#include <algorithm>
#include <initializer_list>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Utils;

namespace {

using Edges = std::vector<std::pair<unsigned long long, unsigned long long>>;

ObjectGraph makeGraph(const Edges& edges, std::initializer_list<unsigned long long> isolated = {}) {
    ObjectGraph graph;
    for (auto vertex : isolated) graph.addVertex(ID(vertex));
    for (auto [a, b] : edges) {
        graph.addVertex(ID(a), ID(b));
        graph.addEdge(ID(a), ID(b), ID(1));
    }
    return graph;
}

void expectDecomposition(const ObjectGraph& graph,
                         const std::vector<std::vector<unsigned long long>>& expectedBlocks,
                         const std::vector<unsigned long long>& expectedCuts) {
    const auto result = decomposeByArticulationPoints(graph);
    std::vector<std::vector<unsigned long long>> blocks;
    for (const auto& block : result.blocks) {
        blocks.emplace_back();
        for (auto vertex : block) blocks.back().push_back(vertex.id);
    }
    std::vector<unsigned long long> cuts;
    for (auto vertex : result.articulationPoints) cuts.push_back(vertex.id);
    EXPECT_EQ(blocks, expectedBlocks);
    EXPECT_EQ(cuts, expectedCuts);
}

// Independent deletion/reachability oracle for small induced subgraphs.
std::size_t countComponents(const ObjectGraph& graph, const std::vector<ID>& vertices,
                            std::optional<ID> removed = std::nullopt) {
    std::unordered_set<ID> remaining(vertices.begin(), vertices.end());
    if (removed) remaining.erase(*removed);
    std::size_t count = 0;
    while (!remaining.empty()) {
        std::vector<ID> pending{*remaining.begin()};
        remaining.erase(pending.front());
        for (std::size_t i = 0; i < pending.size(); ++i) {
            for (auto it = remaining.begin(); it != remaining.end();) {
                if (graph.hasEdge(pending[i], *it)) {
                    pending.push_back(*it);
                    it = remaining.erase(it);
                } else {
                    ++it;
                }
            }
        }
        ++count;
    }
    return count;
}

void verifyAgainstDeletionOracle(const ObjectGraph& graph) {
    const auto result = decomposeByArticulationPoints(graph);
    const auto vertices = graph.getVertices();
    const auto components = countComponents(graph, vertices);
    std::vector<ID> expectedCuts;
    for (auto vertex : vertices) {
        const bool cut = countComponents(graph, vertices, vertex) > components;
        if (cut) expectedCuts.push_back(vertex);
        const auto memberships = std::count_if(result.blocks.begin(), result.blocks.end(), [&](const auto& block) {
            return std::find(block.begin(), block.end(), vertex) != block.end();
        });
        if (cut) EXPECT_GT(memberships, 1);
        else EXPECT_EQ(memberships, 1);
    }
    std::sort(expectedCuts.begin(), expectedCuts.end());
    EXPECT_EQ(result.articulationPoints, expectedCuts);
    EXPECT_TRUE(std::is_sorted(result.blocks.begin(), result.blocks.end()));
    for (const auto& block : result.blocks) {
        ASSERT_FALSE(block.empty());
        EXPECT_TRUE(std::is_sorted(block.begin(), block.end()));
        EXPECT_EQ(std::adjacent_find(block.begin(), block.end()), block.end());
        EXPECT_EQ(countComponents(graph, block), 1U);
        if (block.size() > 2)
            for (auto vertex : block) EXPECT_EQ(countComponents(graph, block, vertex), 1U);
    }
    for (const auto& edge : graph.getAllEdges()) {
        const auto owners = std::count_if(result.blocks.begin(), result.blocks.end(), [&](const auto& block) {
            return std::find(block.begin(), block.end(), edge.from) != block.end()
                && std::find(block.begin(), block.end(), edge.to) != block.end();
        });
        EXPECT_EQ(owners, 1);
    }
}

TEST(ArticulationDecompositionTest, EmptyGraph) {
    expectDecomposition(makeGraph({}), {}, {});
}

TEST(ArticulationDecompositionTest, SingleVertexAndSelfLoop) {
    auto graph = makeGraph({}, {0});
    expectDecomposition(graph, {{0}}, {});
    ASSERT_TRUE(graph.addEdge(ID(0), ID(0), ID(1)));
    expectDecomposition(graph, {{0}}, {});
}

TEST(ArticulationDecompositionTest, SingleBridgeHasNoArticulation) {
    expectDecomposition(makeGraph({{10, 900}}), {{10, 900}}, {});
}

TEST(ArticulationDecompositionTest, CycleHasOneBlockAndNoArticulation) {
    expectDecomposition(makeGraph({{0, 1}, {1, 2}, {2, 3}, {3, 0}}), {{0, 1, 2, 3}}, {});
}

TEST(ArticulationDecompositionTest, TwoCyclesShareOneArticulation) {
    expectDecomposition(makeGraph({{0, 1}, {1, 2}, {2, 0}, {2, 3}, {3, 4}, {4, 2}}),
                        {{0, 1, 2}, {2, 3, 4}}, {2});
}

TEST(ArticulationDecompositionTest, ChainOfBlocksContainsSeveralArticulations) {
    expectDecomposition(makeGraph({{0, 1}, {1, 2}, {2, 0}, {2, 3}, {3, 4}, {4, 5}, {5, 3}, {5, 6}}),
                        {{0, 1, 2}, {2, 3}, {3, 4, 5}, {5, 6}}, {2, 3, 5});
}

TEST(ArticulationDecompositionTest, ArticulationCanBelongToFourBlocks) {
    auto graph = makeGraph({{0, 1}, {1, 2}, {2, 0}, {0, 3}, {3, 4}, {4, 0}, {0, 5}, {0, 6}});
    expectDecomposition(graph, {{0, 1, 2}, {0, 3, 4}, {0, 5}, {0, 6}}, {0});
    ASSERT_TRUE(graph.addEdge(ID(0), ID(0), ID(1)));
    EXPECT_FALSE(graph.addEdge(ID(1), ID(0), ID(9)));
    expectDecomposition(graph, {{0, 1, 2}, {0, 3, 4}, {0, 5}, {0, 6}}, {0});
}

TEST(ArticulationDecompositionTest, DisconnectedInputKeepsIsolatedVertices) {
    expectDecomposition(makeGraph({{0, 1}, {1, 2}, {10, 11}, {11, 12}, {12, 10}}, {20}),
                        {{0, 1}, {1, 2}, {10, 11, 12}, {20}}, {1});
}

TEST(ArticulationDecompositionTest, ResultDoesNotDependOnInsertionOrderOrEdgeDirection) {
    Edges edges{{0, 1}, {1, 2}, {2, 0}, {2, 3}, {3, 4}, {4, 2}, {2, 5}, {5, 6}};
    const auto first = decomposeByArticulationPoints(makeGraph(edges));
    std::reverse(edges.begin(), edges.end());
    for (auto& [a, b] : edges) std::swap(a, b);
    const auto second = decomposeByArticulationPoints(makeGraph(edges));
    EXPECT_EQ(first.blocks, second.blocks);
    EXPECT_EQ(first.articulationPoints, second.articulationPoints);
}

TEST(ArticulationDecompositionTest, ScopedGeometryCanHaveANonPointArticulation) {
    GeometryStorage storage;
    const auto a = storage.createPoint(0, 0);
    const auto b = storage.createPoint(10, 0);
    const auto line = storage.createLine(a, b);
    ASSERT_TRUE(line);
    const auto outside = storage.createPoint(100, 100);
    RequirementSystem system(&storage, std::vector<ID>{*line});
    const auto graph = system.buildDependencyGraph();

    expectDecomposition(graph, {{a.id, line->id}, {b.id, line->id}}, {line->id});
    EXPECT_EQ(system.getFigureVariables(*line).size(), 4U);
    EXPECT_FALSE(graph.hasVertex(outside));
}

TEST(ArticulationDecompositionTest, AllGraphsOnFiveVerticesMatchDeletionOracle) {
    Edges possibleEdges;
    for (unsigned long long a = 0; a < 5; ++a)
        for (auto b = a + 1; b < 5; ++b) possibleEdges.emplace_back(a, b);
    for (unsigned mask = 0; mask < (1U << possibleEdges.size()); ++mask) {
        SCOPED_TRACE(mask);
        Edges edges;
        for (std::size_t i = 0; i < possibleEdges.size(); ++i)
            if (mask & (1U << i)) edges.push_back(possibleEdges[i]);
        verifyAgainstDeletionOracle(makeGraph(edges, {0, 1, 2, 3, 4}));
    }
}

TEST(ArticulationDecompositionTest, LongChainAndClosingEdgeDoNotRequireRecursiveTraversal) {
    constexpr unsigned long long count = 50000;
    ObjectGraph graph;
    for (unsigned long long i = 0; i < count; ++i) graph.addVertex(ID(i));
    for (unsigned long long i = 1; i < count; ++i) graph.addEdge(ID(i - 1), ID(i), ID(1));
    const auto chain = decomposeByArticulationPoints(graph);
    ASSERT_EQ(chain.blocks.size(), count - 1);
    ASSERT_EQ(chain.articulationPoints.size(), count - 2);
    for (unsigned long long i = 0; i < count - 1; ++i)
        EXPECT_EQ(chain.blocks[i], (std::vector<ID>{ID(i), ID(i + 1)}));
    for (unsigned long long i = 1; i < count - 1; ++i)
        EXPECT_EQ(chain.articulationPoints[i - 1], ID(i));

    ASSERT_TRUE(graph.addEdge(ID(0), ID(count - 1), ID(1)));
    const auto cycle = decomposeByArticulationPoints(graph);
    EXPECT_TRUE(cycle.articulationPoints.empty());
    ASSERT_EQ(cycle.blocks.size(), 1U);
    ASSERT_EQ(cycle.blocks.front().size(), count);
    for (unsigned long long i = 0; i < count; ++i) EXPECT_EQ(cycle.blocks.front()[i], ID(i));
}

} // namespace
