#include "GeometryGraphBuilder.h"
#include "GeometryStorage.h"

namespace OurPaintDCM::Figures {

namespace {

const Utils::ID helperWeight{static_cast<unsigned long long>(-1)};

void addVertexAll(ObjectGraph& g, const GeometryStorage& storage) {
    for (const auto& r : storage.pointsWithIds()) {
        g.addVertex(r.id);
    }
    for (const auto& r : storage.linesWithIds()) {
        g.addVertex(r.id);
    }
    for (const auto& r : storage.circlesWithIds()) {
        g.addVertex(r.id);
    }
    for (const auto& r : storage.arcsWithIds()) {
        g.addVertex(r.id);
    }
}

void wireFigureEdges(ObjectGraph& g, Utils::ID figId, const GeometryStorage& storage) {
    for (Utils::ID p : storage.getDependencies(figId)) {
        g.addVertex(figId);
        g.addVertex(p);
        g.addEdge(figId, p, helperWeight);
    }
}

} // namespace

ObjectGraph GeometryGraphBuilder::buildObjectGraph(const GeometryStorage& storage) {
    ObjectGraph graph;
    addVertexAll(graph, storage);
    for (const auto& r : storage.linesWithIds()) {
        wireFigureEdges(graph, r.id, storage);
    }
    for (const auto& r : storage.circlesWithIds()) {
        wireFigureEdges(graph, r.id, storage);
    }
    for (const auto& r : storage.arcsWithIds()) {
        wireFigureEdges(graph, r.id, storage);
    }
    return graph;
}

std::optional<ObjectGraph> GeometryGraphBuilder::buildObjectSubgraph(const GeometryStorage& storage, Utils::ID id) {
    auto typeOpt = storage.getType(id);
    if (!typeOpt.has_value()) {
        return std::nullopt;
    }

    ObjectGraph graph;
    graph.addVertex(id);

    if (*typeOpt == FigureType::ET_POINT2D) {
        for (Utils::ID depFig : storage.getDependents(id)) {
            graph.addVertex(depFig);
            graph.addEdge(id, depFig, helperWeight);
            for (Utils::ID p : storage.getDependencies(depFig)) {
                graph.addVertex(p);
                graph.addEdge(depFig, p, helperWeight);
            }
        }
        return graph;
    }

    for (Utils::ID p : storage.getDependencies(id)) {
        graph.addVertex(p);
        graph.addEdge(id, p, helperWeight);
    }
    return graph;
}

} // namespace OurPaintDCM::Figures
