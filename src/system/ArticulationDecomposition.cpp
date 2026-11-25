#include "ArticulationDecomposition.h"
#include <algorithm>
#include <unordered_map>
#include <utility>

namespace OurPaintDCM::System {
namespace {

using Utils::ID;

class BlockSearch {
    struct VertexState {
        std::size_t discovery;
        std::size_t low;
        std::size_t blockCount = 0;
    };
    struct Frame {
        ID vertex;
        std::vector<Edge<ID, ID>> edges;
        std::size_t nextEdge = 0;
    };

    const Figures::ObjectGraph& graph;
    std::unordered_map<ID, VertexState> states;
    std::vector<Frame> frames;
    std::vector<ID> pendingVertices;
    ArticulationDecomposition result;

    void discover(ID vertex) {
        const auto discovery = states.size() + 1;
        states.emplace(vertex, VertexState{discovery, discovery});
        pendingVertices.push_back(vertex);
        // Graph exposes neighbours by value; keep one list only while this frame is active.
        frames.push_back({vertex, graph.getVertexEdges(vertex)});
    }

    void emitBlock(ID parent, ID child) {
        std::vector<ID> block{parent};
        ++states.at(parent).blockCount;
        ID vertex;
        do {
            vertex = pendingVertices.back();
            pendingVertices.pop_back();
            block.push_back(vertex);
            ++states.at(vertex).blockCount;
        } while (vertex != child);
        std::sort(block.begin(), block.end());
        result.blocks.push_back(std::move(block));
    }

    void finishVertex() {
        const auto vertex = frames.back().vertex;
        const auto& state = states.at(vertex);
        frames.pop_back();
        if (frames.empty()) {
            if (state.blockCount == 0) result.blocks.push_back({vertex});
            pendingVertices.pop_back();
            return;
        }

        const auto parent = frames.back().vertex;
        auto& parentState = states.at(parent);
        parentState.low = std::min(parentState.low, state.low);
        // This subtree cannot bypass parent; parent stays pending for its other blocks.
        if (state.low >= parentState.discovery) emitBlock(parent, vertex);
    }

    void visitComponent(ID root) {
        discover(root);
        while (!frames.empty()) {
            auto& frame = frames.back();
            if (frame.nextEdge == frame.edges.size()) {
                finishVertex();
                continue;
            }

            const auto neighbour = frame.edges[frame.nextEdge++].to;
            if (neighbour == frame.vertex ||
                (frames.size() > 1 && neighbour == frames[frames.size() - 2].vertex)) continue;

            const auto found = states.find(neighbour);
            if (found == states.end()) {
                discover(neighbour);
            } else {
                auto& state = states.at(frame.vertex);
                state.low = std::min(state.low, found->second.discovery);
            }
        }
    }

public:
    explicit BlockSearch(const Figures::ObjectGraph& graph) : graph(graph) {
        states.reserve(graph.vertexCount());
        frames.reserve(graph.vertexCount());
        pendingVertices.reserve(graph.vertexCount());
    }

    ArticulationDecomposition run() {
        for (const auto vertex : graph.getVertices())
            if (!states.contains(vertex)) visitComponent(vertex);
        for (const auto& [vertex, state] : states)
            if (state.blockCount > 1) result.articulationPoints.push_back(vertex);
        std::sort(result.articulationPoints.begin(), result.articulationPoints.end());
        std::sort(result.blocks.begin(), result.blocks.end());
        return std::move(result);
    }
};

} // namespace

ArticulationDecomposition decomposeByArticulationPoints(const Figures::ObjectGraph& graph) {
    return BlockSearch(graph).run();
}

} // namespace OurPaintDCM::System
