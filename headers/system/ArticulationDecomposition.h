#pragma once

#include "GeometryGraphBuilder.h"
#include <vector>

namespace OurPaintDCM::System {

struct ArticulationDecomposition {
    std::vector<std::vector<Utils::ID>> blocks;
    std::vector<Utils::ID> articulationPoints;
};

/** Decompose a supplied local graph into vertex-biconnected blocks.
 * Bridges form two-vertex blocks; isolated vertices form singleton blocks.
 * Self-loops do not affect decomposition. Shared vertices occur in each incident block.
 * IDs within blocks, blocks and articulation points are sorted for reproducibility,
 * not for numerical solving. Geometry and constraint functions are not copied.
 */
[[nodiscard]] ArticulationDecomposition decomposeByArticulationPoints(const Figures::ObjectGraph& graph);

} // namespace OurPaintDCM::System
