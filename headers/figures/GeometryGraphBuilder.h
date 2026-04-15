#ifndef OURPAINTDCM_HEADERS_FIGURES_GEOMETRYGRAPHBUILDER_H
#define OURPAINTDCM_HEADERS_FIGURES_GEOMETRYGRAPHBUILDER_H

#include "Graph.h"
#include "ID.h"

#include <optional>

// Forward declaration avoids heavy include cycle
namespace OurPaintDCM::Figures {
class GeometryStorage;
}

namespace OurPaintDCM::Figures {

using ObjectGraph = Graph<Utils::ID, Utils::ID, UndirectedPolicy, WeightedPolicy>;

/**
 * @brief Builds topology graphs using only public GeometryStorage APIs.
 *
 * Edge wiring uses getDependencies() per figure — no ad hoc scan of all objects for incidence.
 */
class GeometryGraphBuilder {
public:
    /**
     * @brief Full object graph: every live ID as vertex; edges join each figure to its point IDs.
     */
    [[nodiscard]] static ObjectGraph buildObjectGraph(const GeometryStorage& storage);

    /**
     * @brief Local neighbourhood around @p id: vertex @p id, adjacent figures/points, and edges among them.
     * @return std::nullopt if @p id is not present in @p storage.
     */
    [[nodiscard]] static std::optional<ObjectGraph> buildObjectSubgraph(const GeometryStorage& storage, Utils::ID id);
};

} // namespace OurPaintDCM::Figures

#endif
