#ifndef OURPAINTDCM_HEADERS_FIGURES_GEOMETRYDEPENDENCYINDEX_H
#define OURPAINTDCM_HEADERS_FIGURES_GEOMETRYDEPENDENCYINDEX_H

#include "ID.h"

#include <unordered_map>
#include <vector>

namespace OurPaintDCM::Figures {

/**
 * @brief Bidirectional adjacency for point↔figure incidence.
 *
 * - point → figures that reference that point (for getDependents-style queries).
 * - figure → point IDs it depends on (endpoints / center), fixed small arity.
 *
 * Updates are local to the arity k of the figure (O(k)); no full storage scan.
 */
class GeometryDependencyIndex {
public:
    /**
     * @brief Registers a line and links @p lineId to endpoints @p p1, @p p2.
     */
    void linkLine(Utils::ID lineId, Utils::ID p1, Utils::ID p2);

    /**
     * @brief Registers a circle; @p center is the only incident point ID in the index.
     */
    void linkCircle(Utils::ID circleId, Utils::ID center);

    /**
     * @brief Registers an arc through @p p1, @p p2 with center @p center.
     */
    void linkArc(Utils::ID arcId, Utils::ID p1, Utils::ID p2, Utils::ID center);

    /**
     * @brief Removes @p figureId from both directions: drops figure→points and
     *        removes the figure from each point’s incident list (cleans empty point entries).
     */
    void unlinkFigure(Utils::ID figureId);

    /**
     * @brief Copy of all figure IDs incident on @p pointId (lines/circles/arcs using this point).
     * @return Empty if the point has no incidents or is unknown.
     */
    [[nodiscard]] std::vector<Utils::ID> dependentsOfPoint(Utils::ID pointId) const;

    /**
     * @brief Const reference to the stored point-ID list for @p figureId (arity 1–3).
     * @return Empty static vector if @p figureId is unknown; do not store the reference
     *         beyond the index’s lifetime or across mutating calls on this index.
     */
    [[nodiscard]] const std::vector<Utils::ID>& pointsForFigure(Utils::ID figureId) const noexcept;

    /**
     * @brief True if @p pointId has at least one incident figure in the index.
     */
    [[nodiscard]] bool hasDependents(Utils::ID pointId) const noexcept;

    /**
     * @brief True if @p figureId has no figure→points entry (not linked or already unlinked).
     */
    [[nodiscard]] bool emptyFigure(Utils::ID figureId) const noexcept;

    /**
     * @brief Clears both maps.
     */
    void clear() noexcept;

private:
    /** @brief Appends @p id to @p vec if not already present (linear scan). */
    static void pushUnique(std::vector<Utils::ID>& vec, Utils::ID id);
    /** @brief Removes all occurrences of @p id from @p vec. */
    static void eraseId(std::vector<Utils::ID>& vec, Utils::ID id);

    std::unordered_map<Utils::ID, std::vector<Utils::ID>> pointToFigures_;
    std::unordered_map<Utils::ID, std::vector<Utils::ID>> figureToPoints_;
    static const std::vector<Utils::ID> kEmpty;
};

} // namespace OurPaintDCM::Figures

#endif
