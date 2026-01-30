#ifndef OURPAINTDCM_HEADERS_SYSTEM_REQUIREMENTSYSTEM_H
#define OURPAINTDCM_HEADERS_SYSTEM_REQUIREMENTSYSTEM_H

#include "RequirementFunctionSystem.h"
#include "RequirementFunctionFactory.h"
#include "GeometryStorage.h"
#include "Enums.h"

namespace OurPaintDCM::System {

/**
 * @brief High-level requirement system bound to GeometryStorage.
 *
 * Adds geometric constraints by IDs and uses RequirementFunctionSystem
 * directly for evaluation and diagnostics.
 */
class RequirementSystem : public RequirementFunctionSystem {
    Figures::GeometryStorage* _storage;

public:
    /**
     * @brief Construct system bound to a GeometryStorage.
     * @param storage Pointer to the storage (must outlive this object).
     */
    explicit RequirementSystem(Figures::GeometryStorage* storage);

    /// @brief Add point-line distance constraint by IDs.
    void addPointLineDist(Utils::ID pointId, Utils::ID lineId, double dist);

    /// @brief Add point-on-line constraint by IDs.
    void addPointOnLine(Utils::ID pointId, Utils::ID lineId);

    /// @brief Add point-point distance constraint by IDs.
    void addPointPointDist(Utils::ID p1Id, Utils::ID p2Id, double dist);

    /// @brief Add point-on-point constraint by IDs.
    void addPointOnPoint(Utils::ID p1Id, Utils::ID p2Id);

    /// @brief Add line-circle distance constraint by IDs.
    void addLineCircleDist(Utils::ID lineId, Utils::ID circleId, double dist);

    /// @brief Add line-on-circle constraint by IDs.
    void addLineOnCircle(Utils::ID lineId, Utils::ID circleId);

    /// @brief Add parallel-lines constraint by IDs.
    void addLineLineParallel(Utils::ID l1Id, Utils::ID l2Id);

    /// @brief Add perpendicular-lines constraint by IDs.
    void addLineLinePerpendicular(Utils::ID l1Id, Utils::ID l2Id);

    /// @brief Add angle-between-lines constraint by IDs.
    void addLineLineAngle(Utils::ID l1Id, Utils::ID l2Id, double angle);

    /// @brief Add vertical-line constraint by ID.
    void addVertical(Utils::ID lineId);

    /// @brief Add horizontal-line constraint by ID.
    void addHorizontal(Utils::ID lineId);

    /// @brief Add arc-center perpendicular-bisector constraint by ID.
    void addArcCenterOnPerpendicular(Utils::ID arcId);
};

}

#endif
