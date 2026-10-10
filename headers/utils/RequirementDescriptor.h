#ifndef OURPAINTDCM_HEADERS_UTILS_REQUIREMENTDESCRIPTOR_H
#define OURPAINTDCM_HEADERS_UTILS_REQUIREMENTDESCRIPTOR_H

#include "Enums.h"
#include "ID.h"
#include "NumericValidation.h"
#include <vector>
#include <optional>
#include <stdexcept>
#include <cmath>

namespace OurPaintDCM::Utils {

/**
 * @brief High-level descriptor for geometric requirements.
 *
 * Provides a unified way to describe any geometric constraint
 * using object IDs and an optional parameter value.
 * This allows adding requirements through a single interface
 * instead of multiple specialized methods.
 *
 * ## Usage Example
 * @code
 * // Distance between two points
 * RequirementDescriptor desc;
 * desc.type = RequirementType::ET_POINTPOINTDIST;
 * desc.objectIds = {pointId1, pointId2};
 * desc.param = 50.0;  // distance value
 * system.addRequirement(desc);
 *
 * // Or using the builder-style methods
 * auto desc = RequirementDescriptor::pointPointDist(p1, p2, 50.0);
 * system.addRequirement(desc);
 * @endcode
 */
struct RequirementDescriptor {
    std::optional<ID> id;              ///< ID of the requirement (set after creation)
    RequirementType type;              ///< Type of the requirement
    std::vector<ID> objectIds;         ///< IDs of objects involved
    std::optional<double> param;       ///< Optional parameter (distance, angle, etc.)
    double weight = 1.0;               ///< Finite non-negative residual multiplier; objective uses (weight * residual)^2.
                                       ///< Fixed and point-coincidence requirements use 1 (active) or 0 (disabled).
    std::optional<Endpoint> firstEndpoint;  ///< Selected end of objectIds[0] for arc tangency.
    std::optional<Endpoint> secondEndpoint; ///< Selected end of objectIds[1] for arc tangency.

    /// @brief Default constructor
    RequirementDescriptor() = default;

    /**
     * @brief Full constructor
     * @param t Requirement type
     * @param ids Vector of object IDs
     * @param p Optional parameter value
     * @param w Non-negative residual weight (default 1)
     */
    RequirementDescriptor(RequirementType t,
                          std::vector<ID> ids,
                          std::optional<double> p = std::nullopt,
                          double w = 1.0)
        : type(t), objectIds(std::move(ids)), param(p), weight(w) {}

    // ==================== Factory methods ====================

    /// @brief Create point-line distance descriptor
    static RequirementDescriptor pointLineDist(ID pointId, ID lineId, double dist) {
        return {RequirementType::ET_POINTLINEDIST, {pointId, lineId}, dist};
    }

    /// @brief Create point-on-line descriptor
    static RequirementDescriptor pointOnLine(ID pointId, ID lineId) {
        return {RequirementType::ET_POINTONLINE, {pointId, lineId}};
    }

    /// @brief Create point-point distance descriptor
    static RequirementDescriptor pointPointDist(ID p1Id, ID p2Id, double dist) {
        return {RequirementType::ET_POINTPOINTDIST, {p1Id, p2Id}, dist};
    }

    /// @brief Create point-on-point descriptor
    static RequirementDescriptor pointOnPoint(ID p1Id, ID p2Id) {
        return {RequirementType::ET_POINTONPOINT, {p1Id, p2Id}};
    }

    /// @brief Create line-circle distance descriptor
    static RequirementDescriptor lineCircleDist(ID lineId, ID circleId, double dist) {
        return {RequirementType::ET_LINECIRCLEDIST, {lineId, circleId}, dist};
    }

    /// @brief Create line-on-circle descriptor
    static RequirementDescriptor lineOnCircle(ID lineId, ID circleId) {
        return {RequirementType::ET_LINEONCIRCLE, {lineId, circleId}};
    }

    /// @brief Create line-in-circle descriptor
    static RequirementDescriptor lineInCircle(ID lineId, ID circleId) {
        return {RequirementType::ET_LINEINCIRCLE, {lineId, circleId}};
    }

    /// @brief Create parallel lines descriptor
    static RequirementDescriptor lineLineParallel(ID l1Id, ID l2Id) {
        return {RequirementType::ET_LINELINEPARALLEL, {l1Id, l2Id}};
    }

    /// @brief Create perpendicular lines descriptor
    static RequirementDescriptor lineLinePerpendicular(ID l1Id, ID l2Id) {
        return {RequirementType::ET_LINELINEPERPENDICULAR, {l1Id, l2Id}};
    }

    /// @brief Create angle between lines descriptor
    /// @param angle Angle in radians.
    static RequirementDescriptor lineLineAngle(ID l1Id, ID l2Id, double angle) {
        return {RequirementType::ET_LINELINEANGLE, {l1Id, l2Id}, angle};
    }

    /// @brief Create vertical line descriptor
    static RequirementDescriptor vertical(ID lineId) {
        return {RequirementType::ET_VERTICAL, {lineId}};
    }

    /// @brief Create horizontal line descriptor
    static RequirementDescriptor horizontal(ID lineId) {
        return {RequirementType::ET_HORIZONTAL, {lineId}};
    }

    /// @brief Create arc center on perpendicular bisector descriptor
    static RequirementDescriptor arcCenterOnPerpendicular(ID arcId) {
        return {RequirementType::ET_ARCCENTERONPERPENDICULAR, {arcId}};
    }

    /// @brief Create fix-point descriptor
    static RequirementDescriptor fixPoint(ID pointId) {
        return {RequirementType::ET_FIXPOINT, {pointId}};
    }

    /// @brief Create fix-line descriptor
    static RequirementDescriptor fixLine(ID lineId) {
        return {RequirementType::ET_FIXLINE, {lineId}};
    }

    /// @brief Create fix-circle descriptor
    static RequirementDescriptor fixCircle(ID circleId) {
        return {RequirementType::ET_FIXCIRCLE, {circleId}};
    }

    static RequirementDescriptor pointOnCircle(ID pointId, ID circleId) {
        return {RequirementType::ET_POINTONCIRCLE, {pointId, circleId}};
    }

    static RequirementDescriptor circleRadius(ID circleId, double radius) {
        return {RequirementType::ET_CIRCLERADIUS, {circleId}, radius};
    }

    /// The original diameter is stored; its residual is measured in radius units.
    static RequirementDescriptor circleDiameter(ID circleId, double diameter) {
        return {RequirementType::ET_CIRCLEDIAMETER, {circleId}, diameter};
    }

    static RequirementDescriptor equalLength(ID line1Id, ID line2Id) {
        return {RequirementType::ET_EQUALLENGTH, {line1Id, line2Id}};
    }

    static RequirementDescriptor equalRadius(ID circle1Id, ID circle2Id) {
        return {RequirementType::ET_EQUALRADIUS, {circle1Id, circle2Id}};
    }

    static RequirementDescriptor pointAtMidpoint(ID pointId, ID lineId) {
        return {RequirementType::ET_POINTATMIDPOINT, {pointId, lineId}};
    }

    static RequirementDescriptor symmetricAboutLine(ID p, ID q, ID axis) {
        return {RequirementType::ET_SYMMETRICABOUTLINE, {p, q, axis}};
    }
    static RequirementDescriptor symmetricAboutHorizontal(ID p, ID q, double y = 0) {
        return {RequirementType::ET_SYMMETRICABOUTHORIZONTAL, {p, q}, y};
    }
    static RequirementDescriptor symmetricAboutVertical(ID p, ID q, double x = 0) {
        return {RequirementType::ET_SYMMETRICABOUTVERTICAL, {p, q}, x};
    }
    static RequirementDescriptor symmetricAboutPoint(ID p, ID q, ID center) {
        return {RequirementType::ET_SYMMETRICABOUTPOINT, {p, q, center}};
    }

    /// Tangency to the infinite supporting line; side is preserved until explicitly edited.
    static RequirementDescriptor lineCircleTangent(ID line, ID circle, TangencySide side) {
        return {RequirementType::ET_LINECIRCLETANGENT, {line, circle}, static_cast<double>(side)};
    }

    static RequirementDescriptor circleCircleTangent(ID first, ID second, CircleTangencyKind kind) {
        return {RequirementType::ET_CIRCLECIRCLETANGENT, {first, second}, static_cast<double>(kind)};
    }

    /// Connect the selected arc and line endpoints and impose geometric tangency.
    static RequirementDescriptor arcLineTangent(ID arc, Endpoint arcEnd, ID line, Endpoint lineEnd) {
        RequirementDescriptor d{RequirementType::ET_ARCLINETANGENT,{arc,line}};
        d.firstEndpoint=arcEnd; d.secondEndpoint=lineEnd; return d;
    }
    /// Connect the selected endpoints without selecting a traversal direction.
    static RequirementDescriptor arcArcTangent(ID first, Endpoint firstEnd, ID second, Endpoint secondEnd) {
        RequirementDescriptor d{RequirementType::ET_ARCARCTANGENT,{first,second}};
        d.firstEndpoint=firstEnd; d.secondEndpoint=secondEnd; return d;
    }

    // ==================== Validation ====================

    /**
     * @brief Validate that descriptor has correct number of objects for its type.
     * @return true if valid
     * @throws std::invalid_argument with description of the problem
     */
    bool validate() const {
        const bool arcTangency=type == RequirementType::ET_ARCLINETANGENT || type == RequirementType::ET_ARCARCTANGENT;
        const auto validEnd=[](const std::optional<Endpoint>& e) {
            return e && (*e == Endpoint::FIRST || *e == Endpoint::SECOND);
        };
        if (arcTangency ? (!validEnd(firstEndpoint) || !validEnd(secondEndpoint))
                        : (firstEndpoint.has_value() || secondEndpoint.has_value()))
            throw std::invalid_argument("Endpoint selection is required only for arc tangency");
        if (!std::isfinite(weight) || weight < 0.0) {
            throw std::invalid_argument("Requirement weight must be finite and non-negative");
        }
        if (weight != 1.0 && weight != 0.0 &&
            (type == RequirementType::ET_POINTONPOINT ||
             type == RequirementType::ET_FIXPOINT ||
             type == RequirementType::ET_FIXLINE ||
             type == RequirementType::ET_FIXCIRCLE)) {
            throw std::invalid_argument("Eliminated and fixed requirements must use weight 0 or 1");
        }
        switch (type) {
            case RequirementType::ET_SYMMETRICABOUTLINE:
            case RequirementType::ET_SYMMETRICABOUTPOINT:
                if (objectIds.size() != 3)
                    throw std::invalid_argument("Symmetry requires exactly 3 object IDs");
                break;
            case RequirementType::ET_POINTLINEDIST:
            case RequirementType::ET_POINTONLINE:
            case RequirementType::ET_POINTPOINTDIST:
            case RequirementType::ET_POINTONPOINT:
            case RequirementType::ET_LINECIRCLEDIST:
            case RequirementType::ET_LINEONCIRCLE:
            case RequirementType::ET_LINEINCIRCLE:
            case RequirementType::ET_LINELINEPARALLEL:
            case RequirementType::ET_LINELINEPERPENDICULAR:
            case RequirementType::ET_LINELINEANGLE:
            case RequirementType::ET_POINTONCIRCLE:
            case RequirementType::ET_EQUALLENGTH:
            case RequirementType::ET_EQUALRADIUS:
            case RequirementType::ET_POINTATMIDPOINT:
            case RequirementType::ET_LINECIRCLETANGENT:
            case RequirementType::ET_CIRCLECIRCLETANGENT:
            case RequirementType::ET_ARCLINETANGENT:
            case RequirementType::ET_ARCARCTANGENT:
            case RequirementType::ET_SYMMETRICABOUTHORIZONTAL:
            case RequirementType::ET_SYMMETRICABOUTVERTICAL:
                if (objectIds.size() != 2) {
                    throw std::invalid_argument("Requirement type requires exactly 2 object IDs");
                }
                break;
            case RequirementType::ET_VERTICAL:
            case RequirementType::ET_HORIZONTAL:
            case RequirementType::ET_ARCCENTERONPERPENDICULAR:
            case RequirementType::ET_FIXPOINT:
            case RequirementType::ET_FIXLINE:
            case RequirementType::ET_FIXCIRCLE:
            case RequirementType::ET_CIRCLERADIUS:
            case RequirementType::ET_CIRCLEDIAMETER:
                if (objectIds.size() != 1) {
                    throw std::invalid_argument("Requirement type requires exactly 1 object ID");
                }
                break;
            default:
                throw std::invalid_argument("Unsupported requirement type");
        }
        validateRequirementParameter(type, param);
        return true;
    }
};

} // namespace OurPaintDCM::Utils

#endif // OURPAINTDCM_HEADERS_UTILS_REQUIREMENTDESCRIPTOR_H
