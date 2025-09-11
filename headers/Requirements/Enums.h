#ifndef HEADERS_REQUIREMENTS_ENUM_H
#define HEADERS_REQUIREMENTS_ENUM_H
#include <cstdint>

namespace OurPaintDCM::Requirements {
/**
 * @brief Enum which illustrates every requirement type
 * Each value describes a geometric constraint that can be applied
 * to points, sections (lines), circles, or arcs in the system.
 * These constraints are used by solvers to maintain relationships
 * between geometric objects.
 */
enum class RequirementType: uint8_t {
    /// Fixed distance between point and section.
    ET_POINTSECTIONDIST,

    /// Point must lie on a section (distance == 0).
    ET_POINTONSECTION,

    /// Fixed distance between two points.
    ET_POINTPOINTDIST,

    /// Two points must coincide(distance == 0).
    ET_POINTONPOINT,

    /// Fixed distance between section and circle.
    ET_SECTIONCIRCLEDIST,

    /// Section endpoints must be on a circle.
    ET_SECTIONONCIRCLE,

    /// Section lies entirely inside a circle.
    ET_SECTIONINCIRCLE,

    /// Two sections must be parallel.
    ET_SECTIONSECTIONPARALLEL,

    /// Two sections must be perpendicular.
    ET_SECTIONSECTIONPERPENDICULAR,

    /// Fixed angle between two sections.
    ET_SECTIONSECTIONANGLE,

    /// Section must be vertical (along Y-axis).
    ET_VERTICAL,

    /// Section must be horizontal (along X-axis).
    ET_HORIZONTAL,

    /// Default constraints for every arc, for making 3 points working like a arc
    ET_ARCCENTERONPERPENDICULAR
};

/**
 * @brief Enum for all geometric figure types.
 */
enum class FigureType : uint8_t {
    /// 2D point object.
    ET_POINT2D,

    /// Line or line segment.
    ET_LINES,

    /// Circle object.
    ET_CIRCLE,

    /// Circular arc object.
    ET_ARC
};

}
#endif //HEADERS_REQUIREMENTS_ENUM_H