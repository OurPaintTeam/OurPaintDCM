#ifndef OURPAINTDCM_HEADERS_UTILS_FIGUREDESCRIPTOR_H
#define OURPAINTDCM_HEADERS_UTILS_FIGUREDESCRIPTOR_H

#include "Enums.h"
#include "ID.h"
#include <vector>
#include <optional>
#include <stdexcept>

namespace OurPaintDCM::Utils {

/**
 * @brief High-level descriptor for geometric figures.
 *
 * Provides a unified way to describe any geometric figure
 * for CRUD operations through a single interface.
 */
struct FigureDescriptor {
    std::optional<ID> id;
    FigureType type;
    std::vector<ID> pointIds;
    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> radius;

    FigureDescriptor() = default;

    /**
     * @brief Full constructor for FigureDescriptor.
     * @param t Figure type.
     * @param ids Point IDs (for Line, Circle, Arc).
     * @param xVal X coordinate (for Point).
     * @param yVal Y coordinate (for Point).
     * @param r Radius (for Circle).
     */
    FigureDescriptor(FigureType t,
                     std::vector<ID> ids = {},
                     std::optional<double> xVal = std::nullopt,
                     std::optional<double> yVal = std::nullopt,
                     std::optional<double> r = std::nullopt)
        : type(t), pointIds(std::move(ids)), x(xVal), y(yVal), radius(r) {}

    /**
     * @brief Create descriptor for a 2D point.
     * @param xVal X coordinate.
     * @param yVal Y coordinate.
     * @return FigureDescriptor for point creation.
     */
    static FigureDescriptor point(double xVal, double yVal) {
        return {FigureType::ET_POINT2D, {}, xVal, yVal};
    }

    /**
     * @brief Create descriptor for a line.
     * @param p1Id First point ID.
     * @param p2Id Second point ID.
     * @return FigureDescriptor for line creation.
     */
    static FigureDescriptor line(ID p1Id, ID p2Id) {
        return {FigureType::ET_LINE, {p1Id, p2Id}};
    }

    /**
     * @brief Create descriptor for a circle.
     * @param centerId Center point ID.
     * @param r Radius value.
     * @return FigureDescriptor for circle creation.
     */
    static FigureDescriptor circle(ID centerId, double r) {
        return {FigureType::ET_CIRCLE, {centerId}, std::nullopt, std::nullopt, r};
    }

    /**
     * @brief Create descriptor for an arc.
     * @param p1Id First endpoint ID.
     * @param p2Id Second endpoint ID.
     * @param centerId Center point ID.
     * @return FigureDescriptor for arc creation.
     */
    static FigureDescriptor arc(ID p1Id, ID p2Id, ID centerId) {
        return {FigureType::ET_ARC, {p1Id, p2Id, centerId}};
    }

    /**
     * @brief Validate descriptor fields for the given figure type.
     * @return true if valid.
     * @throws std::invalid_argument if validation fails.
     */
    bool validate() const {
        switch (type) {
            case FigureType::ET_POINT2D:
                if (!x.has_value() || !y.has_value()) {
                    throw std::invalid_argument("Point requires x and y coordinates");
                }
                break;
            case FigureType::ET_LINE:
                if (pointIds.size() != 2) {
                    throw std::invalid_argument("Line requires exactly 2 point IDs");
                }
                break;
            case FigureType::ET_CIRCLE:
                if (pointIds.size() != 1) {
                    throw std::invalid_argument("Circle requires exactly 1 center point ID");
                }
                if (!radius.has_value() || radius.value() <= 0) {
                    throw std::invalid_argument("Circle requires positive radius");
                }
                break;
            case FigureType::ET_ARC:
                if (pointIds.size() != 3) {
                    throw std::invalid_argument("Arc requires exactly 3 point IDs");
                }
                break;
        }
        return true;
    }
};

/**
 * @brief Descriptor for updating point coordinates.
 */
struct PointUpdateDescriptor {
    ID pointId;
    std::optional<double> newX;
    std::optional<double> newY;

    PointUpdateDescriptor(ID id, std::optional<double> x = std::nullopt, std::optional<double> y = std::nullopt)
        : pointId(id), newX(x), newY(y) {}
};

/**
 * @brief Descriptor for updating circle radius.
 */
struct CircleUpdateDescriptor {
    ID circleId;
    double newRadius;

    CircleUpdateDescriptor(ID id, double r) : circleId(id), newRadius(r) {}
};

} // namespace OurPaintDCM::Utils

#endif // OURPAINTDCM_HEADERS_UTILS_FIGUREDESCRIPTOR_H
