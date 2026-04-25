#ifndef OURPAINTDCM_HEADERS_UTILS_FIGUREDESCRIPTOR_H
#define OURPAINTDCM_HEADERS_UTILS_FIGUREDESCRIPTOR_H

#include "Enums.h"
#include "ID.h"
#include <vector>
#include <optional>
#include <stdexcept>
#include <utility>

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
    std::vector<double> coords;
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
        FigureDescriptor d{FigureType::ET_POINT2D, {}, xVal, yVal};
        d.coords = {xVal, yVal};
        return d;
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
     * @brief Create descriptor for a line by coordinates.
     * @param x1 First point X.
     * @param y1 First point Y.
     * @param x2 Second point X.
     * @param y2 Second point Y.
     * @return FigureDescriptor for line creation.
     */
    static FigureDescriptor line(double x1, double y1, double x2, double y2) {
        FigureDescriptor d{FigureType::ET_LINE};
        d.coords = {x1, y1, x2, y2};
        return d;
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
     * @brief Create descriptor for a circle by center coordinates.
     * @param cx Center X.
     * @param cy Center Y.
     * @param r Radius value.
     * @return FigureDescriptor for circle creation.
     */
    static FigureDescriptor circle(double cx, double cy, double r) {
        FigureDescriptor d{FigureType::ET_CIRCLE, {}, std::nullopt, std::nullopt, r};
        d.coords = {cx, cy};
        return d;
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
     * @brief Create descriptor for an arc by coordinates.
     * @param x1 First endpoint X.
     * @param y1 First endpoint Y.
     * @param x2 Second endpoint X.
     * @param y2 Second endpoint Y.
     * @param cx Center X.
     * @param cy Center Y.
     * @return FigureDescriptor for arc creation.
     */
    static FigureDescriptor arc(double x1, double y1, double x2, double y2, double cx, double cy) {
        FigureDescriptor d{FigureType::ET_ARC};
        d.coords = {x1, y1, x2, y2, cx, cy};
        return d;
    }

    /**
     * @brief Validate descriptor fields for the given figure type.
     * @return true if valid.
     * @throws std::invalid_argument if validation fails.
     */
    bool validate() const {
        switch (type) {
            case FigureType::ET_POINT2D:
                if (coords.size() == 2) {
                    break;
                }
                if (!x.has_value() || !y.has_value()) {
                    throw std::invalid_argument("Point requires x and y coordinates");
                }
                break;
            case FigureType::ET_LINE:
                if (coords.size() == 4) {
                    break;
                }
                if (pointIds.size() != 2) {
                    throw std::invalid_argument("Line requires coordinates or exactly 2 point IDs");
                }
                break;
            case FigureType::ET_CIRCLE:
                if (coords.size() != 2 && pointIds.size() != 1) {
                    throw std::invalid_argument("Circle requires center coordinates or exactly 1 center point ID");
                }
                if (!radius.has_value() || radius.value() <= 0) {
                    throw std::invalid_argument("Circle requires positive radius");
                }
                break;
            case FigureType::ET_ARC:
                if (coords.size() == 6) {
                    break;
                }
                if (pointIds.size() != 3) {
                    throw std::invalid_argument("Arc requires coordinates or exactly 3 point IDs");
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
    std::optional<double> newCenterX;
    std::optional<double> newCenterY;
    bool updateRadius;

    CircleUpdateDescriptor(ID id, double r)
        : circleId(id), newRadius(r), updateRadius(true) {}

    CircleUpdateDescriptor(ID id,
                           std::optional<double> centerX,
                           std::optional<double> centerY,
                           std::optional<double> radius = std::nullopt)
        : circleId(id),
          newRadius(radius.value_or(0.0)),
          newCenterX(centerX),
          newCenterY(centerY),
          updateRadius(radius.has_value()) {}

    CircleUpdateDescriptor(ID id, double centerX, double centerY, double radius)
        : circleId(id),
          newRadius(radius),
          newCenterX(centerX),
          newCenterY(centerY),
          updateRadius(true) {}

    static CircleUpdateDescriptor center(ID id, double centerX, double centerY) {
        return {id, std::optional<double>{centerX}, std::optional<double>{centerY}, std::nullopt};
    }

    [[nodiscard]] bool hasCenterUpdate() const noexcept {
        return newCenterX.has_value() || newCenterY.has_value();
    }

    [[nodiscard]] bool hasRadiusUpdate() const noexcept {
        return updateRadius;
    }
};

/**
 * @brief Descriptor for updating line endpoint coordinates.
 */
struct LineUpdateDescriptor {
    ID lineId;
    std::optional<double> newX1;
    std::optional<double> newY1;
    std::optional<double> newX2;
    std::optional<double> newY2;

    LineUpdateDescriptor(ID id,
                         std::optional<double> x1 = std::nullopt,
                         std::optional<double> y1 = std::nullopt,
                         std::optional<double> x2 = std::nullopt,
                         std::optional<double> y2 = std::nullopt)
        : lineId(id), newX1(x1), newY1(y1), newX2(x2), newY2(y2) {}
};

/**
 * @brief Descriptor for updating arc endpoint and center coordinates.
 */
struct ArcUpdateDescriptor {
    ID arcId;
    std::optional<double> newX1;
    std::optional<double> newY1;
    std::optional<double> newX2;
    std::optional<double> newY2;
    std::optional<double> newCenterX;
    std::optional<double> newCenterY;

    ArcUpdateDescriptor(ID id,
                        std::optional<double> x1 = std::nullopt,
                        std::optional<double> y1 = std::nullopt,
                        std::optional<double> x2 = std::nullopt,
                        std::optional<double> y2 = std::nullopt,
                        std::optional<double> centerX = std::nullopt,
                        std::optional<double> centerY = std::nullopt)
        : arcId(id),
          newX1(x1),
          newY1(y1),
          newX2(x2),
          newY2(y2),
          newCenterX(centerX),
          newCenterY(centerY) {}
};

/**
 * @brief Unified descriptor for updating any supported figure.
 */
struct FigureUpdateDescriptor {
    ID figureId;
    FigureType type;
    std::vector<double> coords;
    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> radius;

    FigureUpdateDescriptor(ID id, FigureType figureType)
        : figureId(id), type(figureType) {}

    static FigureUpdateDescriptor point(ID id, double xVal, double yVal) {
        FigureUpdateDescriptor d{id, FigureType::ET_POINT2D};
        d.coords = {xVal, yVal};
        d.x = xVal;
        d.y = yVal;
        return d;
    }

    static FigureUpdateDescriptor line(ID id, double x1, double y1, double x2, double y2) {
        FigureUpdateDescriptor d{id, FigureType::ET_LINE};
        d.coords = {x1, y1, x2, y2};
        return d;
    }

    static FigureUpdateDescriptor circle(ID id, double centerX, double centerY, double r) {
        FigureUpdateDescriptor d{id, FigureType::ET_CIRCLE};
        d.coords = {centerX, centerY};
        d.radius = r;
        return d;
    }

    static FigureUpdateDescriptor circleCenter(ID id, double centerX, double centerY) {
        FigureUpdateDescriptor d{id, FigureType::ET_CIRCLE};
        d.coords = {centerX, centerY};
        return d;
    }

    static FigureUpdateDescriptor circleRadius(ID id, double r) {
        FigureUpdateDescriptor d{id, FigureType::ET_CIRCLE};
        d.radius = r;
        return d;
    }

    static FigureUpdateDescriptor arc(ID id,
                                      double x1,
                                      double y1,
                                      double x2,
                                      double y2,
                                      double centerX,
                                      double centerY) {
        FigureUpdateDescriptor d{id, FigureType::ET_ARC};
        d.coords = {x1, y1, x2, y2, centerX, centerY};
        return d;
    }
};

} // namespace OurPaintDCM::Utils

#endif // OURPAINTDCM_HEADERS_UTILS_FIGUREDESCRIPTOR_H
