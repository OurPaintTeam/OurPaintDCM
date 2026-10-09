#ifndef OURPAINTDCM_FUNCTION_REQUIREMENTFUNCTIONFACTORY_H
#define OURPAINTDCM_FUNCTION_REQUIREMENTFUNCTIONFACTORY_H

#include "RequirementFunction.h"
#include "Point2D.h"
#include "Line.h"
#include "Circle.h"
#include "Arc.h"
#include <memory>
#include <vector>

namespace OurPaintDCM::Function {

/**
 * @brief Factory for creating RequirementFunction objects from geometric figures.
 *
 * Extracts double* pointers from figures and constructs the appropriate
 * constraint functions. All created functions reference the original
 * data in GeometryStorage, so changes are reflected automatically.
 */
class RequirementFunctionFactory {
public:
    template<class MathFunction>
    static std::shared_ptr<RequirementFunction> bind(
        Utils::RequirementType type,
        const std::vector<VAR>& variables, double target = 0.0) {
        return std::make_shared<RequirementFunction>(type,
            std::make_shared<MathFunction>(variables,target));
    }
    /// @brief Create point-line distance function.
    static std::shared_ptr<RequirementFunction> createPointLineDist(
        Figures::Point2D* point,
        Figures::Line<Figures::Point2D>* line,
        double distance
    );

    /// @brief Create point-on-line function.
    static std::shared_ptr<RequirementFunction> createPointOnLine(
        Figures::Point2D* point,
        Figures::Line<Figures::Point2D>* line
    );

    /// @brief Create point-point distance function.
    static std::shared_ptr<RequirementFunction> createPointPointDist(
        Figures::Point2D* p1,
        Figures::Point2D* p2,
        double distance
    );

    /// @brief Create point-on-point function.
    static std::shared_ptr<RequirementFunction> createPointOnPoint(
        Figures::Point2D* p1,
        Figures::Point2D* p2
    );

    /// @brief Create line-circle distance function.
    static std::shared_ptr<RequirementFunction> createLineCircleDist(
        Figures::Line<Figures::Point2D>* line,
        Figures::Circle<Figures::Point2D>* circle,
        double distance
    );

    /// @brief Create line-on-circle function.
    static std::vector<std::shared_ptr<RequirementFunction>> createLineOnCircle(
        Figures::Line<Figures::Point2D>* line,
        Figures::Circle<Figures::Point2D>* circle
    );

    static std::shared_ptr<RequirementFunction> createPointOnCircle(
        Figures::Point2D* point, Figures::Circle<Figures::Point2D>* circle);
    static std::shared_ptr<RequirementFunction> createCircleRadius(
        Figures::Circle<Figures::Point2D>* circle, double radius);
    static std::shared_ptr<RequirementFunction> createCircleDiameter(
        Figures::Circle<Figures::Point2D>* circle, double diameter);

    /// @brief Create parallel-lines function.
    static std::shared_ptr<RequirementFunction> createLineLineParallel(
        Figures::Line<Figures::Point2D>* l1,
        Figures::Line<Figures::Point2D>* l2
    );

    /// @brief Create perpendicular-lines function.
    static std::shared_ptr<RequirementFunction> createLineLinePerpendicular(
        Figures::Line<Figures::Point2D>* l1,
        Figures::Line<Figures::Point2D>* l2
    );

    /// @brief Create angle-between-lines function.
    static std::shared_ptr<RequirementFunction> createLineLineAngle(
        Figures::Line<Figures::Point2D>* l1,
        Figures::Line<Figures::Point2D>* l2,
        double angle
    );

    /// @brief Create vertical-line function.
    static std::shared_ptr<RequirementFunction> createVertical(
        Figures::Line<Figures::Point2D>* line
    );

    /// @brief Create horizontal-line function.
    static std::shared_ptr<RequirementFunction> createHorizontal(
        Figures::Line<Figures::Point2D>* line
    );

    /// @brief Create arc center perpendicular-bisector function.
    static std::shared_ptr<RequirementFunction> createArcCenterOnPerpendicular(
        Figures::Arc<Figures::Point2D>* arc
    );

    /// @brief Create fix-point functions (one per coordinate).
    static std::vector<std::shared_ptr<RequirementFunction>> createFixPoint(
        Figures::Point2D* point
    );

    /// @brief Create fix-line functions (one per endpoint coordinate).
    static std::vector<std::shared_ptr<RequirementFunction>> createFixLine(
        Figures::Line<Figures::Point2D>* line
    );

    /// @brief Create fix-circle functions (one per center coordinate + radius).
    static std::vector<std::shared_ptr<RequirementFunction>> createFixCircle(
        Figures::Circle<Figures::Point2D>* circle
    );
};

}

#endif
