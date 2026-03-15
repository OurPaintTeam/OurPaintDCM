#include "RequirementFunctionFactory.h"

namespace OurPaintDCM::Function {

std::shared_ptr<PointLineDistanceFunction> RequirementFunctionFactory::createPointLineDist(
    Figures::Point2D* point,
    Figures::Line<Figures::Point2D>* line,
    double distance
) {
    std::vector<VAR> vars = {
        point->ptrX(), point->ptrY(),
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return std::make_shared<PointLineDistanceFunction>(vars, distance);
}

std::shared_ptr<PointOnLineFunction> RequirementFunctionFactory::createPointOnLine(
    Figures::Point2D* point,
    Figures::Line<Figures::Point2D>* line
) {
    std::vector<VAR> vars = {
        point->ptrX(), point->ptrY(),
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return std::make_shared<PointOnLineFunction>(vars);
}

std::shared_ptr<PointPointDistanceFunction> RequirementFunctionFactory::createPointPointDist(
    Figures::Point2D* p1,
    Figures::Point2D* p2,
    double distance
) {
    std::vector<VAR> vars = {
        p1->ptrX(), p1->ptrY(),
        p2->ptrX(), p2->ptrY()
    };
    return std::make_shared<PointPointDistanceFunction>(vars, distance);
}

std::shared_ptr<PointOnPointFunction> RequirementFunctionFactory::createPointOnPoint(
    Figures::Point2D* p1,
    Figures::Point2D* p2
) {
    std::vector<VAR> vars = {
        p1->ptrX(), p1->ptrY(),
        p2->ptrX(), p2->ptrY()
    };
    return std::make_shared<PointOnPointFunction>(vars);
}

std::shared_ptr<LineCircleDistanceFunction> RequirementFunctionFactory::createLineCircleDist(
    Figures::Line<Figures::Point2D>* line,
    Figures::Circle<Figures::Point2D>* circle,
    double distance
) {
    std::vector<VAR> vars = {
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY(),
        circle->center->ptrX(), circle->center->ptrY(),
        circle->ptrRadius()
    };
    return std::make_shared<LineCircleDistanceFunction>(vars, distance);
}

std::shared_ptr<LineOnCircleFunction> RequirementFunctionFactory::createLineOnCircle(
    Figures::Line<Figures::Point2D>* line,
    Figures::Circle<Figures::Point2D>* circle
) {
    std::vector<VAR> vars = {
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY(),
        circle->center->ptrX(), circle->center->ptrY(),
        circle->ptrRadius()
    };
    return std::make_shared<LineOnCircleFunction>(vars);
}

std::shared_ptr<LineLineParallelFunction> RequirementFunctionFactory::createLineLineParallel(
    Figures::Line<Figures::Point2D>* l1,
    Figures::Line<Figures::Point2D>* l2
) {
    std::vector<VAR> vars = {
        l1->p1->ptrX(), l1->p1->ptrY(),
        l1->p2->ptrX(), l1->p2->ptrY(),
        l2->p1->ptrX(), l2->p1->ptrY(),
        l2->p2->ptrX(), l2->p2->ptrY()
    };
    return std::make_shared<LineLineParallelFunction>(vars);
}

std::shared_ptr<LineLinePerpendicularFunction> RequirementFunctionFactory::createLineLinePerpendicular(
    Figures::Line<Figures::Point2D>* l1,
    Figures::Line<Figures::Point2D>* l2
) {
    std::vector<VAR> vars = {
        l1->p1->ptrX(), l1->p1->ptrY(),
        l1->p2->ptrX(), l1->p2->ptrY(),
        l2->p1->ptrX(), l2->p1->ptrY(),
        l2->p2->ptrX(), l2->p2->ptrY()
    };
    return std::make_shared<LineLinePerpendicularFunction>(vars);
}

std::shared_ptr<LineLineAngleFunction> RequirementFunctionFactory::createLineLineAngle(
    Figures::Line<Figures::Point2D>* l1,
    Figures::Line<Figures::Point2D>* l2,
    double angle
) {
    std::vector<VAR> vars = {
        l1->p1->ptrX(), l1->p1->ptrY(),
        l1->p2->ptrX(), l1->p2->ptrY(),
        l2->p1->ptrX(), l2->p1->ptrY(),
        l2->p2->ptrX(), l2->p2->ptrY()
    };
    return std::make_shared<LineLineAngleFunction>(vars, angle);
}

std::shared_ptr<VerticalFunction> RequirementFunctionFactory::createVertical(
    Figures::Line<Figures::Point2D>* line
) {
    std::vector<VAR> vars = {
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return std::make_shared<VerticalFunction>(vars);
}

std::shared_ptr<HorizontalFunction> RequirementFunctionFactory::createHorizontal(
    Figures::Line<Figures::Point2D>* line
) {
    std::vector<VAR> vars = {
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return std::make_shared<HorizontalFunction>(vars);
}

std::shared_ptr<ArcCenterOnPerpendicularFunction> RequirementFunctionFactory::createArcCenterOnPerpendicular(
    Figures::Arc<Figures::Point2D>* arc
) {
    std::vector<VAR> vars = {
        arc->p1->ptrX(), arc->p1->ptrY(),
        arc->p2->ptrX(), arc->p2->ptrY(),
        arc->p_center->ptrX(), arc->p_center->ptrY()
    };
    return std::make_shared<ArcCenterOnPerpendicularFunction>(vars);
}
std::vector<std::shared_ptr<FixCoordinateFunction>> RequirementFunctionFactory::createFixPoint(
    Figures::Point2D* point
) {
    return {
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXPOINT, std::vector<VAR>{point->ptrX()}, point->x()),
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXPOINT, std::vector<VAR>{point->ptrY()}, point->y())
    };
}

std::vector<std::shared_ptr<FixCoordinateFunction>> RequirementFunctionFactory::createFixLine(
    Figures::Line<Figures::Point2D>* line
) {
    return {
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXLINE, std::vector<VAR>{line->p1->ptrX()}, line->p1->x()),
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXLINE, std::vector<VAR>{line->p1->ptrY()}, line->p1->y()),
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXLINE, std::vector<VAR>{line->p2->ptrX()}, line->p2->x()),
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXLINE, std::vector<VAR>{line->p2->ptrY()}, line->p2->y())
    };
}

std::vector<std::shared_ptr<FixCoordinateFunction>> RequirementFunctionFactory::createFixCircle(
    Figures::Circle<Figures::Point2D>* circle
) {
    return {
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXCIRCLE, std::vector<VAR>{circle->center->ptrX()}, circle->center->x()),
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXCIRCLE, std::vector<VAR>{circle->center->ptrY()}, circle->center->y()),
        std::make_shared<FixCoordinateFunction>(
            Utils::RequirementType::ET_FIXCIRCLE, std::vector<VAR>{circle->ptrRadius()}, circle->radius)
    };
}

} // namespace OurPaintDCM::Function