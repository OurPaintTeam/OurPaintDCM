#include "RequirementFunctionFactory.h"

namespace OurPaintDCM::Function {

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createPointLineDist(
    Figures::Point2D* point,
    Figures::Line<Figures::Point2D>* line,
    double distance
) {
    std::vector<VAR> vars = {
        point->ptrX(), point->ptrY(),
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return bind(Utils::RequirementType::ET_POINTLINEDIST, Math::ConstraintKind::PointLineDistance, vars, distance);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createPointOnLine(
    Figures::Point2D* point,
    Figures::Line<Figures::Point2D>* line
) {
    std::vector<VAR> vars = {
        point->ptrX(), point->ptrY(),
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return bind(Utils::RequirementType::ET_POINTONLINE, Math::ConstraintKind::PointOnLine, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createPointPointDist(
    Figures::Point2D* p1,
    Figures::Point2D* p2,
    double distance
) {
    std::vector<VAR> vars = {
        p1->ptrX(), p1->ptrY(),
        p2->ptrX(), p2->ptrY()
    };
    return bind(Utils::RequirementType::ET_POINTPOINTDIST, Math::ConstraintKind::PointPointDistance, vars, distance);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createPointOnPoint(
    Figures::Point2D* p1,
    Figures::Point2D* p2
) {
    std::vector<VAR> vars = {
        p1->ptrX(), p1->ptrY(),
        p2->ptrX(), p2->ptrY()
    };
    return bind(Utils::RequirementType::ET_POINTONPOINT, Math::ConstraintKind::PointOnPoint, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createLineCircleDist(
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
    return bind(Utils::RequirementType::ET_LINECIRCLEDIST, Math::ConstraintKind::SegmentCircleDistance, vars, distance);
}

std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createLineOnCircle(
    Figures::Line<Figures::Point2D>* line,
    Figures::Circle<Figures::Point2D>* circle
) {
    std::vector<std::shared_ptr<RequirementFunction>> rows;
    for (auto* endpoint : {line->p1,line->p2}) {
        rows.push_back(bind(Utils::RequirementType::ET_LINEONCIRCLE, Math::ConstraintKind::PointOnCircle,
            {endpoint->ptrX(),endpoint->ptrY(),circle->center->ptrX(),circle->center->ptrY(),circle->ptrRadius()}));
    }
    return rows;
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createLineLineParallel(
    Figures::Line<Figures::Point2D>* l1,
    Figures::Line<Figures::Point2D>* l2
) {
    std::vector<VAR> vars = {
        l1->p1->ptrX(), l1->p1->ptrY(),
        l1->p2->ptrX(), l1->p2->ptrY(),
        l2->p1->ptrX(), l2->p1->ptrY(),
        l2->p2->ptrX(), l2->p2->ptrY()
    };
    return bind(Utils::RequirementType::ET_LINELINEPARALLEL, Math::ConstraintKind::Parallel, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createLineLinePerpendicular(
    Figures::Line<Figures::Point2D>* l1,
    Figures::Line<Figures::Point2D>* l2
) {
    std::vector<VAR> vars = {
        l1->p1->ptrX(), l1->p1->ptrY(),
        l1->p2->ptrX(), l1->p2->ptrY(),
        l2->p1->ptrX(), l2->p1->ptrY(),
        l2->p2->ptrX(), l2->p2->ptrY()
    };
    return bind(Utils::RequirementType::ET_LINELINEPERPENDICULAR, Math::ConstraintKind::Perpendicular, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createLineLineAngle(
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
    return bind(Utils::RequirementType::ET_LINELINEANGLE, Math::ConstraintKind::Angle, vars, angle);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createVertical(
    Figures::Line<Figures::Point2D>* line
) {
    std::vector<VAR> vars = {
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return bind(Utils::RequirementType::ET_VERTICAL, Math::ConstraintKind::Vertical, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createHorizontal(
    Figures::Line<Figures::Point2D>* line
) {
    std::vector<VAR> vars = {
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return bind(Utils::RequirementType::ET_HORIZONTAL, Math::ConstraintKind::Horizontal, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createArcCenterOnPerpendicular(
    Figures::Arc<Figures::Point2D>* arc
) {
    std::vector<VAR> vars = {
        arc->p1->ptrX(), arc->p1->ptrY(),
        arc->p2->ptrX(), arc->p2->ptrY(),
        arc->p_center->ptrX(), arc->p_center->ptrY()
    };
    return bind(Utils::RequirementType::ET_ARCCENTERONPERPENDICULAR, Math::ConstraintKind::ArcBisector, vars);
}
std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createFixPoint(
    Figures::Point2D* point
) {
    return {
        bind(Utils::RequirementType::ET_FIXPOINT, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{point->ptrX()}, point->x()),
        bind(Utils::RequirementType::ET_FIXPOINT, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{point->ptrY()}, point->y())
    };
}

std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createFixLine(
    Figures::Line<Figures::Point2D>* line
) {
    return {
        bind(Utils::RequirementType::ET_FIXLINE, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{line->p1->ptrX()}, line->p1->x()),
        bind(Utils::RequirementType::ET_FIXLINE, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{line->p1->ptrY()}, line->p1->y()),
        bind(Utils::RequirementType::ET_FIXLINE, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{line->p2->ptrX()}, line->p2->x()),
        bind(Utils::RequirementType::ET_FIXLINE, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{line->p2->ptrY()}, line->p2->y())
    };
}

std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createFixCircle(
    Figures::Circle<Figures::Point2D>* circle
) {
    return {
        bind(Utils::RequirementType::ET_FIXCIRCLE, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{circle->center->ptrX()}, circle->center->x()),
        bind(Utils::RequirementType::ET_FIXCIRCLE, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{circle->center->ptrY()}, circle->center->y()),
        bind(Utils::RequirementType::ET_FIXCIRCLE, Math::ConstraintKind::FixCoordinate, std::vector<VAR>{circle->ptrRadius()}, circle->radius)
    };
}

} // namespace OurPaintDCM::Function
