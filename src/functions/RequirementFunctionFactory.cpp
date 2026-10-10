#include "RequirementFunctionFactory.h"
#include "NumericValidation.h"

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
    return bind<PointSectionDistanceError>(Utils::RequirementType::ET_POINTLINEDIST, vars, distance);
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
    return bind<PointOnSectionError>(Utils::RequirementType::ET_POINTONLINE, vars);
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
    return bind<PointPointDistanceError>(Utils::RequirementType::ET_POINTPOINTDIST, vars, distance);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createPointOnPoint(
    Figures::Point2D* p1,
    Figures::Point2D* p2
) {
    std::vector<VAR> vars = {
        p1->ptrX(), p1->ptrY(),
        p2->ptrX(), p2->ptrY()
    };
    return bind<PointOnPointError>(Utils::RequirementType::ET_POINTONPOINT, vars);
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
    return bind<SectionCircleDistanceError>(Utils::RequirementType::ET_LINECIRCLEDIST, vars, distance);
}

std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createLineOnCircle(
    Figures::Line<Figures::Point2D>* line,
    Figures::Circle<Figures::Point2D>* circle
) {
    std::vector<std::shared_ptr<RequirementFunction>> rows;
    for (auto* endpoint : {line->p1,line->p2}) {
        rows.push_back(bind<PointOnCircleError>(Utils::RequirementType::ET_LINEONCIRCLE,
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
    return bind<SectionSectionParallelError>(Utils::RequirementType::ET_LINELINEPARALLEL, vars);
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
    return bind<SectionSectionPerpendicularError>(Utils::RequirementType::ET_LINELINEPERPENDICULAR, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createLineCircleTangent(
    Figures::Line<Figures::Point2D>* line, Figures::Circle<Figures::Point2D>* circle, Utils::TangencySide side) {
    return bind<LineCircleTangentError>(Utils::RequirementType::ET_LINECIRCLETANGENT,
        {line->p1->ptrX(),line->p1->ptrY(),line->p2->ptrX(),line->p2->ptrY(),
         circle->center->ptrX(),circle->center->ptrY(),circle->ptrRadius()},static_cast<double>(side));
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createPointOnCircle(
    Figures::Point2D* point, Figures::Circle<Figures::Point2D>* circle) {
    return bind<PointOnCircleError>(Utils::RequirementType::ET_POINTONCIRCLE,
        {point->ptrX(), point->ptrY(), circle->center->ptrX(), circle->center->ptrY(), circle->ptrRadius()});
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createCircleRadius(
    Figures::Circle<Figures::Point2D>* circle, double radius) {
    return bind<CircleRadiusError>(Utils::RequirementType::ET_CIRCLERADIUS, {circle->ptrRadius()}, radius);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createCircleDiameter(
    Figures::Circle<Figures::Point2D>* circle, double diameter) {
    Utils::requirePositiveRadius(diameter);
    return bind<CircleRadiusError>(Utils::RequirementType::ET_CIRCLEDIAMETER, {circle->ptrRadius()}, diameter / 2.0);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createEqualLength(
    Figures::Line<Figures::Point2D>* first, Figures::Line<Figures::Point2D>* second) {
    return bind<EqualLengthError>(Utils::RequirementType::ET_EQUALLENGTH,
        {first->p1->ptrX(), first->p1->ptrY(), first->p2->ptrX(), first->p2->ptrY(),
         second->p1->ptrX(), second->p1->ptrY(), second->p2->ptrX(), second->p2->ptrY()});
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createEqualRadius(
    Figures::Circle<Figures::Point2D>* first, Figures::Circle<Figures::Point2D>* second) {
    return bind<EqualRadiusError>(Utils::RequirementType::ET_EQUALRADIUS, {first->ptrRadius(), second->ptrRadius()});
}

std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createPointAtMidpoint(
    Figures::Point2D* point, Figures::Line<Figures::Point2D>* line) {
    return {
        bind<MidpointCoordinateError>(Utils::RequirementType::ET_POINTATMIDPOINT,
            {point->ptrX(), line->p1->ptrX(), line->p2->ptrX()}),
        bind<MidpointCoordinateError>(Utils::RequirementType::ET_POINTATMIDPOINT,
            {point->ptrY(), line->p1->ptrY(), line->p2->ptrY()})
    };
}

std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createSymmetricAboutLine(
    Figures::Point2D* p, Figures::Point2D* q, Figures::Line<Figures::Point2D>* axis) {
    const std::vector<VAR> vars{p->ptrX(),p->ptrY(),q->ptrX(),q->ptrY(),
        axis->p1->ptrX(),axis->p1->ptrY(),axis->p2->ptrX(),axis->p2->ptrY()};
    return {bind<SymmetryAlongError>(Utils::RequirementType::ET_SYMMETRICABOUTLINE,vars),
            bind<SymmetryAcrossError>(Utils::RequirementType::ET_SYMMETRICABOUTLINE,vars)};
}
std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createSymmetricAboutHorizontal(
    Figures::Point2D* p, Figures::Point2D* q, double y) {
    const auto type=Utils::RequirementType::ET_SYMMETRICABOUTHORIZONTAL;
    return {bind<CoordinateDifferenceError>(type,{p->ptrX(),q->ptrX()}),
            bind<CoordinateAverageError>(type,{p->ptrY(),q->ptrY()},y)};
}
std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createSymmetricAboutVertical(
    Figures::Point2D* p, Figures::Point2D* q, double x) {
    const auto type=Utils::RequirementType::ET_SYMMETRICABOUTVERTICAL;
    return {bind<CoordinateDifferenceError>(type,{p->ptrY(),q->ptrY()}),
            bind<CoordinateAverageError>(type,{p->ptrX(),q->ptrX()},x)};
}
std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createSymmetricAboutPoint(
    Figures::Point2D* p, Figures::Point2D* q, Figures::Point2D* c) {
    const auto type=Utils::RequirementType::ET_SYMMETRICABOUTPOINT;
    return {bind<MidpointCoordinateError>(type,{c->ptrX(),p->ptrX(),q->ptrX()}),
            bind<MidpointCoordinateError>(type,{c->ptrY(),p->ptrY(),q->ptrY()})};
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
    return bind<SectionSectionAngleError>(Utils::RequirementType::ET_LINELINEANGLE, vars, angle);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createVertical(
    Figures::Line<Figures::Point2D>* line
) {
    std::vector<VAR> vars = {
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return bind<VerticalError>(Utils::RequirementType::ET_VERTICAL, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createHorizontal(
    Figures::Line<Figures::Point2D>* line
) {
    std::vector<VAR> vars = {
        line->p1->ptrX(), line->p1->ptrY(),
        line->p2->ptrX(), line->p2->ptrY()
    };
    return bind<HorizontalError>(Utils::RequirementType::ET_HORIZONTAL, vars);
}

std::shared_ptr<RequirementFunction> RequirementFunctionFactory::createArcCenterOnPerpendicular(
    Figures::Arc<Figures::Point2D>* arc
) {
    std::vector<VAR> vars = {
        arc->p1->ptrX(), arc->p1->ptrY(),
        arc->p2->ptrX(), arc->p2->ptrY(),
        arc->p_center->ptrX(), arc->p_center->ptrY()
    };
    return bind<ArcCenterOnPerpendicularError>(Utils::RequirementType::ET_ARCCENTERONPERPENDICULAR, vars);
}
std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createFixPoint(
    Figures::Point2D* point
) {
    return {
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXPOINT, std::vector<VAR>{point->ptrX()}, point->x()),
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXPOINT, std::vector<VAR>{point->ptrY()}, point->y())
    };
}

std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createFixLine(
    Figures::Line<Figures::Point2D>* line
) {
    return {
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXLINE, std::vector<VAR>{line->p1->ptrX()}, line->p1->x()),
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXLINE, std::vector<VAR>{line->p1->ptrY()}, line->p1->y()),
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXLINE, std::vector<VAR>{line->p2->ptrX()}, line->p2->x()),
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXLINE, std::vector<VAR>{line->p2->ptrY()}, line->p2->y())
    };
}

std::vector<std::shared_ptr<RequirementFunction>> RequirementFunctionFactory::createFixCircle(
    Figures::Circle<Figures::Point2D>* circle
) {
    return {
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXCIRCLE, std::vector<VAR>{circle->center->ptrX()}, circle->center->x()),
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXCIRCLE, std::vector<VAR>{circle->center->ptrY()}, circle->center->y()),
        bind<FixCoordinateError>(Utils::RequirementType::ET_FIXCIRCLE, std::vector<VAR>{circle->ptrRadius()}, circle->radius)
    };
}

} // namespace OurPaintDCM::Function
