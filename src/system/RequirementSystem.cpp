#include "system/RequirementSystem.h"

namespace OurPaintDCM::System {

RequirementSystem::RequirementSystem(Figures::GeometryStorage* storage)
    : _storage(storage) {}

void RequirementSystem::addPointLineDist(Utils::ID pointId, Utils::ID lineId, double dist) {
    auto* point = _storage->get<Figures::Point2D>(pointId);
    auto* line = _storage->get<Figures::Line2D>(lineId);
    auto func = Function::RequirementFunctionFactory::createPointLineDist(point, line, dist);
    addFunction(func);
}

void RequirementSystem::addPointOnLine(Utils::ID pointId, Utils::ID lineId) {
    auto* point = _storage->get<Figures::Point2D>(pointId);
    auto* line = _storage->get<Figures::Line2D>(lineId);
    auto func = Function::RequirementFunctionFactory::createPointOnLine(point, line);
    addFunction(func);
}

void RequirementSystem::addPointPointDist(Utils::ID p1Id, Utils::ID p2Id, double dist) {
    auto* p1 = _storage->get<Figures::Point2D>(p1Id);
    auto* p2 = _storage->get<Figures::Point2D>(p2Id);
    auto func = Function::RequirementFunctionFactory::createPointPointDist(p1, p2, dist);
    addFunction(func);
}

void RequirementSystem::addPointOnPoint(Utils::ID p1Id, Utils::ID p2Id) {
    auto* p1 = _storage->get<Figures::Point2D>(p1Id);
    auto* p2 = _storage->get<Figures::Point2D>(p2Id);
    auto func = Function::RequirementFunctionFactory::createPointOnPoint(p1, p2);
    addFunction(func);
}

void RequirementSystem::addLineCircleDist(Utils::ID lineId, Utils::ID circleId, double dist) {
    auto* line = _storage->get<Figures::Line2D>(lineId);
    auto* circle = _storage->get<Figures::Circle2D>(circleId);
    auto func = Function::RequirementFunctionFactory::createLineCircleDist(line, circle, dist);
    addFunction(func);
}

void RequirementSystem::addLineOnCircle(Utils::ID lineId, Utils::ID circleId) {
    auto* line = _storage->get<Figures::Line2D>(lineId);
    auto* circle = _storage->get<Figures::Circle2D>(circleId);
    auto func = Function::RequirementFunctionFactory::createLineOnCircle(line, circle);
    addFunction(func);
}

void RequirementSystem::addLineLineParallel(Utils::ID l1Id, Utils::ID l2Id) {
    auto* l1 = _storage->get<Figures::Line2D>(l1Id);
    auto* l2 = _storage->get<Figures::Line2D>(l2Id);
    auto func = Function::RequirementFunctionFactory::createLineLineParallel(l1, l2);
    addFunction(func);
}

void RequirementSystem::addLineLinePerpendicular(Utils::ID l1Id, Utils::ID l2Id) {
    auto* l1 = _storage->get<Figures::Line2D>(l1Id);
    auto* l2 = _storage->get<Figures::Line2D>(l2Id);
    auto func = Function::RequirementFunctionFactory::createLineLinePerpendicular(l1, l2);
    addFunction(func);
}

void RequirementSystem::addLineLineAngle(Utils::ID l1Id, Utils::ID l2Id, double angle) {
    auto* l1 = _storage->get<Figures::Line2D>(l1Id);
    auto* l2 = _storage->get<Figures::Line2D>(l2Id);
    auto func = Function::RequirementFunctionFactory::createLineLineAngle(l1, l2, angle);
    addFunction(func);
}

void RequirementSystem::addVertical(Utils::ID lineId) {
    auto* line = _storage->get<Figures::Line2D>(lineId);
    auto func = Function::RequirementFunctionFactory::createVertical(line);
    addFunction(func);
}

void RequirementSystem::addHorizontal(Utils::ID lineId) {
    auto* line = _storage->get<Figures::Line2D>(lineId);
    auto func = Function::RequirementFunctionFactory::createHorizontal(line);
    addFunction(func);
}

void RequirementSystem::addArcCenterOnPerpendicular(Utils::ID arcId) {
    auto* arc = _storage->get<Figures::Arc2D>(arcId);
    auto func = Function::RequirementFunctionFactory::createArcCenterOnPerpendicular(arc);
    addFunction(func);
}

} // namespace OurPaintDCM::System
