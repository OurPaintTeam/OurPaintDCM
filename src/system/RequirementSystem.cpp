#include "RequirementSystem.h"

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace {

template <typename T>
T* requireGeometry(T* ptr) {
    if (ptr == nullptr) {
        throw std::runtime_error("Invalid geometry ID or type mismatch");
    }
    return ptr;
}

std::vector<VAR> makePointVars(OurPaintDCM::Figures::Point2D* point) {
    return {point->ptrX(), point->ptrY()};
}

std::vector<VAR> makeTwoPointVars(OurPaintDCM::Figures::Point2D* p1,
                                  OurPaintDCM::Figures::Point2D* p2) {
    return {p1->ptrX(), p1->ptrY(), p2->ptrX(), p2->ptrY()};
}

std::vector<VAR> makePointLineVars(OurPaintDCM::Figures::Point2D* point,
                                   OurPaintDCM::Figures::Point2D* l1,
                                   OurPaintDCM::Figures::Point2D* l2) {
    return {point->ptrX(), point->ptrY(), l1->ptrX(), l1->ptrY(), l2->ptrX(), l2->ptrY()};
}

std::vector<VAR> makeLineLineVars(OurPaintDCM::Figures::Point2D* l1p1,
                                  OurPaintDCM::Figures::Point2D* l1p2,
                                  OurPaintDCM::Figures::Point2D* l2p1,
                                  OurPaintDCM::Figures::Point2D* l2p2) {
    return {
        l1p1->ptrX(), l1p1->ptrY(), l1p2->ptrX(), l1p2->ptrY(),
        l2p1->ptrX(), l2p1->ptrY(), l2p2->ptrX(), l2p2->ptrY()};
}

std::vector<VAR> makeLineCircleVars(OurPaintDCM::Figures::Point2D* l1,
                                    OurPaintDCM::Figures::Point2D* l2,
                                    OurPaintDCM::Figures::Point2D* center,
                                    double* radius) {
    return {
        l1->ptrX(), l1->ptrY(), l2->ptrX(), l2->ptrY(),
        center->ptrX(), center->ptrY(), radius};
}

} // namespace

namespace OurPaintDCM::System {

RequirementSystem::RequirementSystem(Figures::GeometryStorage* storage)
    : _storage(storage) {}

Utils::ID RequirementSystem::addRequirement(const Utils::RequirementDescriptor& descriptor) {
    descriptor.validate();

    const Utils::ID previousGeneratorState = _reqIdGen.current();

    Utils::ID reqId;
    if (descriptor.id.has_value()) {
        reqId = *descriptor.id;
        const unsigned long long nextMin = reqId.id + 1ULL;
        if (nextMin > _reqIdGen.current().id) {
            _reqIdGen.set(Utils::ID(nextMin));
        }
    } else {
        reqId = _reqIdGen.nextID();
    }

    _requirements.push_back({reqId, descriptor.type, descriptor.objectIds, descriptor.param});

    try {
        rebuildFunctionsAndAliases();
    } catch (...) {
        _requirements.pop_back();
        _reqIdGen.set(previousGeneratorState);
        rebuildFunctionsAndAliases();
        throw;
    }

    return reqId;
}

void RequirementSystem::rebuildFunctionsAndAliases() {
    RequirementFunctionSystem::clear();
    _pointRepresentative.clear();
    _coincidentPointGroups.clear();

    if (_storage == nullptr) {
        return;
    }

    std::unordered_map<Utils::ID, Utils::ID> parent;

    const std::function<Utils::ID(Utils::ID)> findRoot = [&](Utils::ID pointId) -> Utils::ID {
        auto it = parent.find(pointId);
        if (it == parent.end() || it->second == pointId) {
            return pointId;
        }
        const Utils::ID root = findRoot(it->second);
        it->second = root;
        return root;
    };

    const auto unite = [&](Utils::ID first, Utils::ID second) {
        parent.try_emplace(first, first);
        parent.try_emplace(second, second);

        const Utils::ID firstRoot = findRoot(first);
        const Utils::ID secondRoot = findRoot(second);
        if (firstRoot == secondRoot) {
            return;
        }

        if (firstRoot < secondRoot) {
            parent[firstRoot] = secondRoot;
        } else {
            parent[secondRoot] = firstRoot;
        }
    };

    for (const auto& entry : _requirements) {
        if (entry.type != Utils::RequirementType::ET_POINTONPOINT) {
            continue;
        }

        requireGeometry(_storage->get<Figures::Point2D>(entry.objectIds[0]));
        requireGeometry(_storage->get<Figures::Point2D>(entry.objectIds[1]));
        unite(entry.objectIds[0], entry.objectIds[1]);
    }

    for (const auto& [pointId, _] : parent) {
        const Utils::ID representative = findRoot(pointId);
        _pointRepresentative[pointId] = representative;
        _coincidentPointGroups[representative].push_back(pointId);
    }

    for (auto& [_, group] : _coincidentPointGroups) {
        std::sort(group.begin(), group.end(), [](Utils::ID lhs, Utils::ID rhs) {
            return lhs.id < rhs.id;
        });
    }

    const auto resolveLinePoints = [&](Utils::ID lineId) {
        requireGeometry(_storage->get<Figures::Line2D>(lineId));
        const auto dependencies = _storage->getDependencies(lineId);
        if (dependencies.size() != 2) {
            throw std::runtime_error("Line dependencies are inconsistent");
        }
        return std::pair{resolvePoint(dependencies[0]), resolvePoint(dependencies[1])};
    };

    const auto resolveCircleData = [&](Utils::ID circleId) {
        auto* circle = requireGeometry(_storage->get<Figures::Circle2D>(circleId));
        const auto dependencies = _storage->getDependencies(circleId);
        if (dependencies.size() != 1) {
            throw std::runtime_error("Circle dependencies are inconsistent");
        }
        return std::pair{resolvePoint(dependencies[0]), circle->ptrRadius()};
    };

    const auto resolveArcPoints = [&](Utils::ID arcId) {
        requireGeometry(_storage->get<Figures::Arc2D>(arcId));
        const auto dependencies = _storage->getDependencies(arcId);
        if (dependencies.size() != 3) {
            throw std::runtime_error("Arc dependencies are inconsistent");
        }
        return std::tuple{
            resolvePoint(dependencies[0]),
            resolvePoint(dependencies[1]),
            resolvePoint(dependencies[2])};
    };

    for (const auto& entry : _requirements) {
        const auto& ids = entry.objectIds;

        switch (entry.type) {
            case Utils::RequirementType::ET_POINTLINEDIST: {
                auto* point = resolvePoint(ids[0]);
                const auto [lineP1, lineP2] = resolveLinePoints(ids[1]);
                addFunction(std::make_shared<Function::PointLineDistanceFunction>(
                    makePointLineVars(point, lineP1, lineP2),
                    entry.param.value()));
                break;
            }
            case Utils::RequirementType::ET_POINTONLINE: {
                auto* point = resolvePoint(ids[0]);
                const auto [lineP1, lineP2] = resolveLinePoints(ids[1]);
                addFunction(std::make_shared<Function::PointOnLineFunction>(
                    makePointLineVars(point, lineP1, lineP2)));
                break;
            }
            case Utils::RequirementType::ET_POINTPOINTDIST: {
                auto* p1 = resolvePoint(ids[0]);
                auto* p2 = resolvePoint(ids[1]);
                addFunction(std::make_shared<Function::PointPointDistanceFunction>(
                    makeTwoPointVars(p1, p2),
                    entry.param.value()));
                break;
            }
            case Utils::RequirementType::ET_POINTONPOINT:
                break;
            case Utils::RequirementType::ET_LINECIRCLEDIST: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                const auto [center, radius] = resolveCircleData(ids[1]);
                addFunction(std::make_shared<Function::LineCircleDistanceFunction>(
                    makeLineCircleVars(lineP1, lineP2, center, radius),
                    entry.param.value()));
                break;
            }
            case Utils::RequirementType::ET_LINEONCIRCLE: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                const auto [center, radius] = resolveCircleData(ids[1]);
                addFunction(std::make_shared<Function::LineOnCircleFunction>(
                    makeLineCircleVars(lineP1, lineP2, center, radius)));
                break;
            }
            case Utils::RequirementType::ET_LINEINCIRCLE:
                throw std::runtime_error("LineInCircle requirement is not yet supported via unified interface");
            case Utils::RequirementType::ET_LINELINEPARALLEL: {
                const auto [l1p1, l1p2] = resolveLinePoints(ids[0]);
                const auto [l2p1, l2p2] = resolveLinePoints(ids[1]);
                addFunction(std::make_shared<Function::LineLineParallelFunction>(
                    makeLineLineVars(l1p1, l1p2, l2p1, l2p2)));
                break;
            }
            case Utils::RequirementType::ET_LINELINEPERPENDICULAR: {
                const auto [l1p1, l1p2] = resolveLinePoints(ids[0]);
                const auto [l2p1, l2p2] = resolveLinePoints(ids[1]);
                addFunction(std::make_shared<Function::LineLinePerpendicularFunction>(
                    makeLineLineVars(l1p1, l1p2, l2p1, l2p2)));
                break;
            }
            case Utils::RequirementType::ET_LINELINEANGLE: {
                const auto [l1p1, l1p2] = resolveLinePoints(ids[0]);
                const auto [l2p1, l2p2] = resolveLinePoints(ids[1]);
                addFunction(std::make_shared<Function::LineLineAngleFunction>(
                    makeLineLineVars(l1p1, l1p2, l2p1, l2p2),
                    entry.param.value()));
                break;
            }
            case Utils::RequirementType::ET_VERTICAL: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                addFunction(std::make_shared<Function::VerticalFunction>(
                    makeTwoPointVars(lineP1, lineP2)));
                break;
            }
            case Utils::RequirementType::ET_HORIZONTAL: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                addFunction(std::make_shared<Function::HorizontalFunction>(
                    makeTwoPointVars(lineP1, lineP2)));
                break;
            }
            case Utils::RequirementType::ET_ARCCENTERONPERPENDICULAR: {
                const auto [arcP1, arcP2, center] = resolveArcPoints(ids[0]);
                addFunction(std::make_shared<Function::ArcCenterOnPerpendicularFunction>(
                    std::vector<VAR>{
                        arcP1->ptrX(), arcP1->ptrY(),
                        arcP2->ptrX(), arcP2->ptrY(),
                        center->ptrX(), center->ptrY()}));
                break;
            }
            case Utils::RequirementType::ET_FIXPOINT: {
                auto* originalPoint = requireGeometry(_storage->get<Figures::Point2D>(ids[0]));
                auto* point = resolvePoint(ids[0]);
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXPOINT,
                    std::vector<VAR>{point->ptrX()},
                    originalPoint->x()));
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXPOINT,
                    std::vector<VAR>{point->ptrY()},
                    originalPoint->y()));
                break;
            }
            case Utils::RequirementType::ET_FIXLINE: {
                requireGeometry(_storage->get<Figures::Line2D>(ids[0]));
                const auto dependencies = _storage->getDependencies(ids[0]);
                if (dependencies.size() != 2) {
                    throw std::runtime_error("Line dependencies are inconsistent");
                }
                auto* originalP1 = requireGeometry(_storage->get<Figures::Point2D>(dependencies[0]));
                auto* originalP2 = requireGeometry(_storage->get<Figures::Point2D>(dependencies[1]));
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXLINE,
                    std::vector<VAR>{lineP1->ptrX()},
                    originalP1->x()));
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXLINE,
                    std::vector<VAR>{lineP1->ptrY()},
                    originalP1->y()));
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXLINE,
                    std::vector<VAR>{lineP2->ptrX()},
                    originalP2->x()));
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXLINE,
                    std::vector<VAR>{lineP2->ptrY()},
                    originalP2->y()));
                break;
            }
            case Utils::RequirementType::ET_FIXCIRCLE: {
                const auto [center, radius] = resolveCircleData(ids[0]);
                auto* circle = requireGeometry(_storage->get<Figures::Circle2D>(ids[0]));
                const auto dependencies = _storage->getDependencies(ids[0]);
                if (dependencies.size() != 1) {
                    throw std::runtime_error("Circle dependencies are inconsistent");
                }
                auto* originalCenter = requireGeometry(_storage->get<Figures::Point2D>(dependencies[0]));
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXCIRCLE,
                    std::vector<VAR>{center->ptrX()},
                    originalCenter->x()));
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXCIRCLE,
                    std::vector<VAR>{center->ptrY()},
                    originalCenter->y()));
                addFunction(std::make_shared<Function::FixCoordinateFunction>(
                    Utils::RequirementType::ET_FIXCIRCLE,
                    std::vector<VAR>{radius},
                    circle->radius));
                break;
            }
        }
    }

    applyDirectAssignments();
    synchronizeCoincidentPoints();
}

const RequirementSystem::RequirementEntry* RequirementSystem::getRequirement(Utils::ID reqId) const noexcept {
    for (const auto& entry : _requirements) {
        if (entry.id == reqId) {
            return &entry;
        }
    }
    return nullptr;
}

bool RequirementSystem::hasRequirement(Utils::ID reqId) const noexcept {
    return getRequirement(reqId) != nullptr;
}

std::optional<Utils::RequirementType> RequirementSystem::getRequirementType(Utils::ID reqId) const noexcept {
    const auto* entry = getRequirement(reqId);
    if (entry == nullptr) {
        return std::nullopt;
    }
    return entry->type;
}

std::optional<std::vector<Utils::ID>> RequirementSystem::getRequirementObjectIds(Utils::ID reqId) const noexcept {
    const auto* entry = getRequirement(reqId);
    if (entry == nullptr) {
        return std::nullopt;
    }
    return entry->objectIds;
}

std::optional<double> RequirementSystem::getRequirementParam(Utils::ID reqId) const noexcept {
    const auto* entry = getRequirement(reqId);
    if (entry == nullptr) {
        return std::nullopt;
    }
    return entry->param;
}

void RequirementSystem::addPointLineDist(Utils::ID pointId, Utils::ID lineId, double dist) {
    addRequirement(Utils::RequirementDescriptor::pointLineDist(pointId, lineId, dist));
}

void RequirementSystem::addPointOnLine(Utils::ID pointId, Utils::ID lineId) {
    addRequirement(Utils::RequirementDescriptor::pointOnLine(pointId, lineId));
}

void RequirementSystem::addPointPointDist(Utils::ID p1Id, Utils::ID p2Id, double dist) {
    addRequirement(Utils::RequirementDescriptor::pointPointDist(p1Id, p2Id, dist));
}

void RequirementSystem::addPointOnPoint(Utils::ID p1Id, Utils::ID p2Id) {
    addRequirement(Utils::RequirementDescriptor::pointOnPoint(p1Id, p2Id));
}

void RequirementSystem::addLineCircleDist(Utils::ID lineId, Utils::ID circleId, double dist) {
    addRequirement(Utils::RequirementDescriptor::lineCircleDist(lineId, circleId, dist));
}

void RequirementSystem::addLineOnCircle(Utils::ID lineId, Utils::ID circleId) {
    addRequirement(Utils::RequirementDescriptor::lineOnCircle(lineId, circleId));
}

void RequirementSystem::addLineLineParallel(Utils::ID l1Id, Utils::ID l2Id) {
    addRequirement(Utils::RequirementDescriptor::lineLineParallel(l1Id, l2Id));
}

void RequirementSystem::addLineLinePerpendicular(Utils::ID l1Id, Utils::ID l2Id) {
    addRequirement(Utils::RequirementDescriptor::lineLinePerpendicular(l1Id, l2Id));
}

void RequirementSystem::addLineLineAngle(Utils::ID l1Id, Utils::ID l2Id, double angle) {
    addRequirement(Utils::RequirementDescriptor::lineLineAngle(l1Id, l2Id, angle));
}

void RequirementSystem::addVertical(Utils::ID lineId) {
    addRequirement(Utils::RequirementDescriptor::vertical(lineId));
}

void RequirementSystem::addHorizontal(Utils::ID lineId) {
    addRequirement(Utils::RequirementDescriptor::horizontal(lineId));
}

void RequirementSystem::addArcCenterOnPerpendicular(Utils::ID arcId) {
    addRequirement(Utils::RequirementDescriptor::arcCenterOnPerpendicular(arcId));
}

void RequirementSystem::addFixPoint(Utils::ID pointId) {
    addRequirement(Utils::RequirementDescriptor::fixPoint(pointId));
}

void RequirementSystem::addFixLine(Utils::ID lineId) {
    addRequirement(Utils::RequirementDescriptor::fixLine(lineId));
}

void RequirementSystem::addFixCircle(Utils::ID circleId) {
    addRequirement(Utils::RequirementDescriptor::fixCircle(circleId));
}

void RequirementSystem::clear() {
    RequirementFunctionSystem::clear();
    _requirements.clear();
    _pointRepresentative.clear();
    _coincidentPointGroups.clear();
}

Utils::ID RequirementSystem::resolvePointRepresentative(Utils::ID pointId) const noexcept {
    const auto it = _pointRepresentative.find(pointId);
    if (it == _pointRepresentative.end()) {
        return pointId;
    }
    return it->second;
}

Figures::Point2D* RequirementSystem::resolvePoint(Utils::ID pointId) const {
    return requireGeometry(_storage->get<Figures::Point2D>(resolvePointRepresentative(pointId)));
}

std::vector<Utils::ID> RequirementSystem::getCoincidentPoints(Utils::ID pointId) const {
    const Utils::ID representative = resolvePointRepresentative(pointId);
    const auto groupIt = _coincidentPointGroups.find(representative);
    if (groupIt == _coincidentPointGroups.end()) {
        return {pointId};
    }
    return groupIt->second;
}

void RequirementSystem::applyDirectAssignments() const {
    for (const auto& function : getFunctions()) {
        VAR var = nullptr;
        double value = 0.0;
        if (function->tryGetAssignment(var, value)) {
            *var = value;
        }
    }
}

void RequirementSystem::synchronizeCoincidentPoints() const noexcept {
    if (_storage == nullptr) {
        return;
    }

    for (const auto& [representativeId, group] : _coincidentPointGroups) {
        auto* representative = _storage->get<Figures::Point2D>(representativeId);
        if (representative == nullptr) {
            continue;
        }

        for (const Utils::ID pointId : group) {
            if (pointId == representativeId) {
                continue;
            }

            auto* point = _storage->get<Figures::Point2D>(pointId);
            if (point == nullptr) {
                continue;
            }

            point->x() = representative->x();
            point->y() = representative->y();
        }
    }
}

Figures::ObjectGraph RequirementSystem::buildDependencyGraph() const {
    auto graph = _storage->buildObjectGraph();
    for (const auto& req : _requirements) {
        for (std::size_t i = 0; i < req.objectIds.size(); ++i) {
            for (std::size_t j = i + 1; j < req.objectIds.size(); ++j) {
                graph.addEdge(req.objectIds[i], req.objectIds[j], req.id);
            }
        }
    }

    return graph;
}

} // namespace OurPaintDCM::System
