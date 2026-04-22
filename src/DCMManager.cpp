#include "DCMManager.h"
#include "ErrorFunction.h"
#include "SparseLSMTask.h"
#include "sparse/SparseLevenbergMarquardtSolver.h"
#include <algorithm>
#include <memory>
#include <stdexcept>

namespace {

void eraseRequirementId(std::vector<OurPaintDCM::Utils::ID>& ids, OurPaintDCM::Utils::ID id) {
    ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
}

std::vector<Variable*> makeMathVariables(std::initializer_list<double*> refs) {
    std::vector<Variable*> vars;
    vars.reserve(refs.size());
    for (double* ref : refs) {
        vars.push_back(new Variable(ref));
    }
    return vars;
}

std::unique_ptr<::Function> makeFixResidual(double* valueRef, double target) {
    return std::unique_ptr<::Function>(
        new Subtraction(new Variable(valueRef), new Constant(target)));
}

} // anonymous namespace

namespace OurPaintDCM {

DCMManager::DCMManager()
    : _reqSystem(&_storage) {}

Utils::ID DCMManager::addFigure(const Utils::FigureDescriptor& descriptor) {
    descriptor.validate();

    Utils::ID figureId;
    std::vector<Utils::ID> relatedFigures;
    Utils::FigureDescriptor storedDesc = descriptor;

    auto registerPoint = [&](Utils::ID pid, double px, double py) {
        Utils::FigureDescriptor pd;
        pd.type = Utils::FigureType::ET_POINT2D;
        pd.id = pid;
        pd.coords = {px, py};
        pd.x = px;
        pd.y = py;
        _figureRecords[pid] = pd;
        ComponentID c = createNewComponent();
        addFigureToComponent(pid, c);
    };

    switch (descriptor.type) {
        case Utils::FigureType::ET_POINT2D: {
            const double px = descriptor.coords.size() == 2 ? descriptor.coords[0] : descriptor.x.value();
            const double py = descriptor.coords.size() == 2 ? descriptor.coords[1] : descriptor.y.value();
            figureId = _storage.createPoint(px, py);
            break;
        }
        case Utils::FigureType::ET_LINE: {
            if (descriptor.coords.size() == 4) {
                const Utils::ID p1Id = _storage.createPoint(descriptor.coords[0], descriptor.coords[1]);
                const Utils::ID p2Id = _storage.createPoint(descriptor.coords[2], descriptor.coords[3]);
                registerPoint(p1Id, descriptor.coords[0], descriptor.coords[1]);
                registerPoint(p2Id, descriptor.coords[2], descriptor.coords[3]);
                auto lineOpt = _storage.createLine(p1Id, p2Id);
                if (!lineOpt) {
                    throw std::runtime_error("Line creation failed");
                }
                figureId = *lineOpt;
                relatedFigures = {p1Id, p2Id};
                storedDesc.pointIds = relatedFigures;
            } else {
                auto lineOpt = _storage.createLine(descriptor.pointIds[0], descriptor.pointIds[1]);
                if (!lineOpt) {
                    throw std::runtime_error("Line creation failed");
                }
                figureId = *lineOpt;
                relatedFigures = descriptor.pointIds;
            }
            break;
        }
        case Utils::FigureType::ET_CIRCLE: {
            if (descriptor.coords.size() == 2) {
                const Utils::ID centerId = _storage.createPoint(descriptor.coords[0], descriptor.coords[1]);
                registerPoint(centerId, descriptor.coords[0], descriptor.coords[1]);
                auto circOpt = _storage.createCircle(centerId, descriptor.radius.value());
                if (!circOpt) {
                    throw std::runtime_error("Circle creation failed");
                }
                figureId = *circOpt;
                relatedFigures = {centerId};
                storedDesc.pointIds = relatedFigures;
            } else {
                auto circOpt = _storage.createCircle(descriptor.pointIds[0], descriptor.radius.value());
                if (!circOpt) {
                    throw std::runtime_error("Circle creation failed");
                }
                figureId = *circOpt;
                relatedFigures = descriptor.pointIds;
            }
            break;
        }
        case Utils::FigureType::ET_ARC: {
            if (descriptor.coords.size() == 6) {
                const Utils::ID p1Id = _storage.createPoint(descriptor.coords[0], descriptor.coords[1]);
                const Utils::ID p2Id = _storage.createPoint(descriptor.coords[2], descriptor.coords[3]);
                const Utils::ID centerId = _storage.createPoint(descriptor.coords[4], descriptor.coords[5]);
                registerPoint(p1Id, descriptor.coords[0], descriptor.coords[1]);
                registerPoint(p2Id, descriptor.coords[2], descriptor.coords[3]);
                registerPoint(centerId, descriptor.coords[4], descriptor.coords[5]);
                auto arcOpt = _storage.createArc(p1Id, p2Id, centerId);
                if (!arcOpt) {
                    throw std::runtime_error("Arc creation failed");
                }
                figureId = *arcOpt;
                relatedFigures = {p1Id, p2Id, centerId};
                storedDesc.pointIds = relatedFigures;
            } else {
                auto arcOpt = _storage.createArc(
                    descriptor.pointIds[0], descriptor.pointIds[1], descriptor.pointIds[2]);
                if (!arcOpt) {
                    throw std::runtime_error("Arc creation failed");
                }
                figureId = *arcOpt;
                relatedFigures = descriptor.pointIds;
            }
            break;
        }
        default:
            throw std::invalid_argument("Unsupported figure type for addFigure");
    }

    storedDesc.id = figureId;
    _figureRecords[figureId] = storedDesc;

    ComponentID compId = createNewComponent();
    addFigureToComponent(figureId, compId);

    if (!relatedFigures.empty()) {
        relatedFigures.push_back(figureId);
        mergeComponents(relatedFigures);
    }

    return figureId;
}

void DCMManager::removeFigure(Utils::ID figureId, bool forceCascade) {
    if (!_storage.contains(figureId)) {
        throw std::runtime_error("Figure not found");
    }

    std::vector<Utils::ID> dependencyPoints;
    if (forceCascade) {
        auto reqs = getRequirementsForFigure(figureId);
        for (const auto& reqId : reqs) {
            removeRequirement(reqId);
        }
        dependencyPoints = _storage.getDependencies(figureId);
    }

    std::vector<Utils::ID> cascadedFigures;
    if (forceCascade) {
        const auto ty = _storage.getType(figureId);
        if (ty.has_value() && *ty == Utils::FigureType::ET_POINT2D) {
            cascadedFigures = _storage.getDependents(figureId);
        }
    }

    const auto removed = _storage.remove(figureId, forceCascade);
    if (removed == Figures::RemoveResult::NotFound) {
        throw std::runtime_error("Figure not found");
    }
    if (removed == Figures::RemoveResult::BlockedByDependents) {
        throw std::runtime_error("Dependencies exist");
    }

    for (const auto& id : cascadedFigures) {
        _figureRecords.erase(id);
        removeFigureFromComponent(id);
    }
    _figureRecords.erase(figureId);
    removeFigureFromComponent(figureId);

    if (forceCascade) {
        for (const auto& depPointId : dependencyPoints) {
            if (_storage.contains(depPointId)) {
                removeFigure(depPointId, true);
            }
        }
    }

    pruneRequirementsWithMissingObjects();
    splitComponentsAfterRemoval();
}

void DCMManager::updatePoint(const Utils::PointUpdateDescriptor& descriptor) {
    syncRequirementSystemIfNeeded();

    auto* point = _storage.get<Figures::Point2D>(descriptor.pointId);
    if (!point) {
        throw std::runtime_error("Point not found");
    }

    const Utils::ID solvePointId = _reqSystem.resolvePointRepresentative(descriptor.pointId);
    auto* solvePoint = _storage.get<Figures::Point2D>(solvePointId);
    if (solvePoint == nullptr) {
        throw std::runtime_error("Point not found");
    }

    const auto coincidentPoints = _reqSystem.getCoincidentPoints(descriptor.pointId);
    const std::unordered_set<Utils::ID> coincidentPointSet(coincidentPoints.begin(), coincidentPoints.end());

    auto pointGroupHasFixConstraint = [&]() {
        for (const auto& reqId : _requirementOrder) {
            const auto it = _requirementRecords.find(reqId);
            if (it == _requirementRecords.end()) {
                continue;
            }

            const auto& req = it->second;
            switch (req.type) {
                case Utils::RequirementType::ET_FIXPOINT: {
                    if (!req.objectIds.empty() && coincidentPointSet.contains(req.objectIds[0])) {
                        return true;
                    }
                    break;
                }
                case Utils::RequirementType::ET_FIXLINE: {
                    if (req.objectIds.empty()) {
                        break;
                    }
                    const auto dependencies = _storage.getDependencies(req.objectIds[0]);
                    for (const auto& pointId : dependencies) {
                        if (coincidentPointSet.contains(pointId)) {
                            return true;
                        }
                    }
                    break;
                }
                case Utils::RequirementType::ET_FIXCIRCLE: {
                    if (req.objectIds.empty()) {
                        break;
                    }
                    const auto dependencies = _storage.getDependencies(req.objectIds[0]);
                    if (!dependencies.empty() && coincidentPointSet.contains(dependencies[0])) {
                        return true;
                    }
                    break;
                }
                default:
                    break;
            }
        }
        return false;
    };

    if (pointGroupHasFixConstraint()) {
        _reqSystem.synchronizeCoincidentPoints();
        return;
    }

    if (descriptor.newX.has_value()) {
        solvePoint->x() = descriptor.newX.value();
    }
    if (descriptor.newY.has_value()) {
        solvePoint->y() = descriptor.newY.value();
    }

    _reqSystem.synchronizeCoincidentPoints();

    if (_solveMode == Utils::SolveMode::DRAG) {
        std::unordered_set<double*> lockedVars = {solvePoint->ptrX(), solvePoint->ptrY()};
        auto comp = getComponentForFigure(descriptor.pointId);
        if (comp.has_value()) {
            solveWithLockedVars(comp.value(), lockedVars);
        }
    }
}

void DCMManager::updateCircle(const Utils::CircleUpdateDescriptor& descriptor) {
    auto* circle = _storage.get<Figures::Circle2D>(descriptor.circleId);
    if (!circle) {
        throw std::runtime_error("Circle not found");
    }

    bool radiusHasFixConstraint = false;
    for (const auto& [_, req] : _requirementRecords) {
        if (req.type == Utils::RequirementType::ET_FIXCIRCLE &&
            !req.objectIds.empty() &&
            req.objectIds[0] == descriptor.circleId) {
            radiusHasFixConstraint = true;
            break;
        }
    }
    if (radiusHasFixConstraint) {
        return;
    }

    circle->radius = descriptor.newRadius;

    if (_solveMode == Utils::SolveMode::DRAG) {
        std::unordered_set<double*> lockedVars = {circle->ptrRadius()};
        auto comp = getComponentForFigure(descriptor.circleId);
        if (comp.has_value()) {
            solveWithLockedVars(comp.value(), lockedVars);
        }
    }
}

std::optional<Utils::FigureDescriptor> DCMManager::getFigure(Utils::ID figureId) const {
    auto it = _figureRecords.find(figureId);
    if (it == _figureRecords.end() || !_storage.contains(figureId)) {
        return std::nullopt;
    }

    Utils::FigureDescriptor desc = it->second;

    switch (desc.type) {
        case Utils::FigureType::ET_POINT2D: {
            auto* point = _storage.get<Figures::Point2D>(figureId);
            if (!point) {
                return std::nullopt;
            }
            desc.coords = {point->x(), point->y()};
            desc.x = point->x();
            desc.y = point->y();
            break;
        }
        case Utils::FigureType::ET_LINE: {
            if (desc.pointIds.size() < 2) {
                return std::nullopt;
            }
            const auto* line = _storage.get<Figures::Line2D>(figureId);
            if (line == nullptr || line->p1 == nullptr || line->p2 == nullptr) {
                return std::nullopt;
            }
            desc.coords = {line->p1->x(), line->p1->y(), line->p2->x(), line->p2->y()};
            break;
        }
        case Utils::FigureType::ET_CIRCLE: {
            if (desc.pointIds.empty()) {
                return std::nullopt;
            }
            const auto* circle = _storage.get<Figures::Circle2D>(figureId);
            if (circle == nullptr || circle->center == nullptr) {
                return std::nullopt;
            }
            desc.coords = {circle->center->x(), circle->center->y()};
            desc.radius = circle->radius;
            break;
        }
        case Utils::FigureType::ET_ARC: {
            if (desc.pointIds.size() < 3) {
                return std::nullopt;
            }
            const auto* arc = _storage.get<Figures::Arc2D>(figureId);
            if (arc == nullptr || arc->p1 == nullptr || arc->p2 == nullptr || arc->p_center == nullptr) {
                return std::nullopt;
            }
            desc.coords = {
                arc->p1->x(), arc->p1->y(), arc->p2->x(), arc->p2->y(),
                arc->p_center->x(), arc->p_center->y()};
            break;
        }
        default:
            return std::nullopt;
    }

    return desc;
}

bool DCMManager::hasFigure(Utils::ID figureId) const noexcept {
    return _figureRecords.contains(figureId) && _storage.contains(figureId);
}

std::vector<Utils::FigureDescriptor> DCMManager::getAllFigures() const {
    std::vector<Utils::FigureDescriptor> result;
    result.reserve(_figureRecords.size());
    for (const auto& [id, desc] : _figureRecords) {
        auto fullDesc = getFigure(id);
        if (fullDesc.has_value()) {
            result.push_back(std::move(*fullDesc));
        }
    }
    return result;
}

std::vector<Utils::FigureDescriptor> DCMManager::getAllPoints() const {
    const auto& points = _storage.pointsWithIds();
    std::vector<Utils::FigureDescriptor> result;
    result.reserve(points.size());
    for (const auto& ref : points) {
        if (ref.ptr == nullptr) {
            continue;
        }
        Utils::FigureDescriptor desc;
        desc.id = ref.id;
        desc.type = Utils::FigureType::ET_POINT2D;
        desc.coords = {ref.ptr->x(), ref.ptr->y()};
        desc.x = ref.ptr->x();
        desc.y = ref.ptr->y();
        result.push_back(std::move(desc));
    }
    return result;
}

std::vector<Utils::FigureDescriptor> DCMManager::getAllLines() const {
    const auto& lines = _storage.linesWithIds();
    std::vector<Utils::FigureDescriptor> result;
    result.reserve(lines.size());
    for (const auto& ref : lines) {
        const auto rec = _figureRecords.find(ref.id);
        if (rec == _figureRecords.end() || rec->second.pointIds.size() < 2 || ref.ptr == nullptr) {
            continue;
        }
        const Figures::Point2D* p1 = ref.ptr->p1;
        const Figures::Point2D* p2 = ref.ptr->p2;
        if (p1 == nullptr || p2 == nullptr) {
            continue;
        }
        Utils::FigureDescriptor desc;
        desc.id = ref.id;
        desc.type = Utils::FigureType::ET_LINE;
        desc.pointIds = rec->second.pointIds;
        desc.coords = {p1->x(), p1->y(), p2->x(), p2->y()};
        result.push_back(std::move(desc));
    }
    return result;
}

std::vector<Utils::FigureDescriptor> DCMManager::getAllCircles() const {
    const auto& circles = _storage.circlesWithIds();
    std::vector<Utils::FigureDescriptor> result;
    result.reserve(circles.size());
    for (const auto& ref : circles) {
        const auto rec = _figureRecords.find(ref.id);
        if (rec == _figureRecords.end() || rec->second.pointIds.empty() || ref.ptr == nullptr) {
            continue;
        }
        const Figures::Point2D* center = ref.ptr->center;
        if (center == nullptr) {
            continue;
        }
        Utils::FigureDescriptor desc;
        desc.id = ref.id;
        desc.type = Utils::FigureType::ET_CIRCLE;
        desc.pointIds = rec->second.pointIds;
        desc.coords = {center->x(), center->y()};
        desc.radius = ref.ptr->radius;
        result.push_back(std::move(desc));
    }
    return result;
}

std::vector<Utils::FigureDescriptor> DCMManager::getAllArcs() const {
    const auto& arcs = _storage.arcsWithIds();
    std::vector<Utils::FigureDescriptor> result;
    result.reserve(arcs.size());
    for (const auto& ref : arcs) {
        const auto rec = _figureRecords.find(ref.id);
        if (rec == _figureRecords.end() || rec->second.pointIds.size() < 3 || ref.ptr == nullptr) {
            continue;
        }
        const Figures::Point2D* p1 = ref.ptr->p1;
        const Figures::Point2D* p2 = ref.ptr->p2;
        const Figures::Point2D* center = ref.ptr->p_center;
        if (p1 == nullptr || p2 == nullptr || center == nullptr) {
            continue;
        }
        Utils::FigureDescriptor desc;
        desc.id = ref.id;
        desc.type = Utils::FigureType::ET_ARC;
        desc.pointIds = rec->second.pointIds;
        desc.coords = {p1->x(), p1->y(), p2->x(), p2->y(), center->x(), center->y()};
        result.push_back(std::move(desc));
    }
    return result;
}

Utils::ID DCMManager::addRequirement(const Utils::RequirementDescriptor& descriptor) {
    descriptor.validate();

    if (descriptor.id.has_value()) {
        if (descriptor.id->id == 0ULL) {
            throw std::invalid_argument("Requirement id must not be 0");
        }
        if (_requirementRecords.contains(*descriptor.id)) {
            throw std::invalid_argument("Requirement id already exists");
        }
    }

    syncRequirementSystemIfNeeded();

    Utils::ID reqId = _reqSystem.addRequirement(descriptor);

    Utils::RequirementDescriptor storedDesc = descriptor;
    storedDesc.id = reqId;
    _requirementRecords[reqId] = storedDesc;
    switch (storedDesc.type) {
        case Utils::RequirementType::ET_FIXPOINT: {
            auto* point = _storage.get<Figures::Point2D>(storedDesc.objectIds[0]);
            if (point == nullptr) {
                throw std::runtime_error("Point not found");
            }
            _fixedRequirementTargets[reqId] = {point->x(), point->y()};
            break;
        }
        case Utils::RequirementType::ET_FIXLINE: {
            const auto dependencies = _storage.getDependencies(storedDesc.objectIds[0]);
            if (dependencies.size() != 2) {
                throw std::runtime_error("Line dependencies are inconsistent");
            }
            auto* p1 = _storage.get<Figures::Point2D>(dependencies[0]);
            auto* p2 = _storage.get<Figures::Point2D>(dependencies[1]);
            if (p1 == nullptr || p2 == nullptr) {
                throw std::runtime_error("Line point not found");
            }
            _fixedRequirementTargets[reqId] = {p1->x(), p1->y(), p2->x(), p2->y()};
            break;
        }
        case Utils::RequirementType::ET_FIXCIRCLE: {
            auto* circle = _storage.get<Figures::Circle2D>(storedDesc.objectIds[0]);
            const auto dependencies = _storage.getDependencies(storedDesc.objectIds[0]);
            if (circle == nullptr || dependencies.size() != 1) {
                throw std::runtime_error("Circle dependencies are inconsistent");
            }
            auto* center = _storage.get<Figures::Point2D>(dependencies[0]);
            if (center == nullptr) {
                throw std::runtime_error("Circle center not found");
            }
            _fixedRequirementTargets[reqId] = {center->x(), center->y(), circle->radius};
            break;
        }
        default:
            _fixedRequirementTargets.erase(reqId);
            break;
    }
    _requirementOrder.push_back(reqId);

    mergeComponents(descriptor.objectIds);

    return reqId;
}

void DCMManager::removeRequirement(Utils::ID reqId) {
    auto it = _requirementRecords.find(reqId);
    if (it == _requirementRecords.end()) {
        throw std::runtime_error("Requirement not found");
    }

    _requirementRecords.erase(it);
    _fixedRequirementTargets.erase(reqId);
    eraseRequirementId(_requirementOrder, reqId);
    _reqSystemSyncedWithRecords = false;
    rebuildComponents();
}

void DCMManager::updateRequirementParam(Utils::ID reqId, double newParam) {
    auto it = _requirementRecords.find(reqId);
    if (it == _requirementRecords.end()) {
        throw std::runtime_error("Requirement not found");
    }

    if (!it->second.param.has_value()) {
        throw std::runtime_error("Requirement has no parameter");
    }

    it->second.param = newParam;
    _reqSystemSyncedWithRecords = false;
}

std::optional<Utils::RequirementDescriptor> DCMManager::getRequirement(Utils::ID reqId) const noexcept {
    auto it = _requirementRecords.find(reqId);
    if (it != _requirementRecords.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool DCMManager::hasRequirement(Utils::ID reqId) const noexcept {
    return _requirementRecords.contains(reqId);
}

std::vector<Utils::RequirementDescriptor> DCMManager::getAllRequirements() const {
    std::vector<Utils::RequirementDescriptor> result;
    result.reserve(_requirementRecords.size());
    for (const auto& reqId : _requirementOrder) {
        const auto it = _requirementRecords.find(reqId);
        if (it != _requirementRecords.end()) {
            result.push_back(it->second);
        }
    }
    return result;
}

std::size_t DCMManager::getComponentCount() const noexcept {
    return _activeComponentCount;
}

std::optional<ComponentID> DCMManager::getComponentForFigure(Utils::ID figureId) const noexcept {
    auto it = _figureToComponent.find(figureId);
    if (it != _figureToComponent.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<Utils::ID> DCMManager::getFiguresInComponent(ComponentID componentId) const {
    if (componentId >= _components.size()) {
        return {};
    }
    return {_components[componentId].begin(), _components[componentId].end()};
}

std::vector<Utils::ID> DCMManager::getRequirementsInComponent(ComponentID componentId) const {
    if (componentId >= _components.size() || _components[componentId].empty()) {
        return {};
    }

    std::vector<Utils::ID> result;
    const auto& compFigures = _components[componentId];

    for (const auto& reqId : _requirementOrder) {
        const auto it = _requirementRecords.find(reqId);
        if (it == _requirementRecords.end()) {
            continue;
        }

        const auto& desc = it->second;
        for (const auto& objId : desc.objectIds) {
            if (compFigures.contains(objId)) {
                result.push_back(reqId);
                break;
            }
        }
    }

    return result;
}

std::vector<std::vector<Utils::ID>> DCMManager::getAllComponents() const {
    std::vector<std::vector<Utils::ID>> result;
    result.reserve(_activeComponentCount);
    for (const auto& comp : _components) {
        if (!comp.empty()) {
            result.emplace_back(comp.begin(), comp.end());
        }
    }
    return result;
}

const Figures::GeometryStorage& DCMManager::getStorage() const noexcept {
    return _storage;
}

const System::RequirementSystem& DCMManager::getRequirementSystem() const {
    const_cast<DCMManager*>(this)->syncRequirementSystemIfNeeded();
    return _reqSystem;
}

Figures::GeometryStorage& DCMManager::storage() noexcept {
    return _storage;
}

System::RequirementSystem& DCMManager::requirementSystem() {
    syncRequirementSystemIfNeeded();
    return _reqSystem;
}

std::size_t DCMManager::figureCount() const noexcept {
    return _storage.size();
}

std::size_t DCMManager::requirementCount() const noexcept {
    return _requirementRecords.size();
}

void DCMManager::clear() {
    _reqSystem.clear();
    _storage.clear();
    _requirementRecords.clear();
    _fixedRequirementTargets.clear();
    _requirementOrder.clear();
    _figureRecords.clear();
    _figureToComponent.clear();
    _components.clear();
    _nextComponentId = 0;
    _activeComponentCount = 0;
    _reqSystemSyncedWithRecords = true;
}

void DCMManager::setSolveMode(Utils::SolveMode mode) noexcept {
    _solveMode = mode;
}

Utils::SolveMode DCMManager::getSolveMode() const noexcept {
    return _solveMode;
}

bool DCMManager::solve(std::optional<ComponentID> componentId) {
    const std::unordered_set<double*> noLockedVars;
    return solveWithLockedVars(componentId, noLockedVars);
}

bool DCMManager::solveWithLockedVars(std::optional<ComponentID> componentId,
                                     const std::unordered_set<double*>& lockedVars) {
    if (_requirementRecords.empty()) {
        return true;
    }

    System::RequirementSystem* systemPtr = nullptr;
    std::unique_ptr<System::RequirementSystem> subsystem;

    switch (_solveMode) {
        case Utils::SolveMode::GLOBAL:
            syncRequirementSystemIfNeeded();
            systemPtr = &_reqSystem;
            break;
        case Utils::SolveMode::LOCAL:
            if (!componentId.has_value())
                throw std::runtime_error("LOCAL mode requires a componentID");
            subsystem = buildSubsystem(componentId.value());
            systemPtr = subsystem.get();
            break;
        case Utils::SolveMode::DRAG:
            if (componentId.has_value()) {
                subsystem = buildSubsystem(componentId.value());
                systemPtr = subsystem.get();
            } else {
                syncRequirementSystemIfNeeded();
                systemPtr = &_reqSystem;
            }
            break;
    }

    auto& system = *systemPtr;
    if (system.getRequirements().empty()) {
        system.synchronizeCoincidentPoints();
        return true;
    }

    std::vector<std::unique_ptr<::Function>> mathFunctionOwners;
    std::vector<double*> mathVariableRefs;
    std::unordered_set<double*> mathVariableRefSet;
    std::unordered_map<double*, double> fixedAssignments;

    const auto rememberVariable = [&](double* valueRef) {
        if (valueRef != nullptr &&
            !lockedVars.contains(valueRef) &&
            !fixedAssignments.contains(valueRef) &&
            mathVariableRefSet.insert(valueRef).second) {
            mathVariableRefs.push_back(valueRef);
        }
    };

    const auto appendFunction = [&](std::unique_ptr<::Function> function, std::initializer_list<double*> refs) {
        for (double* ref : refs) {
            rememberVariable(ref);
        }
        mathFunctionOwners.push_back(std::move(function));
    };

    const auto resolveLinePoints = [&](Utils::ID lineId) {
        const auto dependencies = _storage.getDependencies(lineId);
        if (dependencies.size() != 2) {
            throw std::runtime_error("Line dependencies are inconsistent");
        }
        return std::pair{system.resolvePoint(dependencies[0]), system.resolvePoint(dependencies[1])};
    };

    const auto resolveCircleData = [&](Utils::ID circleId) {
        auto* circle = _storage.get<Figures::Circle2D>(circleId);
        const auto dependencies = _storage.getDependencies(circleId);
        if (circle == nullptr || dependencies.size() != 1) {
            throw std::runtime_error("Circle dependencies are inconsistent");
        }
        return std::tuple{system.resolvePoint(dependencies[0]), circle->ptrRadius(), circle};
    };

    const auto resolveArcPoints = [&](Utils::ID arcId) {
        const auto dependencies = _storage.getDependencies(arcId);
        if (dependencies.size() != 3) {
            throw std::runtime_error("Arc dependencies are inconsistent");
        }
        return std::tuple{
            system.resolvePoint(dependencies[0]),
            system.resolvePoint(dependencies[1]),
            system.resolvePoint(dependencies[2])};
    };

    for (const auto& entry : system.getRequirements()) {
        const auto& ids = entry.objectIds;
        switch (entry.type) {
            case Utils::RequirementType::ET_FIXPOINT: {
                auto* point = system.resolvePoint(ids[0]);
                std::vector<double> targets = {point->x(), point->y()};
                const auto targetIt = _fixedRequirementTargets.find(entry.id);
                if (targetIt != _fixedRequirementTargets.end() && targetIt->second.size() == 2) {
                    targets = targetIt->second;
                }
                fixedAssignments[point->ptrX()] = targets[0];
                fixedAssignments[point->ptrY()] = targets[1];
                break;
            }
            case Utils::RequirementType::ET_FIXLINE: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                std::vector<double> targets = {lineP1->x(), lineP1->y(), lineP2->x(), lineP2->y()};
                const auto targetIt = _fixedRequirementTargets.find(entry.id);
                if (targetIt != _fixedRequirementTargets.end() && targetIt->second.size() == 4) {
                    targets = targetIt->second;
                }
                fixedAssignments[lineP1->ptrX()] = targets[0];
                fixedAssignments[lineP1->ptrY()] = targets[1];
                fixedAssignments[lineP2->ptrX()] = targets[2];
                fixedAssignments[lineP2->ptrY()] = targets[3];
                break;
            }
            case Utils::RequirementType::ET_FIXCIRCLE: {
                const auto [center, radius, circle] = resolveCircleData(ids[0]);
                (void)circle;
                std::vector<double> targets = {center->x(), center->y(), *radius};
                const auto targetIt = _fixedRequirementTargets.find(entry.id);
                if (targetIt != _fixedRequirementTargets.end() && targetIt->second.size() == 3) {
                    targets = targetIt->second;
                }
                fixedAssignments[center->ptrX()] = targets[0];
                fixedAssignments[center->ptrY()] = targets[1];
                fixedAssignments[radius] = targets[2];
                break;
            }
            default:
                break;
        }
    }

    for (const auto& [valueRef, target] : fixedAssignments) {
        *valueRef = target;
    }

    for (const auto& entry : system.getRequirements()) {
        const auto& ids = entry.objectIds;

        switch (entry.type) {
            case Utils::RequirementType::ET_POINTLINEDIST: {
                auto* point = system.resolvePoint(ids[0]);
                const auto [lineP1, lineP2] = resolveLinePoints(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new PointSectionDistanceError(
                        makeMathVariables({
                            point->ptrX(), point->ptrY(),
                            lineP1->ptrX(), lineP1->ptrY(),
                            lineP2->ptrX(), lineP2->ptrY()}),
                        entry.param.value())),
                    {
                        point->ptrX(), point->ptrY(),
                        lineP1->ptrX(), lineP1->ptrY(),
                        lineP2->ptrX(), lineP2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_POINTONLINE: {
                auto* point = system.resolvePoint(ids[0]);
                const auto [lineP1, lineP2] = resolveLinePoints(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new PointOnSectionError(
                        makeMathVariables({
                            point->ptrX(), point->ptrY(),
                            lineP1->ptrX(), lineP1->ptrY(),
                            lineP2->ptrX(), lineP2->ptrY()}))),
                    {
                        point->ptrX(), point->ptrY(),
                        lineP1->ptrX(), lineP1->ptrY(),
                        lineP2->ptrX(), lineP2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_POINTPOINTDIST: {
                auto* p1 = system.resolvePoint(ids[0]);
                auto* p2 = system.resolvePoint(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new PointPointDistanceError(
                        makeMathVariables({p1->ptrX(), p1->ptrY(), p2->ptrX(), p2->ptrY()}),
                        entry.param.value())),
                    {p1->ptrX(), p1->ptrY(), p2->ptrX(), p2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_POINTONPOINT:
                // Coincident points are already merged by RequirementSystem aliasing.
                // Adding a separate residual here double-counts the same constraint
                // and can create stationary but inconsistent solve states.
                break;
            case Utils::RequirementType::ET_LINECIRCLEDIST: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                const auto [center, radius, _circle] = resolveCircleData(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new SectionCircleDistanceError(
                        makeMathVariables({
                            lineP1->ptrX(), lineP1->ptrY(),
                            lineP2->ptrX(), lineP2->ptrY(),
                            center->ptrX(), center->ptrY(),
                            radius}),
                        entry.param.value())),
                    {
                        lineP1->ptrX(), lineP1->ptrY(),
                        lineP2->ptrX(), lineP2->ptrY(),
                        center->ptrX(), center->ptrY(),
                        radius});
                break;
            }
            case Utils::RequirementType::ET_LINEONCIRCLE: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                const auto [center, radius, _circle] = resolveCircleData(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new SectionOnCircleError(
                        makeMathVariables({
                            lineP1->ptrX(), lineP1->ptrY(),
                            lineP2->ptrX(), lineP2->ptrY(),
                            center->ptrX(), center->ptrY(),
                            radius}))),
                    {
                        lineP1->ptrX(), lineP1->ptrY(),
                        lineP2->ptrX(), lineP2->ptrY(),
                        center->ptrX(), center->ptrY(),
                        radius});
                break;
            }
            case Utils::RequirementType::ET_LINEINCIRCLE: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                const auto [center, radius, _circle] = resolveCircleData(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new SectionInCircleError(
                        makeMathVariables({
                            lineP1->ptrX(), lineP1->ptrY(),
                            lineP2->ptrX(), lineP2->ptrY(),
                            center->ptrX(), center->ptrY(),
                            radius}))),
                    {
                        lineP1->ptrX(), lineP1->ptrY(),
                        lineP2->ptrX(), lineP2->ptrY(),
                        center->ptrX(), center->ptrY(),
                        radius});
                break;
            }
            case Utils::RequirementType::ET_LINELINEPARALLEL: {
                const auto [l1p1, l1p2] = resolveLinePoints(ids[0]);
                const auto [l2p1, l2p2] = resolveLinePoints(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new SectionSectionParallelError(
                        makeMathVariables({
                            l1p1->ptrX(), l1p1->ptrY(), l1p2->ptrX(), l1p2->ptrY(),
                            l2p1->ptrX(), l2p1->ptrY(), l2p2->ptrX(), l2p2->ptrY()}))),
                    {
                        l1p1->ptrX(), l1p1->ptrY(), l1p2->ptrX(), l1p2->ptrY(),
                        l2p1->ptrX(), l2p1->ptrY(), l2p2->ptrX(), l2p2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_LINELINEPERPENDICULAR: {
                const auto [l1p1, l1p2] = resolveLinePoints(ids[0]);
                const auto [l2p1, l2p2] = resolveLinePoints(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new SectionSectionPerpendicularError(
                        makeMathVariables({
                            l1p1->ptrX(), l1p1->ptrY(), l1p2->ptrX(), l1p2->ptrY(),
                            l2p1->ptrX(), l2p1->ptrY(), l2p2->ptrX(), l2p2->ptrY()}))),
                    {
                        l1p1->ptrX(), l1p1->ptrY(), l1p2->ptrX(), l1p2->ptrY(),
                        l2p1->ptrX(), l2p1->ptrY(), l2p2->ptrX(), l2p2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_LINELINEANGLE: {
                const auto [l1p1, l1p2] = resolveLinePoints(ids[0]);
                const auto [l2p1, l2p2] = resolveLinePoints(ids[1]);
                appendFunction(
                    std::unique_ptr<::Function>(new SectionSectionAngleError(
                        makeMathVariables({
                            l1p1->ptrX(), l1p1->ptrY(), l1p2->ptrX(), l1p2->ptrY(),
                            l2p1->ptrX(), l2p1->ptrY(), l2p2->ptrX(), l2p2->ptrY()}),
                        entry.param.value())),
                    {
                        l1p1->ptrX(), l1p1->ptrY(), l1p2->ptrX(), l1p2->ptrY(),
                        l2p1->ptrX(), l2p1->ptrY(), l2p2->ptrX(), l2p2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_VERTICAL: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                appendFunction(
                    std::unique_ptr<::Function>(new VerticalError(
                        makeMathVariables({lineP1->ptrX(), lineP1->ptrY(), lineP2->ptrX(), lineP2->ptrY()}))),
                    {lineP1->ptrX(), lineP1->ptrY(), lineP2->ptrX(), lineP2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_HORIZONTAL: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                appendFunction(
                    std::unique_ptr<::Function>(new HorizontalError(
                        makeMathVariables({lineP1->ptrX(), lineP1->ptrY(), lineP2->ptrX(), lineP2->ptrY()}))),
                    {lineP1->ptrX(), lineP1->ptrY(), lineP2->ptrX(), lineP2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_ARCCENTERONPERPENDICULAR: {
                const auto [arcP1, arcP2, center] = resolveArcPoints(ids[0]);
                appendFunction(
                    std::unique_ptr<::Function>(new ArcCenterOnPerpendicularError(
                        makeMathVariables({
                            arcP1->ptrX(), arcP1->ptrY(),
                            arcP2->ptrX(), arcP2->ptrY(),
                            center->ptrX(), center->ptrY()}))),
                    {
                        arcP1->ptrX(), arcP1->ptrY(),
                        arcP2->ptrX(), arcP2->ptrY(),
                        center->ptrX(), center->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_FIXPOINT: {
                auto* point = system.resolvePoint(ids[0]);
                std::vector<double> targets = {point->x(), point->y()};
                const auto targetIt = _fixedRequirementTargets.find(entry.id);
                if (targetIt != _fixedRequirementTargets.end() && targetIt->second.size() == 2) {
                    targets = targetIt->second;
                }
                appendFunction(makeFixResidual(point->ptrX(), targets[0]), {point->ptrX()});
                appendFunction(makeFixResidual(point->ptrY(), targets[1]), {point->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_FIXLINE: {
                const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                std::vector<double> targets = {lineP1->x(), lineP1->y(), lineP2->x(), lineP2->y()};
                const auto targetIt = _fixedRequirementTargets.find(entry.id);
                if (targetIt != _fixedRequirementTargets.end() && targetIt->second.size() == 4) {
                    targets = targetIt->second;
                }
                appendFunction(makeFixResidual(lineP1->ptrX(), targets[0]), {lineP1->ptrX()});
                appendFunction(makeFixResidual(lineP1->ptrY(), targets[1]), {lineP1->ptrY()});
                appendFunction(makeFixResidual(lineP2->ptrX(), targets[2]), {lineP2->ptrX()});
                appendFunction(makeFixResidual(lineP2->ptrY(), targets[3]), {lineP2->ptrY()});
                break;
            }
            case Utils::RequirementType::ET_FIXCIRCLE: {
                const auto [center, radius, circle] = resolveCircleData(ids[0]);
                (void)circle;
                std::vector<double> targets = {center->x(), center->y(), *radius};
                const auto targetIt = _fixedRequirementTargets.find(entry.id);
                if (targetIt != _fixedRequirementTargets.end() && targetIt->second.size() == 3) {
                    targets = targetIt->second;
                }
                appendFunction(makeFixResidual(center->ptrX(), targets[0]), {center->ptrX()});
                appendFunction(makeFixResidual(center->ptrY(), targets[1]), {center->ptrY()});
                appendFunction(makeFixResidual(radius, targets[2]), {radius});
                break;
            }
        }
    }

    if (mathFunctionOwners.empty()) {
        system.synchronizeCoincidentPoints();
        return true;
    }

    if (mathVariableRefs.empty()) {
        // If temporary drag locks consume all remaining DOF, retry without locks.
        // This keeps fixed/eliminated vars constant, but allows the solver
        // to satisfy constraints by moving the dragged point to a feasible position.
        if (!lockedVars.empty()) {
            const std::unordered_set<double*> noLockedVars;
            return solveWithLockedVars(componentId, noLockedVars);
        }
        system.synchronizeCoincidentPoints();
        return true;
    }

    std::vector<::Function*> mathFuncs;
    mathFuncs.reserve(mathFunctionOwners.size());
    for (auto& owner : mathFunctionOwners) {
        mathFuncs.push_back(owner.release());
    }

    std::vector<std::unique_ptr<Variable>> mathVarOwners;
    std::vector<Variable*> mathVars;
    mathVarOwners.reserve(mathVariableRefs.size());
    mathVars.reserve(mathVariableRefs.size());
    for (double* valueRef : mathVariableRefs) {
        mathVarOwners.push_back(std::make_unique<Variable>(valueRef));
        mathVars.push_back(mathVarOwners.back().get());
    }

    std::cout << "[Solver] System size before solve: equations=" << mathFuncs.size()
              << ", variables=" << mathVars.size() << std::endl;

    SparseLSMTask task(std::move(mathFuncs), std::move(mathVars));
    SparseLMSolver solver;
    solver.setTask(&task);
    solver.optimize();
    const bool converged = solver.isConverged();
    system.synchronizeCoincidentPoints();

    return converged;
}

std::unique_ptr<System::RequirementSystem> DCMManager::buildSubsystem(ComponentID componentId) const {
    auto subsystem = std::make_unique<System::RequirementSystem>(
        &const_cast<DCMManager*>(this)->_storage);

    auto reqIds = getRequirementsInComponent(componentId);
    for (const auto& reqId : reqIds) {
        auto it = _requirementRecords.find(reqId);
        if (it != _requirementRecords.end()) {
            subsystem->addRequirement(it->second);
        }
    }

    return subsystem;
}

void DCMManager::rebuildRequirementSystem() {
    _reqSystem.clear();
    for (const auto& reqId : _requirementOrder) {
        const auto it = _requirementRecords.find(reqId);
        if (it != _requirementRecords.end()) {
            _reqSystem.addRequirement(it->second);
        }
    }
    _reqSystemSyncedWithRecords = true;
}

void DCMManager::syncRequirementSystemIfNeeded() {
    if (_reqSystemSyncedWithRecords) {
        return;
    }
    rebuildRequirementSystem();
}

void DCMManager::pruneRequirementsWithMissingObjects() {
    bool changed = false;
    std::vector<Utils::ID> staleRequirementIds;
    for (auto it = _requirementRecords.begin(); it != _requirementRecords.end();) {
        bool stale = false;
        for (const auto& oid : it->second.objectIds) {
            if (!_storage.contains(oid)) {
                stale = true;
                break;
            }
        }
        if (stale) {
            staleRequirementIds.push_back(it->first);
            _fixedRequirementTargets.erase(it->first);
            it = _requirementRecords.erase(it);
            changed = true;
        } else {
            ++it;
        }
    }
    for (const auto& reqId : staleRequirementIds) {
        eraseRequirementId(_requirementOrder, reqId);
    }
    if (changed) {
        _reqSystemSyncedWithRecords = false;
    }
}

void DCMManager::rebuildComponents() {
    _figureToComponent.clear();
    _components.clear();
    _nextComponentId = 0;
    _activeComponentCount = 0;

    for (const auto& entry : _figureRecords) {
        const ComponentID compId = createNewComponent();
        addFigureToComponent(entry.first, compId);
    }

    for (const auto& entry : _requirementRecords) {
        mergeComponents(entry.second.objectIds);
    }
}

void DCMManager::mergeComponents(const std::vector<Utils::ID>& figureIds) {
    if (figureIds.empty()) {
        return;
    }

    std::unordered_set<ComponentID> componentsToMerge;
    for (const auto& fid : figureIds) {
        auto it = _figureToComponent.find(fid);
        if (it != _figureToComponent.end()) {
            componentsToMerge.insert(it->second);
        }
    }

    if (componentsToMerge.size() <= 1) {
        return;
    }

    auto targetIt = componentsToMerge.begin();
    ComponentID targetCompId = *targetIt;
    ++targetIt;

    while (targetIt != componentsToMerge.end()) {
        ComponentID srcCompId = *targetIt;

        for (const auto& figId : _components[srcCompId]) {
            _components[targetCompId].insert(figId);
            _figureToComponent[figId] = targetCompId;
        }
        _components[srcCompId].clear();
        --_activeComponentCount;

        ++targetIt;
    }
}

void DCMManager::splitComponentsAfterRemoval() {
    rebuildComponents();
}

ComponentID DCMManager::createNewComponent() {
    ComponentID id = _nextComponentId++;
    if (id >= _components.size()) {
        _components.resize(id + 1);
    }
    ++_activeComponentCount;
    return id;
}

void DCMManager::addFigureToComponent(Utils::ID figureId, ComponentID componentId) {
    if (componentId >= _components.size()) {
        _components.resize(componentId + 1);
    }
    _components[componentId].insert(figureId);
    _figureToComponent[figureId] = componentId;
}

void DCMManager::removeFigureFromComponent(Utils::ID figureId) {
    auto it = _figureToComponent.find(figureId);
    if (it != _figureToComponent.end()) {
        ComponentID compId = it->second;
        _components[compId].erase(figureId);
        if (_components[compId].empty()) {
            --_activeComponentCount;
        }
        _figureToComponent.erase(it);
    }
}

std::vector<Utils::ID> DCMManager::getRequirementsForFigure(Utils::ID figureId) const {
    std::vector<Utils::ID> result;
    for (const auto& reqId : _requirementOrder) {
        const auto it = _requirementRecords.find(reqId);
        if (it == _requirementRecords.end()) {
            continue;
        }

        const auto& desc = it->second;
        for (const auto& objId : desc.objectIds) {
            if (objId == figureId) {
                result.push_back(reqId);
                break;
            }
        }
    }
    return result;
}

} // namespace OurPaintDCM
