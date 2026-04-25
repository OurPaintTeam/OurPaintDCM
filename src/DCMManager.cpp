#include "DCMManager.h"
#include "ErrorFunction.h"
#include "SparseLSMTask.h"
#include "sparse/SparseLevenbergMarquardtSolver.h"
#include <algorithm>
#include <cstdint>
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

void hashCombine(std::size_t& seed, std::size_t value) {
    seed ^= value + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
}

struct SolveCacheKey {
    std::optional<OurPaintDCM::ComponentID> componentId;
    std::vector<double*> lockedVars;

    bool operator==(const SolveCacheKey& other) const noexcept {
        return componentId == other.componentId && lockedVars == other.lockedVars;
    }
};

struct SolveCacheKeyHasher {
    std::size_t operator()(const SolveCacheKey& key) const noexcept {
        std::size_t seed = 0;
        hashCombine(seed,
                    std::hash<std::size_t>{}(
                        key.componentId.value_or(static_cast<OurPaintDCM::ComponentID>(-1))));
        for (double* valueRef : key.lockedVars) {
            hashCombine(seed, std::hash<std::uintptr_t>{}(reinterpret_cast<std::uintptr_t>(valueRef)));
        }
        return seed;
    }
};

using FixedAssignmentMap = std::unordered_map<double*, double>;

struct BuiltSolvePipeline {
    FixedAssignmentMap fixedAssignments;
    std::vector<std::unique_ptr<Variable>> variableOwners;
    std::unique_ptr<SparseLSMTask> task;
    bool hasFunctions = false;
    bool hasFreeVariables = false;
};

} // anonymous namespace

struct OurPaintDCM::DCMManager::SolveCache {
    struct Entry {
        std::size_t version = 0;
        std::unique_ptr<System::RequirementSystem> subsystem;
        FixedAssignmentMap fixedAssignments;
        std::vector<std::unique_ptr<Variable>> variableOwners;
        std::unique_ptr<SparseLSMTask> task;
        std::unique_ptr<SparseLMSolver> solver;
        bool hasFunctions = false;
        bool hasFreeVariables = false;
    };

    std::size_t version = 0;
    std::unordered_map<SolveCacheKey, Entry, SolveCacheKeyHasher> entries;
};

struct OurPaintDCM::DCMManager::BatchUpdateContext {
    std::unordered_map<ComponentID, std::unordered_set<double*>> lockedVarsByComponent;
    bool needsCoincidentSync = false;
};

struct OurPaintDCM::DCMManager::FixedGeometry {
    std::unordered_set<Utils::ID> pointIds;
    std::unordered_set<Utils::ID> circleIds;
};

namespace OurPaintDCM {

DCMManager::DCMManager()
    : _reqSystem(&_storage),
      _solveCache(std::make_unique<SolveCache>()) {}

DCMManager::~DCMManager() = default;

void DCMManager::invalidateSolveCache() noexcept {
    if (_solveCache == nullptr) {
        return;
    }
    ++_solveCache->version;
    _solveCache->entries.clear();
}

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

    invalidateSolveCache();
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
    invalidateSolveCache();
}

DCMManager::FixedGeometry DCMManager::collectFixedGeometry() const {
    FixedGeometry fixed;

    for (const auto& reqId : _requirementOrder) {
        const auto it = _requirementRecords.find(reqId);
        if (it == _requirementRecords.end() || it->second.objectIds.empty()) {
            continue;
        }

        const auto& req = it->second;
        switch (req.type) {
            case Utils::RequirementType::ET_FIXPOINT:
                fixed.pointIds.insert(req.objectIds[0]);
                break;
            case Utils::RequirementType::ET_FIXLINE: {
                const auto dependencies = _storage.getDependencies(req.objectIds[0]);
                fixed.pointIds.insert(dependencies.begin(), dependencies.end());
                break;
            }
            case Utils::RequirementType::ET_FIXCIRCLE: {
                fixed.circleIds.insert(req.objectIds[0]);
                const auto dependencies = _storage.getDependencies(req.objectIds[0]);
                if (!dependencies.empty()) {
                    fixed.pointIds.insert(dependencies[0]);
                }
                break;
            }
            default:
                break;
        }
    }

    return fixed;
}

bool DCMManager::pointGroupHasFixConstraint(
    Utils::ID pointId,
    const std::unordered_set<Utils::ID>& fixedPointIds) const {
    const auto coincidentPoints = _reqSystem.getCoincidentPoints(pointId);
    for (const auto& coincidentPointId : coincidentPoints) {
        if (fixedPointIds.contains(coincidentPointId)) {
            return true;
        }
    }
    return false;
}

void DCMManager::addDragLocks(Utils::ID figureId,
                              std::initializer_list<double*> vars,
                              BatchUpdateContext& context) {
    if (_solveMode != Utils::SolveMode::DRAG) {
        return;
    }

    auto comp = getComponentForFigure(figureId);
    if (!comp.has_value()) {
        return;
    }

    auto& lockedVars = context.lockedVarsByComponent[comp.value()];
    for (double* var : vars) {
        if (var != nullptr) {
            lockedVars.insert(var);
        }
    }
}

void DCMManager::solveDragUpdates(const BatchUpdateContext& context) {
    if (_solveMode != Utils::SolveMode::DRAG) {
        return;
    }

    for (const auto& [componentId, lockedVars] : context.lockedVarsByComponent) {
        if (!lockedVars.empty()) {
            solveWithLockedVars(componentId, lockedVars);
        }
    }
}

void DCMManager::validatePointUpdate(const Utils::PointUpdateDescriptor& descriptor) const {
    if (_storage.get<Figures::Point2D>(descriptor.pointId) == nullptr) {
        throw std::runtime_error("Point not found");
    }
}

void DCMManager::validateLineUpdate(const Utils::LineUpdateDescriptor& descriptor) const {
    if (_storage.get<Figures::Line2D>(descriptor.lineId) == nullptr) {
        throw std::runtime_error("Line not found");
    }
    if (_storage.getDependencies(descriptor.lineId).size() != 2) {
        throw std::runtime_error("Line dependencies are inconsistent");
    }
}

void DCMManager::validateCircleUpdate(const Utils::CircleUpdateDescriptor& descriptor) const {
    if (_storage.get<Figures::Circle2D>(descriptor.circleId) == nullptr) {
        throw std::runtime_error("Circle not found");
    }
    if (descriptor.hasCenterUpdate() && _storage.getDependencies(descriptor.circleId).size() != 1) {
        throw std::runtime_error("Circle dependencies are inconsistent");
    }
}

void DCMManager::validateArcUpdate(const Utils::ArcUpdateDescriptor& descriptor) const {
    if (_storage.get<Figures::Arc2D>(descriptor.arcId) == nullptr) {
        throw std::runtime_error("Arc not found");
    }
    if (_storage.getDependencies(descriptor.arcId).size() != 3) {
        throw std::runtime_error("Arc dependencies are inconsistent");
    }
}

void DCMManager::validateFigureUpdate(const Utils::FigureUpdateDescriptor& descriptor) const {
    const auto storedType = _storage.getType(descriptor.figureId);
    if (!storedType.has_value()) {
        throw std::runtime_error("Figure not found");
    }
    if (storedType.value() != descriptor.type) {
        throw std::runtime_error("Figure type mismatch");
    }

    switch (descriptor.type) {
        case Utils::FigureType::ET_POINT2D:
            if (!descriptor.coords.empty() && descriptor.coords.size() != 2) {
                throw std::invalid_argument("Point update requires 2 coordinates");
            }
            validatePointUpdate({descriptor.figureId});
            break;
        case Utils::FigureType::ET_LINE:
            if (!descriptor.coords.empty() && descriptor.coords.size() != 4) {
                throw std::invalid_argument("Line update requires 4 coordinates");
            }
            validateLineUpdate({descriptor.figureId});
            break;
        case Utils::FigureType::ET_CIRCLE:
            if (!descriptor.coords.empty() && descriptor.coords.size() != 2) {
                throw std::invalid_argument("Circle update requires 2 center coordinates");
            }
            validateCircleUpdate({
                descriptor.figureId,
                descriptor.coords.size() == 2 ? std::optional<double>{descriptor.coords[0]} : std::nullopt,
                descriptor.coords.size() == 2 ? std::optional<double>{descriptor.coords[1]} : std::nullopt,
                descriptor.radius});
            break;
        case Utils::FigureType::ET_ARC:
            if (!descriptor.coords.empty() && descriptor.coords.size() != 6) {
                throw std::invalid_argument("Arc update requires 6 coordinates");
            }
            validateArcUpdate({descriptor.figureId});
            break;
    }
}

void DCMManager::applyPointUpdateNoSolve(const Utils::PointUpdateDescriptor& descriptor,
                                         const FixedGeometry& fixedGeometry,
                                         BatchUpdateContext& context) {
    validatePointUpdate(descriptor);

    const Utils::ID solvePointId = _reqSystem.resolvePointRepresentative(descriptor.pointId);
    auto* solvePoint = _storage.get<Figures::Point2D>(solvePointId);
    if (solvePoint == nullptr) {
        throw std::runtime_error("Point not found");
    }

    if (pointGroupHasFixConstraint(descriptor.pointId, fixedGeometry.pointIds)) {
        context.needsCoincidentSync = true;
        return;
    }

    if (descriptor.newX.has_value()) {
        solvePoint->x() = descriptor.newX.value();
    }
    if (descriptor.newY.has_value()) {
        solvePoint->y() = descriptor.newY.value();
    }

    context.needsCoincidentSync = true;
    addDragLocks(descriptor.pointId, {solvePoint->ptrX(), solvePoint->ptrY()}, context);
}

void DCMManager::applyLineUpdateNoSolve(const Utils::LineUpdateDescriptor& descriptor,
                                        const FixedGeometry& fixedGeometry,
                                        BatchUpdateContext& context) {
    validateLineUpdate(descriptor);

    const auto dependencies = _storage.getDependencies(descriptor.lineId);
    applyPointUpdateNoSolve(
        {dependencies[0], descriptor.newX1, descriptor.newY1},
        fixedGeometry,
        context);
    applyPointUpdateNoSolve(
        {dependencies[1], descriptor.newX2, descriptor.newY2},
        fixedGeometry,
        context);
}

void DCMManager::applyCircleUpdateNoSolve(const Utils::CircleUpdateDescriptor& descriptor,
                                          const FixedGeometry& fixedGeometry,
                                          BatchUpdateContext& context) {
    validateCircleUpdate(descriptor);

    auto* circle = _storage.get<Figures::Circle2D>(descriptor.circleId);
    if (circle == nullptr) {
        throw std::runtime_error("Circle not found");
    }

    if (descriptor.hasCenterUpdate()) {
        const auto dependencies = _storage.getDependencies(descriptor.circleId);
        applyPointUpdateNoSolve(
            {dependencies[0], descriptor.newCenterX, descriptor.newCenterY},
            fixedGeometry,
            context);
    }

    if (!descriptor.hasRadiusUpdate()) {
        return;
    }
    if (fixedGeometry.circleIds.contains(descriptor.circleId)) {
        return;
    }

    circle->radius = descriptor.newRadius;
    addDragLocks(descriptor.circleId, {circle->ptrRadius()}, context);
}

void DCMManager::applyArcUpdateNoSolve(const Utils::ArcUpdateDescriptor& descriptor,
                                       const FixedGeometry& fixedGeometry,
                                       BatchUpdateContext& context) {
    validateArcUpdate(descriptor);

    const auto dependencies = _storage.getDependencies(descriptor.arcId);
    applyPointUpdateNoSolve(
        {dependencies[0], descriptor.newX1, descriptor.newY1},
        fixedGeometry,
        context);
    applyPointUpdateNoSolve(
        {dependencies[1], descriptor.newX2, descriptor.newY2},
        fixedGeometry,
        context);
    applyPointUpdateNoSolve(
        {dependencies[2], descriptor.newCenterX, descriptor.newCenterY},
        fixedGeometry,
        context);
}

void DCMManager::applyFigureUpdateNoSolve(const Utils::FigureUpdateDescriptor& descriptor,
                                          const FixedGeometry& fixedGeometry,
                                          BatchUpdateContext& context) {
    validateFigureUpdate(descriptor);

    switch (descriptor.type) {
        case Utils::FigureType::ET_POINT2D:
            if (descriptor.coords.size() == 2) {
                applyPointUpdateNoSolve(
                    {descriptor.figureId, descriptor.coords[0], descriptor.coords[1]},
                    fixedGeometry,
                    context);
            } else {
                applyPointUpdateNoSolve({descriptor.figureId, descriptor.x, descriptor.y}, fixedGeometry, context);
            }
            break;
        case Utils::FigureType::ET_LINE:
            if (descriptor.coords.size() == 4) {
                applyLineUpdateNoSolve(
                    {descriptor.figureId,
                     descriptor.coords[0],
                     descriptor.coords[1],
                     descriptor.coords[2],
                     descriptor.coords[3]},
                    fixedGeometry,
                    context);
            } else {
                applyLineUpdateNoSolve({descriptor.figureId}, fixedGeometry, context);
            }
            break;
        case Utils::FigureType::ET_CIRCLE:
            applyCircleUpdateNoSolve(
                {descriptor.figureId,
                 descriptor.coords.size() == 2 ? std::optional<double>{descriptor.coords[0]} : std::nullopt,
                 descriptor.coords.size() == 2 ? std::optional<double>{descriptor.coords[1]} : std::nullopt,
                 descriptor.radius},
                fixedGeometry,
                context);
            break;
        case Utils::FigureType::ET_ARC:
            if (descriptor.coords.size() == 6) {
                applyArcUpdateNoSolve(
                    {descriptor.figureId,
                     descriptor.coords[0],
                     descriptor.coords[1],
                     descriptor.coords[2],
                     descriptor.coords[3],
                     descriptor.coords[4],
                     descriptor.coords[5]},
                    fixedGeometry,
                    context);
            } else {
                applyArcUpdateNoSolve({descriptor.figureId}, fixedGeometry, context);
            }
            break;
    }
}

void DCMManager::updatePoint(const Utils::PointUpdateDescriptor& descriptor) {
    updatePoints({descriptor});
}

void DCMManager::updatePoints(const std::vector<Utils::PointUpdateDescriptor>& descriptors) {
    for (const auto& descriptor : descriptors) {
        validatePointUpdate(descriptor);
    }

    syncRequirementSystemIfNeeded();
    const auto fixedGeometry = collectFixedGeometry();
    BatchUpdateContext context;
    for (const auto& descriptor : descriptors) {
        applyPointUpdateNoSolve(descriptor, fixedGeometry, context);
    }

    if (context.needsCoincidentSync) {
        _reqSystem.synchronizeCoincidentPoints();
    }
    solveDragUpdates(context);
}

void DCMManager::updateLine(const Utils::LineUpdateDescriptor& descriptor) {
    updateLines({descriptor});
}

void DCMManager::updateLines(const std::vector<Utils::LineUpdateDescriptor>& descriptors) {
    for (const auto& descriptor : descriptors) {
        validateLineUpdate(descriptor);
    }

    syncRequirementSystemIfNeeded();
    const auto fixedGeometry = collectFixedGeometry();
    BatchUpdateContext context;
    for (const auto& descriptor : descriptors) {
        applyLineUpdateNoSolve(descriptor, fixedGeometry, context);
    }

    if (context.needsCoincidentSync) {
        _reqSystem.synchronizeCoincidentPoints();
    }
    solveDragUpdates(context);
}

void DCMManager::updateCircle(const Utils::CircleUpdateDescriptor& descriptor) {
    updateCircles({descriptor});
}

void DCMManager::updateCircles(const std::vector<Utils::CircleUpdateDescriptor>& descriptors) {
    bool needsPointResolution = false;
    for (const auto& descriptor : descriptors) {
        validateCircleUpdate(descriptor);
        needsPointResolution = needsPointResolution || descriptor.hasCenterUpdate();
    }

    if (needsPointResolution) {
        syncRequirementSystemIfNeeded();
    }

    const auto fixedGeometry = collectFixedGeometry();
    BatchUpdateContext context;
    for (const auto& descriptor : descriptors) {
        applyCircleUpdateNoSolve(descriptor, fixedGeometry, context);
    }

    if (context.needsCoincidentSync) {
        _reqSystem.synchronizeCoincidentPoints();
    }
    solveDragUpdates(context);
}

void DCMManager::updateArc(const Utils::ArcUpdateDescriptor& descriptor) {
    updateArcs({descriptor});
}

void DCMManager::updateArcs(const std::vector<Utils::ArcUpdateDescriptor>& descriptors) {
    for (const auto& descriptor : descriptors) {
        validateArcUpdate(descriptor);
    }

    syncRequirementSystemIfNeeded();
    const auto fixedGeometry = collectFixedGeometry();
    BatchUpdateContext context;
    for (const auto& descriptor : descriptors) {
        applyArcUpdateNoSolve(descriptor, fixedGeometry, context);
    }

    if (context.needsCoincidentSync) {
        _reqSystem.synchronizeCoincidentPoints();
    }
    solveDragUpdates(context);
}

void DCMManager::updateFigure(const Utils::FigureUpdateDescriptor& descriptor) {
    updateFigures({descriptor});
}

void DCMManager::updateFigures(const std::vector<Utils::FigureUpdateDescriptor>& descriptors) {
    bool needsPointResolution = false;
    for (const auto& descriptor : descriptors) {
        validateFigureUpdate(descriptor);
        needsPointResolution = needsPointResolution ||
            descriptor.type == Utils::FigureType::ET_POINT2D ||
            descriptor.type == Utils::FigureType::ET_LINE ||
            descriptor.type == Utils::FigureType::ET_ARC ||
            (descriptor.type == Utils::FigureType::ET_CIRCLE && !descriptor.coords.empty());
    }

    if (needsPointResolution) {
        syncRequirementSystemIfNeeded();
    }

    const auto fixedGeometry = collectFixedGeometry();
    BatchUpdateContext context;
    for (const auto& descriptor : descriptors) {
        applyFigureUpdateNoSolve(descriptor, fixedGeometry, context);
    }

    if (context.needsCoincidentSync) {
        _reqSystem.synchronizeCoincidentPoints();
    }
    solveDragUpdates(context);
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
    invalidateSolveCache();

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
    invalidateSolveCache();
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
    invalidateSolveCache();
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
    invalidateSolveCache();
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

    if (_solveCache == nullptr) {
        _solveCache = std::make_unique<SolveCache>();
    }

    SolveCacheKey cacheKey;
    switch (_solveMode) {
        case Utils::SolveMode::GLOBAL:
            cacheKey.componentId = std::nullopt;
            break;
        case Utils::SolveMode::LOCAL:
            if (!componentId.has_value()) {
                throw std::runtime_error("LOCAL mode requires a componentID");
            }
            cacheKey.componentId = componentId;
            break;
        case Utils::SolveMode::DRAG:
            cacheKey.componentId = componentId;
            break;
    }

    cacheKey.lockedVars.assign(lockedVars.begin(), lockedVars.end());
    std::sort(cacheKey.lockedVars.begin(), cacheKey.lockedVars.end());

    const auto buildPipeline = [&](System::RequirementSystem& system) {
        BuiltSolvePipeline pipeline;
        std::vector<std::unique_ptr<::Function>> mathFunctionOwners;
        std::vector<double*> mathVariableRefs;
        std::unordered_set<double*> mathVariableRefSet;

        const auto rememberVariable = [&](double* valueRef) {
            if (valueRef != nullptr &&
                !lockedVars.contains(valueRef) &&
                !pipeline.fixedAssignments.contains(valueRef) &&
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
            return std::pair{system.resolvePoint(dependencies[0]), circle->ptrRadius()};
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
                    pipeline.fixedAssignments[point->ptrX()] = targets[0];
                    pipeline.fixedAssignments[point->ptrY()] = targets[1];
                    break;
                }
                case Utils::RequirementType::ET_FIXLINE: {
                    const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                    std::vector<double> targets = {lineP1->x(), lineP1->y(), lineP2->x(), lineP2->y()};
                    const auto targetIt = _fixedRequirementTargets.find(entry.id);
                    if (targetIt != _fixedRequirementTargets.end() && targetIt->second.size() == 4) {
                        targets = targetIt->second;
                    }
                    pipeline.fixedAssignments[lineP1->ptrX()] = targets[0];
                    pipeline.fixedAssignments[lineP1->ptrY()] = targets[1];
                    pipeline.fixedAssignments[lineP2->ptrX()] = targets[2];
                    pipeline.fixedAssignments[lineP2->ptrY()] = targets[3];
                    break;
                }
                case Utils::RequirementType::ET_FIXCIRCLE: {
                    const auto [center, radius] = resolveCircleData(ids[0]);
                    std::vector<double> targets = {center->x(), center->y(), *radius};
                    const auto targetIt = _fixedRequirementTargets.find(entry.id);
                    if (targetIt != _fixedRequirementTargets.end() && targetIt->second.size() == 3) {
                        targets = targetIt->second;
                    }
                    pipeline.fixedAssignments[center->ptrX()] = targets[0];
                    pipeline.fixedAssignments[center->ptrY()] = targets[1];
                    pipeline.fixedAssignments[radius] = targets[2];
                    break;
                }
                default:
                    break;
            }
        }

        for (const auto& [valueRef, target] : pipeline.fixedAssignments) {
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
                    break;
                case Utils::RequirementType::ET_LINECIRCLEDIST: {
                    const auto [lineP1, lineP2] = resolveLinePoints(ids[0]);
                    const auto [center, radius] = resolveCircleData(ids[1]);
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
                    const auto [center, radius] = resolveCircleData(ids[1]);
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
                    const auto [center, radius] = resolveCircleData(ids[1]);
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
                    const auto [center, radius] = resolveCircleData(ids[0]);
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

        pipeline.hasFunctions = !mathFunctionOwners.empty();
        if (!pipeline.hasFunctions) {
            return pipeline;
        }

        pipeline.hasFreeVariables = !mathVariableRefs.empty();
        if (!pipeline.hasFreeVariables) {
            return pipeline;
        }

        std::vector<::Function*> mathFuncs;
        mathFuncs.reserve(mathFunctionOwners.size());
        for (auto& owner : mathFunctionOwners) {
            mathFuncs.push_back(owner.release());
        }

        std::vector<Variable*> mathVars;
        pipeline.variableOwners.reserve(mathVariableRefs.size());
        mathVars.reserve(mathVariableRefs.size());
        for (double* valueRef : mathVariableRefs) {
            pipeline.variableOwners.push_back(std::make_unique<Variable>(valueRef));
            mathVars.push_back(pipeline.variableOwners.back().get());
        }

        pipeline.task = std::make_unique<SparseLSMTask>(std::move(mathFuncs), std::move(mathVars));
        return pipeline;
    };

    auto entryIt = _solveCache->entries.find(cacheKey);
    if (entryIt == _solveCache->entries.end() || entryIt->second.version != _solveCache->version) {
        SolveCache::Entry entry;
        entry.version = _solveCache->version;

        System::RequirementSystem* buildSystem = nullptr;
        if (cacheKey.componentId.has_value()) {
            entry.subsystem = buildSubsystem(cacheKey.componentId.value());
            buildSystem = entry.subsystem.get();
        } else {
            syncRequirementSystemIfNeeded();
            buildSystem = &_reqSystem;
        }

        auto pipeline = buildPipeline(*buildSystem);
        entry.fixedAssignments = std::move(pipeline.fixedAssignments);
        entry.variableOwners = std::move(pipeline.variableOwners);
        entry.task = std::move(pipeline.task);
        entry.hasFunctions = pipeline.hasFunctions;
        entry.hasFreeVariables = pipeline.hasFreeVariables;
        if (entry.task != nullptr) {
            entry.solver = std::make_unique<SparseLMSolver>();
        }

        entryIt = _solveCache->entries.insert_or_assign(std::move(cacheKey), std::move(entry)).first;
    }

    auto& entry = entryIt->second;
    System::RequirementSystem* systemPtr = nullptr;
    if (entry.subsystem != nullptr) {
        systemPtr = entry.subsystem.get();
    } else {
        syncRequirementSystemIfNeeded();
        systemPtr = &_reqSystem;
    }

    auto& system = *systemPtr;
    if (system.getRequirements().empty() || !entry.hasFunctions) {
        system.synchronizeCoincidentPoints();
        return true;
    }

    for (const auto& [valueRef, target] : entry.fixedAssignments) {
        *valueRef = target;
    }

    if (!entry.hasFreeVariables) {
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

    entry.solver->setTask(entry.task.get());
    entry.solver->optimize();
    const bool converged = entry.solver->isConverged();
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
