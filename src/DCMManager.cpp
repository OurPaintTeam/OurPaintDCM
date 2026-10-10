#include "DCMManager.h"
#include "Function.h"
#include "SparseLSMTask.h"
#include "sparse/SparseLevenbergMarquardtSolver.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>

namespace {

void eraseRequirementId(std::vector<OurPaintDCM::Utils::ID>& ids, OurPaintDCM::Utils::ID id) {
    ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
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

class GeometrySolveState {
    struct Value {
        double* variable;
        double original;
        bool radius;
    };
    std::vector<Value> _values;
    bool _accepted = false;

public:
    void savePoint(OurPaintDCM::Figures::Point2D& point) {
        _values.push_back({point.ptrX(), point.x(), false});
        _values.push_back({point.ptrY(), point.y(), false});
    }
    void saveCircle(OurPaintDCM::Figures::Circle2D& circle) {
        _values.push_back({circle.ptrRadius(), circle.radius, true});
    }
    bool valid() const noexcept {
        for (const auto& value : _values) {
            if (!std::isfinite(*value.variable) || (value.radius && *value.variable <= 0.0)) {
                return false;
            }
        }
        return true;
    }
    void accept() noexcept { _accepted = true; }
    ~GeometrySolveState() {
        if (!_accepted) {
            for (const auto& value : _values) *value.variable = value.original;
        }
    }
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
        double solverResidualTolerance = -1.0;
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

DCMManager::Snapshot DCMManager::snapshot() const {
    Snapshot state;
    state._figures = getAllFigures();
    std::sort(state._figures.begin(), state._figures.end(), [](const auto& lhs, const auto& rhs) {
        const bool lhsIsPoint = lhs.type == Utils::FigureType::ET_POINT2D;
        const bool rhsIsPoint = rhs.type == Utils::FigureType::ET_POINT2D;
        if (lhsIsPoint != rhsIsPoint) {
            return lhsIsPoint;
        }
        return lhs.id.value().id < rhs.id.value().id;
    });

    for (auto& figure : state._figures) {
        if (figure.type != Utils::FigureType::ET_POINT2D) {
            figure.coords.clear();
        }
    }

    state._requirements = getAllRequirements();
    state._fixedRequirementTargets = _fixedRequirementTargets;
    state._nextFigureId = _storage.currentID();
    state._nextRequirementId = _reqSystem._reqIdGen.current();
    state._solveMode = _solveMode;
    return state;
}

void DCMManager::restoreSnapshot(const Snapshot& state) {
    for (const auto& figure : state._figures) figure.validate();
    for (const auto& requirement : state._requirements) requirement.validate();
    for (const auto& [id, targets] : state._fixedRequirementTargets) {
        Utils::requireFiniteValues(targets);
    }
    clear();

    for (const auto& figure : state._figures) {
        if (figure.type == Utils::FigureType::ET_POINT2D) {
            addFigure(figure);
        }
    }
    for (const auto& figure : state._figures) {
        if (figure.type != Utils::FigureType::ET_POINT2D) {
            addFigure(figure);
        }
    }
    for (const auto& requirement : state._requirements) {
        _requirementRecords.emplace(*requirement.id, requirement);
        _requirementOrder.push_back(*requirement.id);
    }

    _fixedRequirementTargets = state._fixedRequirementTargets;
    _storage.restoreNextID(state._nextFigureId);
    _reqSystem._reqIdGen.set(state._nextRequirementId);
    _solveMode = state._solveMode;
    rebuildRequirementSystem();
    rebuildComponents();
    invalidateSolveCache();
}

void DCMManager::invalidateSolveCache() noexcept {
    if (_solveCache == nullptr) {
        return;
    }
    ++_solveCache->version;
    _solveCache->entries.clear();
}

Utils::ID DCMManager::addFigure(const Utils::FigureDescriptor& descriptor) {
    descriptor.validate();

    if (descriptor.id.has_value()) {
        if (descriptor.id->id == 0ULL) {
            throw std::invalid_argument("Figure ID must not be 0");
        }
        if (_storage.contains(*descriptor.id)) {
            throw std::invalid_argument("Figure ID already exists");
        }
        if (descriptor.type != Utils::FigureType::ET_POINT2D && !descriptor.coords.empty()) {
            throw std::invalid_argument(
                "Figure with an explicit ID must reference existing points");
        }
    }

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
            figureId = _storage.createPoint(px, py, descriptor.id);
            break;
        }
        case Utils::FigureType::ET_LINE: {
            if (descriptor.coords.size() == 4) {
                const Utils::ID p1Id = _storage.createPoint(descriptor.coords[0], descriptor.coords[1]);
                const Utils::ID p2Id = _storage.createPoint(descriptor.coords[2], descriptor.coords[3]);
                registerPoint(p1Id, descriptor.coords[0], descriptor.coords[1]);
                registerPoint(p2Id, descriptor.coords[2], descriptor.coords[3]);
                auto lineOpt = _storage.createLine(p1Id, p2Id, descriptor.id);
                if (!lineOpt) {
                    throw std::runtime_error("Line creation failed");
                }
                figureId = *lineOpt;
                relatedFigures = {p1Id, p2Id};
                storedDesc.pointIds = relatedFigures;
            } else {
                auto lineOpt = _storage.createLine(
                    descriptor.pointIds[0], descriptor.pointIds[1], descriptor.id);
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
                auto circOpt = _storage.createCircle(centerId, descriptor.radius.value(), descriptor.id);
                if (!circOpt) {
                    throw std::runtime_error("Circle creation failed");
                }
                figureId = *circOpt;
                relatedFigures = {centerId};
                storedDesc.pointIds = relatedFigures;
            } else {
                auto circOpt = _storage.createCircle(
                    descriptor.pointIds[0], descriptor.radius.value(), descriptor.id);
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
                auto arcOpt = _storage.createArc(p1Id, p2Id, centerId, descriptor.id);
                if (!arcOpt) {
                    throw std::runtime_error("Arc creation failed");
                }
                figureId = *arcOpt;
                relatedFigures = {p1Id, p2Id, centerId};
                storedDesc.pointIds = relatedFigures;
            } else {
                auto arcOpt = _storage.createArc(
                    descriptor.pointIds[0], descriptor.pointIds[1], descriptor.pointIds[2], descriptor.id);
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

    _reqSystemSyncedWithRecords = false;
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

    invalidateSolveCache();
    static_cast<System::RequirementFunctionSystem&>(_reqSystem).clear();
    _reqSystemSyncedWithRecords = false;
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
        if (req.weight == 0) continue;
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
    Utils::requireFinite(descriptor.newX);
    Utils::requireFinite(descriptor.newY);
    if (_storage.get<Figures::Point2D>(descriptor.pointId) == nullptr) {
        throw std::runtime_error("Point not found");
    }
}

void DCMManager::validateLineUpdate(const Utils::LineUpdateDescriptor& descriptor) const {
    for (const auto& value : {descriptor.newX1, descriptor.newY1, descriptor.newX2, descriptor.newY2}) {
        Utils::requireFinite(value);
    }
    if (_storage.get<Figures::Line2D>(descriptor.lineId) == nullptr) {
        throw std::runtime_error("Line not found");
    }
    if (_storage.getDependencies(descriptor.lineId).size() != 2) {
        throw std::runtime_error("Line dependencies are inconsistent");
    }
}

void DCMManager::validateCircleUpdate(const Utils::CircleUpdateDescriptor& descriptor) const {
    Utils::requireFinite(descriptor.newCenterX);
    Utils::requireFinite(descriptor.newCenterY);
    Utils::requireNonNegative(descriptor.newRadius);
    if (descriptor.hasRadiusUpdate()) Utils::requirePositiveRadius(descriptor.newRadius);
    if (_storage.get<Figures::Circle2D>(descriptor.circleId) == nullptr) {
        throw std::runtime_error("Circle not found");
    }
    if (descriptor.hasCenterUpdate() && _storage.getDependencies(descriptor.circleId).size() != 1) {
        throw std::runtime_error("Circle dependencies are inconsistent");
    }
}

void DCMManager::validateArcUpdate(const Utils::ArcUpdateDescriptor& descriptor) const {
    for (const auto& value : {descriptor.newX1, descriptor.newY1, descriptor.newX2, descriptor.newY2,
                              descriptor.newCenterX, descriptor.newCenterY}) {
        Utils::requireFinite(value);
    }
    if (_storage.get<Figures::Arc2D>(descriptor.arcId) == nullptr) {
        throw std::runtime_error("Arc not found");
    }
    if (_storage.getDependencies(descriptor.arcId).size() != 3) {
        throw std::runtime_error("Arc dependencies are inconsistent");
    }
}

void DCMManager::validateFigureUpdate(const Utils::FigureUpdateDescriptor& descriptor) const {
    Utils::requireFiniteValues(descriptor.coords);
    Utils::requireFinite(descriptor.x);
    Utils::requireFinite(descriptor.y);
    if (descriptor.radius) Utils::requirePositiveRadius(*descriptor.radius);
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

    auto updated = it->second;
    updated.param = newParam;
    updated.validate();
    it->second.param = newParam;
    _reqSystemSyncedWithRecords = false;
    invalidateSolveCache();
}

void DCMManager::updateLineCircleTangencySide(Utils::ID reqId, Utils::TangencySide side) {
    const auto descriptor=getRequirement(reqId);
    if (!descriptor || descriptor->type != Utils::RequirementType::ET_LINECIRCLETANGENT)
        throw std::invalid_argument("Requirement is not a line/circle tangency");
    updateRequirementParam(reqId,static_cast<double>(side));
}

void DCMManager::updateCircleCircleTangencyKind(Utils::ID reqId, Utils::CircleTangencyKind kind) {
    const auto descriptor=getRequirement(reqId);
    if (!descriptor || descriptor->type != Utils::RequirementType::ET_CIRCLECIRCLETANGENT)
        throw std::invalid_argument("Requirement is not a circle/circle tangency");
    updateRequirementParam(reqId,static_cast<double>(kind));
}

void DCMManager::updateArcTangencyEndpoints(Utils::ID reqId, Utils::Endpoint first, Utils::Endpoint second) {
    auto it=_requirementRecords.find(reqId);
    if (it == _requirementRecords.end() ||
        (it->second.type != Utils::RequirementType::ET_ARCLINETANGENT && it->second.type != Utils::RequirementType::ET_ARCARCTANGENT))
        throw std::invalid_argument("Requirement is not an arc endpoint tangency");
    auto updated=it->second;
    updated.firstEndpoint=first; updated.secondEndpoint=second; updated.validate();
    it->second=std::move(updated);
    _reqSystemSyncedWithRecords=false;
    invalidateSolveCache();
}

void DCMManager::updateRequirementWeight(Utils::ID reqId, double newWeight) {
    auto it = _requirementRecords.find(reqId);
    if (it == _requirementRecords.end()) {
        throw std::runtime_error("Requirement not found");
    }
    auto updated = it->second;
    updated.weight = newWeight;
    updated.validate();
    it->second.weight = newWeight;
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
    invalidateSolveCache();
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

bool DCMManager::solve(std::optional<ComponentID> componentId, double residualTolerance) {
    const std::unordered_set<double*> noLockedVars;
    return solveWithLockedVars(componentId, noLockedVars, residualTolerance);
}

bool DCMManager::solveWithLockedVars(std::optional<ComponentID> componentId,
                                     const std::unordered_set<double*>& lockedVars,
                                     double residualTolerance) {
    if (!std::isfinite(residualTolerance) || residualTolerance < 0.0) {
        throw std::invalid_argument("Residual tolerance must be finite and non-negative");
    }
    SolveCacheKey cacheKey;
    // With no constraints, validate the whole document in every solve mode.
    // Empty LOCAL systems retain the existing behavior of accepting no component ID.
    if (!_requirementRecords.empty() || _storage.arcCount() != 0) {
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
    }

    GeometrySolveState geometryState;
    if (!cacheKey.componentId || !_reqSystemSyncedWithRecords) {
        for (const auto& ref : _storage.pointsWithIds()) {
            geometryState.savePoint(*_storage.get<Figures::Point2D>(ref.id));
        }
        for (const auto& ref : _storage.circlesWithIds()) {
            geometryState.saveCircle(*_storage.get<Figures::Circle2D>(ref.id));
        }
    } else {
        for (const auto id : getFiguresInComponent(*cacheKey.componentId)) {
            if (auto* point = _storage.get<Figures::Point2D>(id)) geometryState.savePoint(*point);
            if (auto* circle = _storage.get<Figures::Circle2D>(id)) geometryState.saveCircle(*circle);
        }
    }
    if (!geometryState.valid()) return false;
    if (_requirementRecords.empty() && _storage.arcCount() == 0) {
        geometryState.accept();
        return true;
    }

    if (_solveCache == nullptr) {
        _solveCache = std::make_unique<SolveCache>();
    }
    cacheKey.lockedVars.assign(lockedVars.begin(), lockedVars.end());
    std::sort(cacheKey.lockedVars.begin(), cacheKey.lockedVars.end());

    const auto buildPipeline = [&](System::RequirementSystem& system) {
        BuiltSolvePipeline pipeline;
        std::vector<std::unique_ptr<::Function>> mathFunctionOwners;
        std::vector<double*> mathVariableRefs;
        std::unordered_set<double*> mathVariableRefSet;

        const auto rememberVariable = [&](double* valueRef) {
            if (!lockedVars.contains(valueRef) &&
                !pipeline.fixedAssignments.contains(valueRef) &&
                mathVariableRefSet.insert(valueRef).second) mathVariableRefs.push_back(valueRef);
        };
        for (const auto& binding : system.getFunctions()) {
            double* variable = nullptr;
            double target = 0;
            if (binding->tryGetAssignment(variable,target)) pipeline.fixedAssignments[variable] = target;
        }

        for (const auto& [valueRef, target] : pipeline.fixedAssignments) {
            *valueRef = target;
        }

        for (const auto& constraint : system.getFunctions()) {
            if (constraint->getWeight() == 0) continue;
            const auto type = constraint->getType();
            if (type == Utils::RequirementType::ET_FIXPOINT ||
                type == Utils::RequirementType::ET_FIXLINE ||
                type == Utils::RequirementType::ET_FIXCIRCLE) {
                continue;
            }
            for (double* variable : constraint->getVars()) {
                rememberVariable(variable);
            }
            mathFunctionOwners.push_back(std::unique_ptr<::Function>(constraint->mathematical()->weightedFunction()));
        }

        pipeline.hasFunctions = !mathFunctionOwners.empty();
        if (!pipeline.hasFunctions) {
            return pipeline;
        }

        pipeline.hasFreeVariables = !mathVariableRefs.empty();
        if (!pipeline.hasFreeVariables) {
            return pipeline;
        }

        std::vector<Variable*> mathVars;
        pipeline.variableOwners.reserve(mathVariableRefs.size());
        mathVars.reserve(mathVariableRefs.size());
        for (double* valueRef : mathVariableRefs) {
            pipeline.variableOwners.push_back(std::make_unique<Variable>(valueRef));
            mathVars.push_back(pipeline.variableOwners.back().get());
        }

        std::vector<::Function*> mathFuncs;
        mathFuncs.reserve(mathFunctionOwners.size());
        for (auto& owner : mathFunctionOwners) {
            mathFuncs.push_back(owner.release());
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
    std::vector<std::size_t> invalidEvaluationCounts;
    for (const auto& binding : system.getFunctions())
        invalidEvaluationCounts.push_back(binding->mathematical()->invalidEvaluations());
    const auto constraintsSatisfied = [&]() {
        for (const auto& binding : system.getFunctions()) {
            if (!binding->mathematical()->satisfied(residualTolerance)) return false;
        }
        // Explicit coincidences and arc contacts are eliminated by point aliasing.
        return system.coincidencesSatisfied(residualTolerance);
    };

    const auto finishSolve = [&]() {
        if (!geometryState.valid()) return false;
        const bool solved = constraintsSatisfied();
        if (!geometryState.valid()) return false;
        if (!solved) {
            for (std::size_t i = 0; i < system.getFunctions().size(); ++i)
                if (system.getFunctions()[i]->mathematical()->invalidEvaluations() != invalidEvaluationCounts[i]) return false;
        }
        geometryState.accept();
        return solved;
    };

    for (const auto& [valueRef, target] : entry.fixedAssignments) {
        *valueRef = target;
    }

    // Undefined geometry has no usable linearization. Reject it before sending
    // infinite residuals into LM, including when fixes or aliases collapse a line.
    for (const auto& function : system.getFunctions()) {
        if (!std::isfinite(function->mathematical()->weightedValue())) {
            return false;
        }
    }

    if (!entry.hasFunctions) {
        system.synchronizeCoincidentPoints();
        return finishSolve();
    }

    if (!entry.hasFreeVariables) {
        // If temporary drag locks consume all remaining DOF, retry without locks.
        // This keeps fixed/eliminated vars constant, but allows the solver
        // to satisfy constraints by moving the dragged point to a feasible position.
        if (!lockedVars.empty()) {
            const std::unordered_set<double*> noLockedVars;
            const bool solved = solveWithLockedVars(componentId, noLockedVars, residualTolerance);
            if (!geometryState.valid()) return false;
            geometryState.accept();
            return solved;
        }
        system.synchronizeCoincidentPoints();
        return finishSolve();
    }

    if (entry.solver == nullptr || entry.solverResidualTolerance != residualTolerance) {
        const double tolerance = std::min(residualTolerance, 1e-4);
        entry.solver = std::make_unique<SparseLMSolver>(
            100, 1e-3, std::min(1e-8, tolerance * 0.01),
            std::min(1e-8, tolerance * 0.01), tolerance * tolerance * 0.01);
        entry.solverResidualTolerance = residualTolerance;
    }
    entry.solver->setTask(entry.task.get());
    entry.solver->optimize();
    system.synchronizeCoincidentPoints();

    return finishSolve();
}

std::unique_ptr<System::RequirementSystem> DCMManager::buildSubsystem(ComponentID componentId) const {
    auto subsystem = std::make_unique<System::RequirementSystem>(
        &const_cast<DCMManager*>(this)->_storage, getFiguresInComponent(componentId));

    std::vector<Utils::RequirementDescriptor> descriptors;
    auto reqIds = getRequirementsInComponent(componentId);
    for (const auto& reqId : reqIds) {
        auto it = _requirementRecords.find(reqId);
        if (it != _requirementRecords.end()) {
            descriptors.push_back(it->second);
        }
    }

    subsystem->replaceRequirements(descriptors, _reqSystem._reqIdGen.current(), _fixedRequirementTargets);
    return subsystem;
}

void DCMManager::rebuildRequirementSystem() {
    std::vector<Utils::RequirementDescriptor> descriptors;
    descriptors.reserve(_requirementOrder.size());
    for (const auto& reqId : _requirementOrder) {
        const auto it = _requirementRecords.find(reqId);
        if (it != _requirementRecords.end()) {
            descriptors.push_back(it->second);
        }
    }
    _reqSystem.replaceRequirements(descriptors, _reqSystem._reqIdGen.current(), _fixedRequirementTargets);
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

    // Geometry links remain even when no requirement connects the objects
    for (const auto& entry : _figureRecords) {
        auto relatedFigures = _storage.getDependencies(entry.first);
        if (!relatedFigures.empty()) {
            relatedFigures.push_back(entry.first);
            mergeComponents(relatedFigures);
        }
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
