#include "DCMManager.h"
#include <stdexcept>

namespace {

class RequirementDerivativeAdapter;

class RequirementFunctionAdapter : public Function {
    std::shared_ptr<OurPaintDCM::Function::RequirementFunction> _req;
public:
    explicit RequirementFunctionAdapter(std::shared_ptr<OurPaintDCM::Function::RequirementFunction> req)
        : _req(std::move(req)) {}
    double evaluate() const override { return _req->evaluate(); }
    ::Function* derivative(Variable* var) const override;
    ::Function* clone() const override { return new RequirementFunctionAdapter(_req); }
    std::string to_string() const override { return "ReqFunc"; }
};

class RequirementDerivativeAdapter : public Function {
    std::shared_ptr<OurPaintDCM::Function::RequirementFunction> _req;
    double* _var;
public:
    RequirementDerivativeAdapter(std::shared_ptr<OurPaintDCM::Function::RequirementFunction> req, double* var)
        : _req(std::move(req)), _var(var) {}
    double evaluate() const override {
        auto grad = _req->gradient();
        auto it = grad.find(_var);
        return (it != grad.end()) ? it->second : 0.0;
    }
    ::Function* derivative(Variable*) const override { return new Constant(0.0); }
    ::Function* clone() const override { return new RequirementDerivativeAdapter(_req, _var); }
    std::string to_string() const override { return "ReqDeriv"; }
};

::Function* RequirementFunctionAdapter::derivative(Variable* var) const {
    return new RequirementDerivativeAdapter(_req, var->value);
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

    if (forceCascade) {
        auto reqs = getRequirementsForFigure(figureId);
        for (const auto& reqId : reqs) {
            removeRequirement(reqId);
        }
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

    pruneRequirementsWithMissingObjects();
    splitComponentsAfterRemoval();
}

void DCMManager::updatePoint(const Utils::PointUpdateDescriptor& descriptor) {
    auto* point = _storage.get<Figures::Point2D>(descriptor.pointId);
    if (!point) {
        throw std::runtime_error("Point not found");
    }

    auto pointHasFixConstraint = [&]() {
        for (const auto& [_, req] : _requirementRecords) {
            switch (req.type) {
                case Utils::RequirementType::ET_FIXPOINT: {
                    if (!req.objectIds.empty() && req.objectIds[0] == descriptor.pointId) {
                        return true;
                    }
                    break;
                }
                case Utils::RequirementType::ET_FIXLINE: {
                    if (req.objectIds.empty()) {
                        break;
                    }
                    const auto* line = _storage.get<Figures::Line2D>(req.objectIds[0]);
                    if (line != nullptr && (line->p1 == point || line->p2 == point)) {
                        return true;
                    }
                    break;
                }
                case Utils::RequirementType::ET_FIXCIRCLE: {
                    if (req.objectIds.empty()) {
                        break;
                    }
                    const auto* circle = _storage.get<Figures::Circle2D>(req.objectIds[0]);
                    if (circle != nullptr && circle->center == point) {
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

    if (pointHasFixConstraint()) {
        return;
    }

    if (descriptor.newX.has_value()) {
        point->x() = descriptor.newX.value();
    }
    if (descriptor.newY.has_value()) {
        point->y() = descriptor.newY.value();
    }

    if (_solveMode == Utils::SolveMode::DRAG) {
        std::unordered_set<double*> lockedVars;
        lockedVars = {point->ptrX(), point->ptrY()};
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

    Utils::ID reqId = _reqSystem.addRequirement(descriptor);

    Utils::RequirementDescriptor storedDesc = descriptor;
    storedDesc.id = reqId;
    _requirementRecords[reqId] = storedDesc;

    mergeComponents(descriptor.objectIds);

    return reqId;
}

void DCMManager::removeRequirement(Utils::ID reqId) {
    auto it = _requirementRecords.find(reqId);
    if (it == _requirementRecords.end()) {
        throw std::runtime_error("Requirement not found");
    }

    _requirementRecords.erase(it);
    rebuildRequirementSystem();
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
    rebuildRequirementSystem();
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
    for (const auto& entry : _requirementRecords) {
        result.push_back(entry.second);
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

    for (const auto& [reqId, desc] : _requirementRecords) {
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

const System::RequirementSystem& DCMManager::getRequirementSystem() const noexcept {
    return _reqSystem;
}

Figures::GeometryStorage& DCMManager::storage() noexcept {
    return _storage;
}

System::RequirementSystem& DCMManager::requirementSystem() noexcept {
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
    _figureRecords.clear();
    _figureToComponent.clear();
    _components.clear();
    _nextComponentId = 0;
    _activeComponentCount = 0;
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

    System::RequirementFunctionSystem* systemPtr = nullptr;
    std::unique_ptr<System::RequirementSystem> subsystem;

    switch (_solveMode) {
        case Utils::SolveMode::GLOBAL:
            rebuildRequirementSystem();
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
                rebuildRequirementSystem();
                systemPtr = &_reqSystem;
            }
            break;
    }

    auto& system = *systemPtr;
    const auto& reqFuncs = system.getFunctions();
    auto allVars = system.getAllVars();
    if (reqFuncs.empty() || allVars.empty()) return true;

    // Preprocess algebraic assignments (x = c): eliminate variable and drop this residual.
    std::unordered_map<double*, double> eliminatedAssignments;
    std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>> activeReqFuncs;
    activeReqFuncs.reserve(reqFuncs.size());
    for (const auto& rf : reqFuncs) {
        double* assignedVar = nullptr;
        double assignedValue = 0.0;
        if (rf->tryGetAssignment(assignedVar, assignedValue)) {
            eliminatedAssignments[assignedVar] = assignedValue;
            *assignedVar = assignedValue;
            continue;
        }
        activeReqFuncs.push_back(rf);
    }

    if (activeReqFuncs.empty()) {
        return true;
    }

    std::vector<Variable*> mathVars;
    mathVars.reserve(allVars.size());
    if (lockedVars.empty()) {
        for (auto* v : allVars) {
            if (eliminatedAssignments.contains(v)) {
                continue;
            }
            mathVars.push_back(new Variable(v));
        }
    } else {
        for (auto* v : allVars) {
            if (!lockedVars.contains(v) && !eliminatedAssignments.contains(v)) {
                mathVars.push_back(new Variable(v));
            }
        }
    }
    if (mathVars.empty()) {
        // If temporary drag locks consume all remaining DOF, retry without locks.
        // This keeps fixed/eliminated vars constant, but allows the solver
        // to satisfy constraints by moving the dragged point to a feasible position.
        if (!lockedVars.empty()) {
            const std::unordered_set<double*> noLockedVars;
            return solveWithLockedVars(componentId, noLockedVars);
        }
        return true;
    }

    std::vector<::Function*> mathFuncs;
    mathFuncs.reserve(activeReqFuncs.size());
    for (auto& rf : activeReqFuncs)
        mathFuncs.push_back(new RequirementFunctionAdapter(rf));

    LSMFORLMTask task(mathFuncs, mathVars);
    LMSparse solver;
    solver.setTask(&task);
    solver.optimize();
    const bool converged = solver.isConverged();
    for (auto* f : mathFuncs) {
        delete f;
    }
    // LSMFORLMTask does not own m_functions; only c_function and Jacobian entries are freed in ~LSMFORLMTask.

    for (auto* v : mathVars) {
        delete v;
    }
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
    for (const auto& [id, desc] : _requirementRecords) {
        _reqSystem.addRequirement(desc);
    }
}

void DCMManager::pruneRequirementsWithMissingObjects() {
    bool changed = false;
    for (auto it = _requirementRecords.begin(); it != _requirementRecords.end();) {
        bool stale = false;
        for (const auto& oid : it->second.objectIds) {
            if (!_storage.contains(oid)) {
                stale = true;
                break;
            }
        }
        if (stale) {
            it = _requirementRecords.erase(it);
            changed = true;
        } else {
            ++it;
        }
    }
    if (changed) {
        rebuildRequirementSystem();
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
    for (const auto& [reqId, desc] : _requirementRecords) {
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