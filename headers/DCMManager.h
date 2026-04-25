#ifndef OURPAINTDCM_HEADERS_DCMMANAGER_H
#define OURPAINTDCM_HEADERS_DCMMANAGER_H

#include "GeometryStorage.h"
#include "RequirementSystem.h"
#include "RequirementDescriptor.h"
#include "FigureDescriptor.h"
#include "Graph.h"
#include "Function.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <cmath>
#include <cstdint>
#include <initializer_list>

namespace OurPaintDCM {

using ComponentID = std::size_t;
using ComponentGraph = Graph<Utils::ID, Utils::ID, UndirectedPolicy, WeightedPolicy>;

enum class RequirementGraphVertexKind : std::uint8_t {
    Figure,
    Requirement,
};

struct RequirementGraphVertex {
    RequirementGraphVertexKind kind{};
    Utils::ID id{};

    bool operator==(const RequirementGraphVertex& other) const noexcept {
        return kind == other.kind && id == other.id;
    }
};

} // namespace OurPaintDCM

namespace std {
template <>
struct hash<OurPaintDCM::RequirementGraphVertex> {
    std::size_t operator()(const OurPaintDCM::RequirementGraphVertex& vertex) const noexcept {
        const auto kind = static_cast<std::uint8_t>(vertex.kind);
        std::size_t seed = std::hash<std::uint8_t>{}(kind);
        seed ^= std::hash<OurPaintDCM::Utils::ID>{}(vertex.id) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
        return seed;
    }
};
}

namespace OurPaintDCM {

using RequirementDependencyGraph =
    Graph<RequirementGraphVertex, Utils::ID, UndirectedPolicy, WeightedPolicy>;

/**
 * @brief Main manager class for the DCM (Dynamic Constraint Manager) system.
 *
 * Provides unified CRUD operations for geometric figures and requirements.
 * Manages connected components of the constraint graph.
 *
 * Change geometry through addFigure / removeFigure / update*; inspect it via getStorage().
 * Mutable storage() is an escape hatch only; the caller bears all consequences (see that method’s docs).
 */
class DCMManager {
public:
    /**
     * @brief Default constructor.
     */
    DCMManager();

    ~DCMManager();

    DCMManager(const DCMManager&) = delete;
    DCMManager& operator=(const DCMManager&) = delete;
    DCMManager(DCMManager&&) noexcept = default;
    DCMManager& operator=(DCMManager&&) noexcept = default;

    /**
     * @brief Add a new geometric figure using descriptor.
     * @param descriptor FigureDescriptor containing figure data.
     * @return ID of the created figure.
     * @throws std::invalid_argument if descriptor validation fails.
     */
    Utils::ID addFigure(const Utils::FigureDescriptor& descriptor);

    /**
     * @brief Remove a figure by ID.
     * @param figureId ID of the figure to remove.
     * @param forceCascade If true, removes dependent figures and requirements.
     * @throws std::runtime_error if figure not found or has dependencies.
     */
    void removeFigure(Utils::ID figureId, bool forceCascade = false);

    /**
     * @brief Update point coordinates.
     * @param descriptor PointUpdateDescriptor with new values.
     * @throws std::runtime_error if point not found.
     */
    void updatePoint(const Utils::PointUpdateDescriptor& descriptor);

    /**
     * @brief Update multiple point coordinates in one batch.
     *
     * In DRAG mode all changed point representatives are locked, and each touched
     * component is solved once after the whole batch is applied.
     *
     * @param descriptors Point updates.
     * @throws std::runtime_error if any point is not found.
     */
    void updatePoints(const std::vector<Utils::PointUpdateDescriptor>& descriptors);

    /**
     * @brief Update line endpoint coordinates.
     * @param descriptor LineUpdateDescriptor with endpoint values.
     * @throws std::runtime_error if line not found.
     */
    void updateLine(const Utils::LineUpdateDescriptor& descriptor);

    /**
     * @brief Update multiple lines in one batch.
     * @param descriptors Line updates.
     * @throws std::runtime_error if any line is not found.
     */
    void updateLines(const std::vector<Utils::LineUpdateDescriptor>& descriptors);

    /**
     * @brief Update circle center and/or radius.
     * @param descriptor CircleUpdateDescriptor with new values.
     * @throws std::runtime_error if circle not found.
     */
    void updateCircle(const Utils::CircleUpdateDescriptor& descriptor);

    /**
     * @brief Update multiple circles in one batch.
     * @param descriptors Circle updates.
     * @throws std::runtime_error if any circle is not found.
     */
    void updateCircles(const std::vector<Utils::CircleUpdateDescriptor>& descriptors);

    /**
     * @brief Update arc endpoint and center coordinates.
     * @param descriptor ArcUpdateDescriptor with point values.
     * @throws std::runtime_error if arc not found.
     */
    void updateArc(const Utils::ArcUpdateDescriptor& descriptor);

    /**
     * @brief Update multiple arcs in one batch.
     * @param descriptors Arc updates.
     * @throws std::runtime_error if any arc is not found.
     */
    void updateArcs(const std::vector<Utils::ArcUpdateDescriptor>& descriptors);

    /**
     * @brief Update any supported figure by unified update descriptor.
     * @param descriptor Figure update.
     * @throws std::runtime_error if figure not found or type does not match.
     */
    void updateFigure(const Utils::FigureUpdateDescriptor& descriptor);

    /**
     * @brief Update multiple figures of arbitrary types in one batch.
     * @param descriptors Figure updates.
     * @throws std::runtime_error if any figure is not found or type does not match.
     */
    void updateFigures(const std::vector<Utils::FigureUpdateDescriptor>& descriptors);

    /**
     * @brief Get figure descriptor with current state.
     * @param figureId ID of the figure.
     * @return Optional containing FigureDescriptor or nullopt if not found.
     */
    std::optional<Utils::FigureDescriptor> getFigure(Utils::ID figureId) const;

    /**
     * @brief Check if figure exists.
     * @param figureId ID to check.
     * @return true if figure exists.
     */
    bool hasFigure(Utils::ID figureId) const noexcept;

    /**
     * @brief Get all figure descriptors.
     * @return Vector of FigureDescriptors for all figures.
     */
    std::vector<Utils::FigureDescriptor> getAllFigures() const;

    /**
     * @brief Get all point descriptors.
     * @return Vector of FigureDescriptors for all points.
     */
    std::vector<Utils::FigureDescriptor> getAllPoints() const;

    /**
     * @brief Get all line descriptors.
     * @return Vector of FigureDescriptors for all lines.
     */
    std::vector<Utils::FigureDescriptor> getAllLines() const;

    /**
     * @brief Get all circle descriptors.
     * @return Vector of FigureDescriptors for all circles.
     */
    std::vector<Utils::FigureDescriptor> getAllCircles() const;

    /**
     * @brief Get all arc descriptors.
     * @return Vector of FigureDescriptors for all arcs.
     */
    std::vector<Utils::FigureDescriptor> getAllArcs() const;

    /**
     * @brief Add a new requirement using descriptor.
     *
     * If descriptor.id is set, that ID is used (must be unique and non-zero).
     * Otherwise a new ID is allocated. The function layer is updated incrementally;
     * full rebuild from records happens only after remove/param change/prune or on first access while stale.
     *
     * @param descriptor RequirementDescriptor containing requirement data.
     * @return ID of the requirement (assigned or generated).
     * @throws std::invalid_argument if descriptor validation fails, id is zero, or id already exists.
     */
    Utils::ID addRequirement(const Utils::RequirementDescriptor& descriptor);

    /**
     * @brief Remove a requirement by ID.
     * @param reqId ID of the requirement to remove.
     * @throws std::runtime_error if requirement not found.
     */
    void removeRequirement(Utils::ID reqId);

    /**
     * @brief Update requirement parameter value.
     * @param reqId ID of the requirement.
     * @param newParam New parameter value.
     * @throws std::runtime_error if requirement not found or has no parameter.
     */
    void updateRequirementParam(Utils::ID reqId, double newParam);

    /**
     * @brief Get requirement descriptor by ID.
     * @param reqId ID of the requirement.
     * @return Optional containing RequirementDescriptor or nullopt if not found.
     */
    std::optional<Utils::RequirementDescriptor> getRequirement(Utils::ID reqId) const noexcept;

    /**
     * @brief Check if requirement exists.
     * @param reqId ID to check.
     * @return true if requirement exists.
     */
    bool hasRequirement(Utils::ID reqId) const noexcept;

    /**
     * @brief Get all requirement descriptors.
     * @return Vector of RequirementDescriptors for all requirements.
     */
    std::vector<Utils::RequirementDescriptor> getAllRequirements() const;

    /**
     * @brief Get requirement IDs that directly reference a figure.
     * @return IDs in requirement insertion order, or empty if the figure is unknown/unconstrained.
     */
    std::vector<Utils::ID> getFiguresRequirements(Utils::ID figureId) const;

    /**
     * @brief Get unique requirement IDs that directly reference any figure in @p figureIds.
     * @return IDs in first-seen graph order, without duplicates.
     */
    std::vector<Utils::ID> getFiguresRequirements(const std::vector<Utils::ID>& figureIds) const;

    /**
     * @brief Get total number of connected components.
     * @return Number of connected components.
     */
    std::size_t getComponentCount() const noexcept;

    /**
     * @brief Get component ID for a given figure.
     * @param figureId ID of the figure.
     * @return Optional containing component ID or nullopt if figure not found.
     */
    std::optional<ComponentID> getComponentForFigure(Utils::ID figureId) const noexcept;

    /**
     * @brief Get all figure IDs in a component.
     * @param componentId ID of the component.
     * @return Vector of figure IDs in the component.
     */
    std::vector<Utils::ID> getFiguresInComponent(ComponentID componentId) const;

    /**
     * @brief Get all requirement IDs in a component.
     * @param componentId ID of the component.
     * @return Vector of requirement IDs affecting the component.
     */
    std::vector<Utils::ID> getRequirementsInComponent(ComponentID componentId) const;

    /**
     * @brief Get all connected components.
     * @return Vector of vectors containing figure IDs for each component.
     */
    std::vector<std::vector<Utils::ID>> getAllComponents() const;

    /**
     * @brief Read-only access to GeometryStorage (preferred, safe path).
     * @return Const reference; geometry cannot be modified through this accessor.
     */
    const Figures::GeometryStorage& getStorage() const noexcept;

    /**
     * @brief Get read-only access to RequirementSystem.
     * @return Const reference to internal RequirementSystem.
     */
    const System::RequirementSystem& getRequirementSystem() const;

    /**
     * @brief Mutable GeometryStorage access — use only when absolutely necessary.
     *
     * @warning Normal code should use addFigure, removeFigure, updatePoint, updateCircle, and getStorage().
     * Direct mutation bypasses manager invariants: _figureRecords, connected components, and RequirementSystem
     * consistency. Restoring a coherent state is entirely the caller's responsibility. Any risk of broken solvers,
     * stale IDs, or undefined behaviour is solely on the code that calls this method.
     */
    Figures::GeometryStorage& storage() noexcept;

    /**
     * @brief Get mutable access to RequirementSystem.
     * @return Reference to internal RequirementSystem.
     */
    System::RequirementSystem& requirementSystem();

    /**
     * @brief Number of objects in GeometryStorage (getStorage().size()).
     *
     * Tracks the canonical geometry set; keeps agreement with hasFigure / removeFigure when records and storage stay in sync.
     */
    std::size_t figureCount() const noexcept;

    /**
     * @brief Get total requirement count.
     * @return Number of requirements.
     */
    std::size_t requirementCount() const noexcept;

    /**
     * @brief Clear all figures, requirements, and components.
     */
    void clear();

    /**
     * @brief Switch current solving mode.
     * @param mode New solving mode.
     */
    void setSolveMode(Utils::SolveMode mode) noexcept;

    /// @brief Get current solving mode.
    Utils::SolveMode getSolveMode() const noexcept;

    /**
     * @brief Solve the constraint system according to the current mode.
     *
     * GLOBAL — solves all requirements at once (Levenberg-Marquardt).
     * LOCAL  — solves only the specified component (Levenberg-Marquardt).
     * DRAG   — lightweight gradient descent, intended to be called from updatePoint/updateCircle.
     *
     * @param componentId Component to solve (used only in LOCAL mode).
     * @return true if the solver converged.
     */
    bool solve(std::optional<ComponentID> componentId = std::nullopt);

private:
    struct SolveCache;
    struct BatchUpdateContext;
    struct FixedGeometry;

    bool solveWithLockedVars(std::optional<ComponentID> componentId,
                             const std::unordered_set<double*>& lockedVars);
    void invalidateSolveCache() noexcept;

    Figures::GeometryStorage _storage;
    System::RequirementSystem _reqSystem;
    RequirementDependencyGraph _requirementGraph;

    std::unordered_map<Utils::ID, Utils::FigureDescriptor> _figureRecords;
    std::unordered_map<Utils::ID, Utils::RequirementDescriptor> _requirementRecords;
    std::unordered_map<Utils::ID, std::vector<double>> _fixedRequirementTargets;
    std::vector<Utils::ID> _requirementOrder;
    std::unordered_map<Utils::ID, ComponentID> _figureToComponent;
    std::vector<std::unordered_set<Utils::ID>> _components;
    ComponentID _nextComponentId = 0;
    std::size_t _activeComponentCount = 0;
    Utils::SolveMode _solveMode = Utils::SolveMode::GLOBAL;
    std::unique_ptr<SolveCache> _solveCache;

    std::unique_ptr<System::RequirementSystem> buildSubsystem(ComponentID componentId) const;

    FixedGeometry collectFixedGeometry() const;
    bool pointGroupHasFixConstraint(
        Utils::ID pointId,
        const std::unordered_set<Utils::ID>& fixedPointIds) const;
    void addDragLocks(Utils::ID figureId,
                      std::initializer_list<double*> vars,
                      BatchUpdateContext& context);
    void solveDragUpdates(const BatchUpdateContext& context);
    void applyPointUpdateNoSolve(const Utils::PointUpdateDescriptor& descriptor,
                                 const FixedGeometry& fixedGeometry,
                                 BatchUpdateContext& context);
    void applyLineUpdateNoSolve(const Utils::LineUpdateDescriptor& descriptor,
                                const FixedGeometry& fixedGeometry,
                                BatchUpdateContext& context);
    void applyCircleUpdateNoSolve(const Utils::CircleUpdateDescriptor& descriptor,
                                  const FixedGeometry& fixedGeometry,
                                  BatchUpdateContext& context);
    void applyArcUpdateNoSolve(const Utils::ArcUpdateDescriptor& descriptor,
                               const FixedGeometry& fixedGeometry,
                               BatchUpdateContext& context);
    void applyFigureUpdateNoSolve(const Utils::FigureUpdateDescriptor& descriptor,
                                  const FixedGeometry& fixedGeometry,
                                  BatchUpdateContext& context);
    void validatePointUpdate(const Utils::PointUpdateDescriptor& descriptor) const;
    void validateLineUpdate(const Utils::LineUpdateDescriptor& descriptor) const;
    void validateCircleUpdate(const Utils::CircleUpdateDescriptor& descriptor) const;
    void validateArcUpdate(const Utils::ArcUpdateDescriptor& descriptor) const;
    void validateFigureUpdate(const Utils::FigureUpdateDescriptor& descriptor) const;

    /// Rebuild _reqSystem from _requirementRecords and mark it in sync.
    void rebuildRequirementSystem();
    /// If records and _reqSystem differ, perform rebuildRequirementSystem().
    void syncRequirementSystemIfNeeded();

    bool _reqSystemSyncedWithRecords = true;
    /** Drops requirement records whose objectIds reference IDs no longer in GeometryStorage; rebuilds the system if needed. */
    void pruneRequirementsWithMissingObjects();
    void rebuildComponents();
    void linkRequirementInGraph(Utils::ID reqId, const std::vector<Utils::ID>& figureIds);
    void unlinkRequirementFromGraph(Utils::ID reqId);
    void unlinkFigureFromRequirementGraph(Utils::ID figureId);
    void mergeComponents(const std::vector<Utils::ID>& figureIds);
    void splitComponentsAfterRemoval();
    ComponentID createNewComponent();
    void addFigureToComponent(Utils::ID figureId, ComponentID componentId);
    void removeFigureFromComponent(Utils::ID figureId);
    std::vector<Utils::ID> getRequirementsForFigure(Utils::ID figureId) const;
};

} // namespace OurPaintDCM

#endif // OURPAINTDCM_HEADERS_DCMMANAGER_H
