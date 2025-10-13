#ifndef HEADERS_GLOBALREQUIREMENTMANAGER_H
#define HEADERS_GLOBALREQUIREMENTMANAGER_H
#include "FigureData.h"
#include "Graph.h"
#include "IDGenerator.h"
#include "RequirementData.h"
#include "Requirements.h"
#include "ID.h"
#include "Components.h"
#include <unordered_map>
#include <unordered_set>

namespace OurPaintDCM {

/**
 * @brief Central manager for global geometric constraints in 2D sketches.
 *
 * This class handles storage, management, diagnostics, and solving of constraints.
 * It tracks:
 * - all variables (_vars),
 * - constraints (_reqs),
 * - requirements info (_reqsData),
 * - error functions (_errors) for numerical solvers,
 * - a dependency graph (_graph) for decomposition and DoF analysis.
 *
 * It supports multiple solving modes (GLOBAL / LOCAL / DRAG) and
 * diagnostics for under/over/well-constrained systems.
 */
class GlobalRequirementManager {
    Utils::IDGenerator                                              _idGenerator; ///< Generates unique IDs
    std::unordered_map<Utils::ID, Requirements::Requirement*>       _reqs;   ///< ID -> Requirement
    std::unordered_set<Utils::RequirementData>                      _reqsData;  ///< Constraints data
    Graph<Utils::ID, Utils::ID, UndirectedPolicy, WeightedPolicy>   _graph;  ///< Dependency graph<ID of figure, ID of requirement>.
    Requirements::Components                                        _components; ///< Include ErrorFunction by ID inside
    Utils::SolveMode                                                _mode;   ///< Current solving mode

public:
    /**
     * @brief Default constructor.
     *
     * Initializes an empty GlobalRequirementManager with:
     * - no variables,
     * - no requirements,
     * - no error functions,
     * - an empty dependency graph,
     * - solve mode set to GLOBAL.
     */
    GlobalRequirementManager();

    /**
     * @brief Destructor.
     *
     * Releases memory associated with requirements and error functions.
     * The exact ownership semantics depend on the project policy:
     *  GlobalRequirementManager owns Requirement* and ErrorFunction*, and they are deleted here.
     */
    ~GlobalRequirementManager();

    // ---------------- Requirements management ----------------


    /**
     * @brief Add a new geometric requirement into the manager.
     *
     * The method performs the following steps:
     * - Inserts the requirement into the internal map (_reqs) by its unique ID.
     * - Registers all involved variables into the global variable set (_vars).
     * - Creates and caches the associated error function in _errors,
     *   which will later be used by the numerical solver.
     * - Updates the dependency graph to reflect the relationship between
     *   figures and this requirement.
     *
     * @param req RequirementData object containing the requirement data.
     * @note Ownership: GlobalRequirementManager owns the requirement,
     *       and it will be deleted in the destructor.
     */
    void addRequirement(Utils::RequirementData req);

    /**
     * brief Remove an existing requirement by its ID.
     * Using linear search to find the requirement in _reqsData and delete it
     * @param id id of requirement to remove
     * @throws std::invalid_argument if the requirement with the given ID does not exist.
     */
    void removeRequirement(Utils::ID id);
    /**
     * @brief Remove all requirements from the manager.
     *
     * Clears all internal data structures, including:
     * - the requirements map (_reqs),
     * - the requirements data set (_reqsData),
     * - the dependency graph (_graph),
     * - the components and error functions (_components).
     *
     * After calling this method, the manager will be in an empty state,
     * ready to accept new requirements.
     */
    void removeAllRequirements();
    /**
     * @brief Retrieve a requirement by its ID.
     *
     * @param id The unique identifier of the requirement to retrieve.
     * @return The corresponding RequirementData object.
     * @throws std::invalid_argument if the requirement with the given ID does not exist.
     */
    Utils::RequirementData getRequirement(Utils::ID id) const;

    // ---------------- Solve modes ----------------
    void             setMode(Utils::SolveMode mode);
    inline Utils::SolveMode getMode() const;
    void             solve();

    // ---------------- Diagnostics ----------------
    inline bool isOverConstrained() const;
    inline bool isUnderConstrained() const;
    inline bool isWellConstrained() const;

private:
    void solveGlobal();
    void solveLocal();
    void solveDrag();

    int computeJacobianRank() const;
    int countVariablesDoF() const;
};
}

#endif //HEADERS_GLOBALREQUIREMENTMANAGER_H