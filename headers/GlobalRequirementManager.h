#ifndef HEADERS_GLOBALREQUIREMENTMANAGER_H
#define HEADERS_GLOBALREQUIREMENTMANAGER_H
#include "FigureData.h"
#include "RequirementData.h"
#include "Graph.h"
#include "Requirements.h"
#include "utils/ID.h"
#include <ErrorFunction.h>
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
 * - error functions (_errors) for numerical solvers,
 * - a dependency graph (_graph) for decomposition and DoF analysis.
 *
 * It supports multiple solving modes (GLOBAL / LOCAL / DRAG) and
 * diagnostics for under/over/well-constrained systems.
 */
class GlobalRequirementManager {
    std::unordered_set<Variable*> _vars; ///< All variables involved in constraints
    std::unordered_map<Utils::ID, Requirements::Requirement*> _reqs; ///< ID -> Requirement
    std::vector<ErrorFunction*> _errors; ///< Cached error functions for solvers
    Graph<Utils::FigureData, Utils::ID> _graph; ///< Dependency graph
    Utils::SolveMode _mode; ///< Current solving mode

public:
    GlobalRequirementManager();
    ~GlobalRequirementManager();

    // ---------------- Requirements management ----------------
    void addRequirement(Requirements::Requirement* req);
    void removeRequirement(Utils::ID id);
    void removeAllRequirements();
    Utils::RequirementData getRequirement(Utils::ID id) const;

    // ---------------- Solve modes ----------------
    void setMode(Utils::SolveMode mode);
    Utils::SolveMode getMode() const;
    void solve();

    // ---------------- Diagnostics ----------------
    bool isOverConstrained() const;
    bool isUnderConstrained() const;
    bool isWellConstrained() const;

private:
    void solveGlobal();
    void solveLocal();
    void solveDrag();

    int computeJacobianRank() const;
    int countVariablesDoF() const;
};
}

#endif //HEADERS_GLOBALREQUIREMENTMANAGER_H