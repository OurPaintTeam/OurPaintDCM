#include "ConstraintSolveOrder.h"
#include "RequirementSystem.h"
#include <algorithm>
#include <tuple>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace OurPaintDCM::System {
namespace {

using Incidence = std::unordered_map<Utils::ID, std::vector<std::size_t>>;

struct BlockInfo {
    std::vector<VAR> variables;
    bool fixed = false;
    bool edited = false;
    std::size_t neighbours = 0;
};

Incidence indexIncidence(const ConstraintDecomposition& plan) {
    Incidence result;
    for (std::size_t i = 0; i < plan.blocks.size(); ++i)
        for (auto figure : plan.blocks[i].figures) result[figure].push_back(i);
    return result;
}

std::vector<BlockInfo> describeBlocks(const RequirementSystem& system,
    const ConstraintDecomposition& plan, const Incidence& incidence, std::span<const Utils::ID> edited) {
    std::unordered_set<VAR> fixedVariables, editedVariables;
    for (const auto& function : system.getFunctions()) {
        VAR variable = nullptr;
        double target = 0;
        if (function->tryGetAssignment(variable, target)) fixedVariables.insert(variable);
    }
    std::unordered_set<Utils::ID> editedIds(edited.begin(), edited.end());
    for (auto figure : edited)
        for (auto* variable : system.getFigureVariables(figure)) editedVariables.insert(variable);

    std::vector<BlockInfo> result(plan.blocks.size());
    for (std::size_t i = 0; i < plan.blocks.size(); ++i) {
        if (plan.blocks[i].functionIndices.empty()) continue;
        auto& info = result[i];
        std::unordered_set<VAR> seen;
        for (auto figure : plan.blocks[i].figures) {
            info.edited |= editedIds.contains(figure);
            info.neighbours += incidence.at(figure).size() - 1;
            for (auto* variable : system.getFigureVariables(figure)) {
                if (seen.insert(variable).second) info.variables.push_back(variable);
                info.fixed |= fixedVariables.contains(variable);
                info.edited |= editedVariables.contains(variable);
            }
        }
    }
    return result;
}

class BlockDiagnosis : public RequirementFunctionSystem {
public:
    Diagnosis evaluate(const std::vector<VAR>& variables) const { return diagnoseWithVariables(variables); }
};

RequirementFunctionSystem::Diagnosis diagnoseBlock(const RequirementSystem& system,
    const ConstraintBlock& block, const std::vector<VAR>& variables) {
    BlockDiagnosis diagnostic;
    for (auto index : block.functionIndices) diagnostic.addFunction(system.getFunctions().at(index));
    return diagnostic.evaluate(variables);
}

std::size_t chooseRoot(const RequirementSystem& system, const ConstraintDecomposition& plan,
                       const std::vector<BlockInfo>& info) {
    std::vector<std::size_t> candidates;
    std::pair<bool, bool> bestPriority{false, false};
    for (std::size_t i = 0; i < plan.blocks.size(); ++i) {
        if (plan.blocks[i].functionIndices.empty()) continue;
        const auto priority = std::pair{info[i].fixed, info[i].edited};
        if (candidates.empty() || priority > bestPriority) {
            candidates.clear();
            bestPriority = priority;
        }
        if (priority == bestPriority) candidates.push_back(i);
    }
    if (candidates.empty()) return 0;
    if (candidates.size() == 1) return candidates.front();

    std::size_t best = candidates.front();
    using Score = std::tuple<bool, std::size_t, std::size_t, std::size_t>;
    std::optional<Score> bestScore;
    for (auto i : candidates) {
        const auto diagnosis = diagnoseBlock(system, plan.blocks[i], info[i].variables);
        const auto score = Score{!diagnosis.degreesOfFreedom.has_value(),
            diagnosis.degreesOfFreedom.value_or(0),
            std::numeric_limits<std::size_t>::max() - info[i].neighbours,
            std::numeric_limits<std::size_t>::max() - info[i].variables.size()};
        if (!bestScore || score < *bestScore ||
            (score == *bestScore && plan.blocks[i].figures < plan.blocks[best].figures)) {
            best = i;
            bestScore = score;
        }
    }
    return best;
}

std::vector<std::size_t> traverseBlocks(const ConstraintDecomposition& plan, const Incidence& incidence,
                                       std::size_t root) {
    std::vector<std::size_t> order{root};
    std::vector<bool> visited(plan.blocks.size());
    visited[root] = true;
    std::unordered_set<Utils::ID> expanded;
    std::size_t next = 0;
    for (std::size_t cursor = 0; cursor < order.size(); ++cursor) {
        for (auto figure : plan.blocks[order[cursor]].figures) {
            if (!expanded.insert(figure).second) continue;
            for (auto block : incidence.at(figure)) {
                if (visited[block]) continue;
                visited[block] = true;
                order.push_back(block);
            }
        }
        if (cursor + 1 == order.size()) {
            while (next < visited.size() && visited[next]) ++next;
            if (next < visited.size()) {
                visited[next] = true;
                order.push_back(next);
            }
        }
    }
    return order;
}

} // namespace

std::vector<std::size_t> chooseConstraintSolveOrder(const RequirementSystem& system,
    const ConstraintDecomposition& plan, std::span<const Utils::ID> editedFigures) {
    if (plan.blocks.empty()) return {};
    const auto incidence = indexIncidence(plan);
    const auto info = describeBlocks(system, plan, incidence, editedFigures);
    return traverseBlocks(plan, incidence, chooseRoot(system, plan, info));
}

} // namespace OurPaintDCM::System
