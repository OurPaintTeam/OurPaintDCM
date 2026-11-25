#include "ConstraintDecomposition.h"
#include "RequirementSystem.h"
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <utility>

namespace OurPaintDCM::System {
namespace {

using BlockIndices = std::vector<std::size_t>;
using FigureBlocks = std::unordered_map<Utils::ID, BlockIndices>;
using RequirementOwners = std::unordered_map<Utils::ID, std::size_t>;

struct ParameterFigures {
    std::vector<Utils::ID> figures;
    std::vector<Utils::ID> boundaries;
};
using ParameterIndex = std::unordered_map<VAR, ParameterFigures>;

FigureBlocks indexFigures(const ConstraintDecomposition& plan) {
    FigureBlocks index;
    for (std::size_t block = 0; block < plan.blocks.size(); ++block)
        for (auto figure : plan.blocks[block].figures) index[figure].push_back(block);
    return index;
}

ParameterIndex indexParameters(const RequirementSystem& system, const FigureBlocks& figures,
                               const std::vector<Utils::ID>& articulationPoints) {
    ParameterIndex index;
    for (const auto& [figure, blocks] : figures) {
        const bool boundary = std::binary_search(articulationPoints.begin(), articulationPoints.end(), figure);
        for (auto* variable : system.getFigureVariables(figure)) {
            auto& entry = index[variable];
            entry.figures.push_back(figure);
            if (boundary) entry.boundaries.push_back(figure);
        }
    }
    for (auto& [variable, entry] : index) {
        std::sort(entry.figures.begin(), entry.figures.end());
        std::sort(entry.boundaries.begin(), entry.boundaries.end());
    }
    return index;
}

bool containsBlock(const std::vector<Utils::ID>& candidates, std::size_t block, const FigureBlocks& figures) {
    return std::any_of(candidates.begin(), candidates.end(), [&](Utils::ID figure) {
        const auto& blocks = figures.at(figure);
        return std::binary_search(blocks.begin(), blocks.end(), block);
    });
}

bool coversVariables(const std::vector<VAR>& variables, std::size_t block,
                     const ParameterIndex& parameters, const FigureBlocks& figures) {
    return std::all_of(variables.begin(), variables.end(), [&](VAR variable) {
        const auto found = parameters.find(variable);
        return found != parameters.end() && containsBlock(found->second.figures, block, figures);
    });
}

std::optional<std::size_t> requirementOwner(const std::vector<Utils::ID>& objects, const FigureBlocks& figures) {
    if (objects.empty()) return 0;
    const BlockIndices* candidates = nullptr;
    for (const auto object : objects) {
        const auto found = figures.find(object);
        if (found == figures.end()) return std::nullopt;
        const auto* blocks = &found->second;
        if (!candidates || blocks->size() < candidates->size()) candidates = blocks;
    }
    for (const auto candidate : *candidates) {
        if (std::all_of(objects.begin(), objects.end(), [&](const auto object) {
            const auto& blocks = figures.at(object);
            return std::binary_search(blocks.begin(), blocks.end(), candidate);
        })) return candidate;
    }
    return std::nullopt;
}

std::optional<RequirementOwners> assignRequirements(const RequirementSystem& system,
    const FigureBlocks& figures, ConstraintDecomposition& plan) {
    RequirementOwners owners;
    for (const auto& requirement : system.getRequirements()) {
        const auto owner = requirementOwner(requirement.objectIds, figures);
        if (!owner) return std::nullopt;
        owners.emplace(requirement.id, *owner);
        plan.blocks[*owner].requirements.push_back(requirement.id);
    }
    return owners;
}

std::optional<std::size_t> functionOwner(const std::vector<VAR>& variables,
                                       const ParameterIndex& parameters, const FigureBlocks& figures) {
    if (variables.empty()) return 0;
    const ParameterFigures* candidates = nullptr;
    std::size_t smallestSize = 0;
    for (auto* variable : variables) {
        const auto found = parameters.find(variable);
        if (found == parameters.end()) return std::nullopt;
        std::size_t size = 0;
        for (auto figure : found->second.figures) size += figures.at(figure).size();
        if (!candidates || size < smallestSize) {
            candidates = &found->second;
            smallestSize = size;
        }
    }
    for (auto figure : candidates->figures)
        for (auto block : figures.at(figure))
            if (coversVariables(variables, block, parameters, figures)) return block;
    return std::nullopt;
}

bool assignFunctions(const RequirementSystem& system, const RequirementOwners& owners,
                     const ParameterIndex& parameters, const FigureBlocks& figures, ConstraintDecomposition& plan) {
    const auto& functions = system.getFunctions();
    for (std::size_t index = 0; index < functions.size(); ++index) {
        const auto& function = functions[index];
        std::optional<std::size_t> owner;
        if (const auto requirement = function->requirementId()) {
            const auto found = owners.find(*requirement);
            if (found == owners.end()) return false;
            owner = found->second;
        } else {
            owner = functionOwner(function->getVars(), parameters, figures);
        }
        if (!owner || !coversVariables(function->getVars(), *owner, parameters, figures)) return false;
        plan.blocks[*owner].functionIndices.push_back(index);
    }
    return true;
}

bool collectSharedParameters(const ParameterIndex& parameters, const FigureBlocks& figures,
                             ConstraintDecomposition& plan) {
    for (const auto& [variable, entry] : parameters) {
        const auto firstBlock = figures.at(entry.figures.front()).front();
        const bool shared = std::any_of(entry.figures.begin(), entry.figures.end(), [&](Utils::ID figure) {
            const auto& blocks = figures.at(figure);
            return blocks.size() > 1 || blocks.front() != firstBlock;
        });
        if (!shared) continue;
        for (auto figure : entry.figures) {
            if (std::binary_search(entry.boundaries.begin(), entry.boundaries.end(), figure)) continue;
            for (auto block : figures.at(figure))
                if (!containsBlock(entry.boundaries, block, figures)) return false;
        }
        plan.sharedVariables.push_back(variable);
    }
    std::sort(plan.sharedVariables.begin(), plan.sharedVariables.end(), std::less<VAR>{});
    return true;
}

} // namespace

std::optional<ConstraintDecomposition> buildConstraintDecomposition(
    const RequirementSystem& system, ArticulationDecomposition decomposition) {
    if (decomposition.blocks.size() < 2 || decomposition.articulationPoints.empty()
        || system.getFunctions().empty()) return std::nullopt;

    ConstraintDecomposition plan;
    plan.articulationPoints = std::move(decomposition.articulationPoints);
    plan.blocks.reserve(decomposition.blocks.size());
    for (auto& figures : decomposition.blocks) plan.blocks.push_back({std::move(figures), {}, {}});

    const auto figures = indexFigures(plan);
    const auto owners = assignRequirements(system, figures, plan);
    if (!owners) return std::nullopt;
    const auto parameters = indexParameters(system, figures, plan.articulationPoints);
    if (!assignFunctions(system, *owners, parameters, figures, plan)
        || !collectSharedParameters(parameters, figures, plan)) return std::nullopt;
    const auto populated = std::count_if(plan.blocks.begin(), plan.blocks.end(), [](const auto& block) {
        return !block.functionIndices.empty();
    });
    if (populated < 2) return std::nullopt;
    return plan;
}

} // namespace OurPaintDCM::System
