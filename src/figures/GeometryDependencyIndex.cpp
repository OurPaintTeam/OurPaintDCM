#include "GeometryDependencyIndex.h"

#include <algorithm>

namespace OurPaintDCM::Figures {

const std::vector<Utils::ID> GeometryDependencyIndex::kEmpty{};

void GeometryDependencyIndex::pushUnique(std::vector<Utils::ID>& vec, Utils::ID id) {
    if (std::find(vec.begin(), vec.end(), id) == vec.end()) {
        vec.push_back(id);
    }
}

void GeometryDependencyIndex::eraseId(std::vector<Utils::ID>& vec, Utils::ID id) {
    vec.erase(std::remove(vec.begin(), vec.end(), id), vec.end());
}

void GeometryDependencyIndex::linkLine(Utils::ID lineId, Utils::ID p1, Utils::ID p2) {
    figureToPoints_[lineId] = {p1, p2};
    pushUnique(pointToFigures_[p1], lineId);
    pushUnique(pointToFigures_[p2], lineId);
}

void GeometryDependencyIndex::linkCircle(Utils::ID circleId, Utils::ID center) {
    figureToPoints_[circleId] = {center};
    pushUnique(pointToFigures_[center], circleId);
}

void GeometryDependencyIndex::linkArc(Utils::ID arcId, Utils::ID p1, Utils::ID p2, Utils::ID center) {
    figureToPoints_[arcId] = {p1, p2, center};
    pushUnique(pointToFigures_[p1], arcId);
    pushUnique(pointToFigures_[p2], arcId);
    pushUnique(pointToFigures_[center], arcId);
}

void GeometryDependencyIndex::unlinkFigure(Utils::ID figureId) {
    auto itFig = figureToPoints_.find(figureId);
    if (itFig == figureToPoints_.end()) {
        return;
    }
    for (Utils::ID pid : itFig->second) {
        auto itP = pointToFigures_.find(pid);
        if (itP != pointToFigures_.end()) {
            eraseId(itP->second, figureId);
            if (itP->second.empty()) {
                pointToFigures_.erase(itP);
            }
        }
    }
    figureToPoints_.erase(itFig);
}

std::vector<Utils::ID> GeometryDependencyIndex::dependentsOfPoint(Utils::ID pointId) const {
    auto it = pointToFigures_.find(pointId);
    if (it == pointToFigures_.end()) {
        return {};
    }
    return it->second;
}

const std::vector<Utils::ID>& GeometryDependencyIndex::pointsForFigure(Utils::ID figureId) const noexcept {
    auto it = figureToPoints_.find(figureId);
    if (it == figureToPoints_.end()) {
        return kEmpty;
    }
    return it->second;
}

bool GeometryDependencyIndex::hasDependents(Utils::ID pointId) const noexcept {
    auto it = pointToFigures_.find(pointId);
    return it != pointToFigures_.end() && !it->second.empty();
}

bool GeometryDependencyIndex::emptyFigure(Utils::ID figureId) const noexcept {
    return figureToPoints_.find(figureId) == figureToPoints_.end();
}

void GeometryDependencyIndex::clear() noexcept {
    pointToFigures_.clear();
    figureToPoints_.clear();
}

} // namespace OurPaintDCM::Figures
