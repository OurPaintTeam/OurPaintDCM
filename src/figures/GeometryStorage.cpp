#include "GeometryStorage.h"
#include "GeometryGraphBuilder.h"

#include <algorithm>

namespace OurPaintDCM::Figures {

void GeometryStorage::registerPointCache(ID id, const Point2D* ptr) {
    m_pointWithIdPos.emplace(id, m_pointsWithIds.size());
    m_pointsWithIds.push_back(FigureRef<Point2D>{id, ptr});
}

void GeometryStorage::registerLineCache(ID id, const Line2D* ptr) {
    m_lineWithIdPos.emplace(id, m_linesWithIds.size());
    m_linesWithIds.push_back(FigureRef<Line2D>{id, ptr});
}

void GeometryStorage::registerCircleCache(ID id, const Circle2D* ptr) {
    m_circleWithIdPos.emplace(id, m_circlesWithIds.size());
    m_circlesWithIds.push_back(FigureRef<Circle2D>{id, ptr});
}

void GeometryStorage::registerArcCache(ID id, const Arc2D* ptr) {
    m_arcWithIdPos.emplace(id, m_arcsWithIds.size());
    m_arcsWithIds.push_back(FigureRef<Arc2D>{id, ptr});
}

void GeometryStorage::erasePointCache(ID id) noexcept {
    eraseCachedById(id, m_pointsWithIds, m_pointWithIdPos);
}

void GeometryStorage::eraseLineCache(ID id) noexcept {
    eraseCachedById(id, m_linesWithIds, m_lineWithIdPos);
}

void GeometryStorage::eraseCircleCache(ID id) noexcept {
    eraseCachedById(id, m_circlesWithIds, m_circleWithIdPos);
}

void GeometryStorage::eraseArcCache(ID id) noexcept {
    eraseCachedById(id, m_arcsWithIds, m_arcWithIdPos);
}

std::size_t GeometryStorage::allocPoint(double x, double y) {
    if (!m_pointFree.empty()) {
        const std::size_t i = m_pointFree.back();
        m_pointFree.pop_back();
        m_pointSlots[i] = std::make_unique<Point2D>(x, y);
        return i;
    }
    m_pointSlots.push_back(std::make_unique<Point2D>(x, y));
    return m_pointSlots.size() - 1;
}

std::size_t GeometryStorage::allocLine(Point2D* a, Point2D* b) {
    if (!m_lineFree.empty()) {
        const std::size_t i = m_lineFree.back();
        m_lineFree.pop_back();
        m_lineSlots[i] = std::make_unique<Line2D>(a, b);
        return i;
    }
    m_lineSlots.push_back(std::make_unique<Line2D>(a, b));
    return m_lineSlots.size() - 1;
}

std::size_t GeometryStorage::allocCircle(Point2D* c, double r) {
    if (!m_circleFree.empty()) {
        const std::size_t i = m_circleFree.back();
        m_circleFree.pop_back();
        m_circleSlots[i] = std::make_unique<Circle2D>(c, r);
        return i;
    }
    m_circleSlots.push_back(std::make_unique<Circle2D>(c, r));
    return m_circleSlots.size() - 1;
}

std::size_t GeometryStorage::allocArc(Point2D* p1, Point2D* p2, Point2D* c) {
    if (!m_arcFree.empty()) {
        const std::size_t i = m_arcFree.back();
        m_arcFree.pop_back();
        m_arcSlots[i] = std::make_unique<Arc2D>(p1, p2, c);
        return i;
    }
    m_arcSlots.push_back(std::make_unique<Arc2D>(p1, p2, c));
    return m_arcSlots.size() - 1;
}

void GeometryStorage::freePoint(std::size_t slot) {
    m_pointSlots[slot].reset();
    m_pointFree.push_back(slot);
}

void GeometryStorage::freeLine(std::size_t slot) {
    m_lineSlots[slot].reset();
    m_lineFree.push_back(slot);
}

void GeometryStorage::freeCircle(std::size_t slot) {
    m_circleSlots[slot].reset();
    m_circleFree.push_back(slot);
}

void GeometryStorage::freeArc(std::size_t slot) {
    m_arcSlots[slot].reset();
    m_arcFree.push_back(slot);
}

std::optional<ID> GeometryStorage::tryPointId(ID id) const noexcept {
    auto it = m_index.find(id);
    if (it == m_index.end() || it->second.type != FigureType::ET_POINT2D) {
        return std::nullopt;
    }
    return id;
}

ID GeometryStorage::createPoint(double x, double y) {
    const std::size_t slot = allocPoint(x, y);
    const ID id = m_idGen.nextID();
    m_index.emplace(id, FigureEntry{FigureType::ET_POINT2D, static_cast<std::uint32_t>(slot)});
    registerPointCache(id, m_pointSlots[slot].get());
    return id;
}

std::optional<ID> GeometryStorage::createLine(ID p1, ID p2) {
    auto it1 = m_index.find(p1);
    auto it2 = m_index.find(p2);
    if (it1 == m_index.end() || it2 == m_index.end()) {
        return std::nullopt;
    }
    if (it1->second.type != FigureType::ET_POINT2D || it2->second.type != FigureType::ET_POINT2D) {
        return std::nullopt;
    }
    Point2D* a = m_pointSlots[it1->second.slot].get();
    Point2D* b = m_pointSlots[it2->second.slot].get();
    const std::size_t slot = allocLine(a, b);
    const ID id = m_idGen.nextID();
    m_index.emplace(id, FigureEntry{FigureType::ET_LINE, static_cast<std::uint32_t>(slot)});
    m_deps.linkLine(id, p1, p2);
    registerLineCache(id, m_lineSlots[slot].get());
    return id;
}

std::optional<ID> GeometryStorage::createCircle(ID center, double radius) {
    auto it = m_index.find(center);
    if (it == m_index.end() || it->second.type != FigureType::ET_POINT2D) {
        return std::nullopt;
    }
    Point2D* c = m_pointSlots[it->second.slot].get();
    const std::size_t slot = allocCircle(c, radius);
    const ID id = m_idGen.nextID();
    m_index.emplace(id, FigureEntry{FigureType::ET_CIRCLE, static_cast<std::uint32_t>(slot)});
    m_deps.linkCircle(id, center);
    registerCircleCache(id, m_circleSlots[slot].get());
    return id;
}

std::optional<ID> GeometryStorage::createArc(ID p1, ID p2, ID center) {
    auto it1 = m_index.find(p1);
    auto it2 = m_index.find(p2);
    auto itc = m_index.find(center);
    if (it1 == m_index.end() || it2 == m_index.end() || itc == m_index.end()) {
        return std::nullopt;
    }
    if (it1->second.type != FigureType::ET_POINT2D ||
        it2->second.type != FigureType::ET_POINT2D ||
        itc->second.type != FigureType::ET_POINT2D) {
        return std::nullopt;
    }
    Point2D* a = m_pointSlots[it1->second.slot].get();
    Point2D* b = m_pointSlots[it2->second.slot].get();
    Point2D* c = m_pointSlots[itc->second.slot].get();
    const std::size_t slot = allocArc(a, b, c);
    const ID id = m_idGen.nextID();
    m_index.emplace(id, FigureEntry{FigureType::ET_ARC, static_cast<std::uint32_t>(slot)});
    m_deps.linkArc(id, p1, p2, center);
    registerArcCache(id, m_arcSlots[slot].get());
    return id;
}

ID GeometryStorage::createFigure(FigureType type, const FigureData& data) {
    switch (type) {
        case FigureType::ET_POINT2D: {
            if (data.points.empty()) {
                throw std::runtime_error("Point data not provided");
            }
            const auto& p = data.points.front();
            return createPoint(p.x, p.y);
        }
        case FigureType::ET_LINE: {
            if (data.points.size() < 2) {
                throw std::runtime_error("Line requires two points");
            }
            const ID idA = createPoint(data.points[0].x, data.points[0].y);
            const ID idB = createPoint(data.points[1].x, data.points[1].y);
            auto lineId = createLine(idA, idB);
            if (!lineId) {
                throw std::runtime_error("Line creation failed");
            }
            return *lineId;
        }
        case FigureType::ET_CIRCLE: {
            const ID c = createPoint(data.center.x, data.center.y);
            auto circ = createCircle(c, data.radius);
            if (!circ) {
                throw std::runtime_error("Circle creation failed");
            }
            return *circ;
        }
        case FigureType::ET_ARC: {
            if (data.points.size() < 2) {
                throw std::runtime_error("Arc requires two endpoints");
            }
            const ID id1 = createPoint(data.points[0].x, data.points[0].y);
            const ID id2 = createPoint(data.points[1].x, data.points[1].y);
            const ID idC = createPoint(data.center.x, data.center.y);
            auto arcId = createArc(id1, id2, idC);
            if (!arcId) {
                throw std::runtime_error("Arc creation failed");
            }
            return *arcId;
        }
        default:
            throw std::runtime_error("Unknown figure type");
    }
}

RemoveResult GeometryStorage::removeFigureOnly(ID id) noexcept {
    auto it = m_index.find(id);
    if (it == m_index.end() || it->second.type == FigureType::ET_POINT2D) {
        return RemoveResult::NotFound;
    }
    const FigureEntry ent = it->second;
    m_deps.unlinkFigure(id);

    switch (ent.type) {
        case FigureType::ET_LINE:
            eraseLineCache(id);
            m_lineSlots[ent.slot].reset();
            m_lineFree.push_back(ent.slot);
            break;
        case FigureType::ET_CIRCLE:
            eraseCircleCache(id);
            m_circleSlots[ent.slot].reset();
            m_circleFree.push_back(ent.slot);
            break;
        case FigureType::ET_ARC:
            eraseArcCache(id);
            m_arcSlots[ent.slot].reset();
            m_arcFree.push_back(ent.slot);
            break;
        default:
            return RemoveResult::NotFound;
    }
    m_index.erase(it);
    return RemoveResult::Ok;
}

RemoveResult GeometryStorage::remove(ID id, bool forceCascade) noexcept {
    auto it = m_index.find(id);
    if (it == m_index.end()) {
        return RemoveResult::NotFound;
    }

    if (it->second.type == FigureType::ET_POINT2D) {
        if (m_deps.hasDependents(id)) {
            if (!forceCascade) {
                return RemoveResult::BlockedByDependents;
            }
            const std::vector<ID> toRemove = m_deps.dependentsOfPoint(id);
            for (ID fig : toRemove) {
                (void)removeFigureOnly(fig);
            }
        }
        auto itPt = m_index.find(id);
        if (itPt == m_index.end() || itPt->second.type != FigureType::ET_POINT2D) {
            return RemoveResult::NotFound;
        }
        const std::uint32_t slot = itPt->second.slot;
        erasePointCache(id);
        freePoint(slot);
        m_index.erase(itPt);
        return RemoveResult::Ok;
    }

    return removeFigureOnly(id);
}

void GeometryStorage::clear() noexcept {
    m_pointSlots.clear();
    m_lineSlots.clear();
    m_circleSlots.clear();
    m_arcSlots.clear();
    m_pointFree.clear();
    m_lineFree.clear();
    m_circleFree.clear();
    m_arcFree.clear();
    m_index.clear();
    m_deps.clear();
    m_pointsWithIds.clear();
    m_linesWithIds.clear();
    m_circlesWithIds.clear();
    m_arcsWithIds.clear();
    m_pointWithIdPos.clear();
    m_lineWithIdPos.clear();
    m_circleWithIdPos.clear();
    m_arcWithIdPos.clear();
    m_idGen.reset();
}

std::optional<FigureType> GeometryStorage::getType(ID id) const noexcept {
    auto it = m_index.find(id);
    if (it == m_index.end()) {
        return std::nullopt;
    }
    return it->second.type;
}

std::optional<FigureEntry> GeometryStorage::getEntry(ID id) const noexcept {
    auto it = m_index.find(id);
    if (it == m_index.end()) {
        return std::nullopt;
    }
    return it->second;
}

bool GeometryStorage::contains(ID id) const noexcept {
    return m_index.contains(id);
}

std::vector<ID> GeometryStorage::getIDsByType(FigureType type) const {
    switch (type) {
        case FigureType::ET_POINT2D: {
            std::vector<ID> ids;
            ids.reserve(m_pointsWithIds.size());
            for (const auto& r : m_pointsWithIds) {
                ids.push_back(r.id);
            }
            return ids;
        }
        case FigureType::ET_LINE: {
            std::vector<ID> ids;
            ids.reserve(m_linesWithIds.size());
            for (const auto& r : m_linesWithIds) {
                ids.push_back(r.id);
            }
            return ids;
        }
        case FigureType::ET_CIRCLE: {
            std::vector<ID> ids;
            ids.reserve(m_circlesWithIds.size());
            for (const auto& r : m_circlesWithIds) {
                ids.push_back(r.id);
            }
            return ids;
        }
        case FigureType::ET_ARC: {
            std::vector<ID> ids;
            ids.reserve(m_arcsWithIds.size());
            for (const auto& r : m_arcsWithIds) {
                ids.push_back(r.id);
            }
            return ids;
        }
        default:
            return {};
    }
}

std::size_t GeometryStorage::size() const noexcept {
    return m_index.size();
}

std::size_t GeometryStorage::pointCount() const noexcept {
    return m_pointsWithIds.size();
}

std::size_t GeometryStorage::lineCount() const noexcept {
    return m_linesWithIds.size();
}

std::size_t GeometryStorage::circleCount() const noexcept {
    return m_circlesWithIds.size();
}

std::size_t GeometryStorage::arcCount() const noexcept {
    return m_arcsWithIds.size();
}

bool GeometryStorage::empty() const noexcept {
    return m_index.empty();
}

const ID& GeometryStorage::currentID() const noexcept {
    return m_idGen.current();
}

std::vector<ID> GeometryStorage::getDependents(ID pointId) const {
    if (!tryPointId(pointId)) {
        return {};
    }
    return m_deps.dependentsOfPoint(pointId);
}

std::vector<ID> GeometryStorage::getDependencies(ID id) const {
    auto it = m_index.find(id);
    if (it == m_index.end() || it->second.type == FigureType::ET_POINT2D) {
        return {};
    }
    const auto& v = m_deps.pointsForFigure(id);
    return {v.begin(), v.end()};
}

ObjectGraph GeometryStorage::buildObjectGraph() const {
    return GeometryGraphBuilder::buildObjectGraph(*this);
}

std::optional<ObjectGraph> GeometryStorage::buildObjectSubgraph(ID id) const {
    return GeometryGraphBuilder::buildObjectSubgraph(*this, id);
}

#ifndef NDEBUG
bool GeometryStorage::validate() const noexcept {
    const std::size_t totalCached =
        m_pointsWithIds.size() + m_linesWithIds.size() + m_circlesWithIds.size() + m_arcsWithIds.size();
    if (m_index.size() != totalCached) {
        return false;
    }
    if (m_index.size() != m_pointWithIdPos.size() + m_lineWithIdPos.size() + m_circleWithIdPos.size() +
                               m_arcWithIdPos.size()) {
        return false;
    }
    for (const auto& [id, ent] : m_index) {
        switch (ent.type) {
            case FigureType::ET_POINT2D:
                if (ent.slot >= m_pointSlots.size() || !m_pointSlots[ent.slot]) {
                    return false;
                }
                if (m_pointSlots[ent.slot].get() != get<Point2D>(id)) {
                    return false;
                }
                break;
            case FigureType::ET_LINE:
                if (ent.slot >= m_lineSlots.size() || !m_lineSlots[ent.slot]) {
                    return false;
                }
                break;
            case FigureType::ET_CIRCLE:
                if (ent.slot >= m_circleSlots.size() || !m_circleSlots[ent.slot]) {
                    return false;
                }
                break;
            case FigureType::ET_ARC:
                if (ent.slot >= m_arcSlots.size() || !m_arcSlots[ent.slot]) {
                    return false;
                }
                break;
            default:
                return false;
        }
    }
    for (const auto& r : m_pointsWithIds) {
        if (!contains(r.id) || get<Point2D>(r.id) != r.ptr) {
            return false;
        }
    }
    for (const auto& r : m_linesWithIds) {
        if (!contains(r.id) || get<Line2D>(r.id) != r.ptr) {
            return false;
        }
        const auto& pts = m_deps.pointsForFigure(r.id);
        if (pts.size() != 2) {
            return false;
        }
    }
    for (const auto& r : m_circlesWithIds) {
        if (!contains(r.id) || get<Circle2D>(r.id) != r.ptr) {
            return false;
        }
        if (m_deps.pointsForFigure(r.id).size() != 1) {
            return false;
        }
    }
    for (const auto& r : m_arcsWithIds) {
        if (!contains(r.id) || get<Arc2D>(r.id) != r.ptr) {
            return false;
        }
        if (m_deps.pointsForFigure(r.id).size() != 3) {
            return false;
        }
    }
    return true;
}
#endif

} // namespace OurPaintDCM::Figures
