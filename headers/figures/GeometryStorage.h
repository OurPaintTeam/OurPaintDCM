#ifndef OURPAINTDCM_HEADERS_FIGURES_GEOMETRYSTORAGE_H
#define OURPAINTDCM_HEADERS_FIGURES_GEOMETRYSTORAGE_H

#include "GeometryDependencyIndex.h"
#include "Point2D.h"
#include "Line.h"
#include "Circle.h"
#include "Arc.h"
#include "ID.h"
#include "IDGenerator.h"
#include "Enums.h"
#include "Graph.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>
#include <concepts>
#include <stdexcept>

namespace OurPaintDCM::Figures {

using Utils::ID;
using Utils::IDGenerator;
using Utils::FigureType;
using Line2D   = Line<Point2D>;
using Circle2D = Circle<Point2D>;
using Arc2D    = Arc<Point2D>;
using ObjectGraph = Graph<ID, ID, UndirectedPolicy, WeightedPolicy>;

/**
 * @brief Types allowed for GeometryStorage::get().
 */
template <typename T>
concept SupportedFigure = std::same_as<T, Point2D> ||
                          std::same_as<T, Line2D>  ||
                          std::same_as<T, Circle2D> ||
                          std::same_as<T, Arc2D>;

/**
 * @brief Outcome of a non-throwing remove operation.
 */
enum class RemoveResult : std::uint8_t {
    Ok,                   /**< Object removed; state updated atomically for this call. */
    NotFound,             /**< No object with this ID (or wrong branch for removeFigureOnly). */
    BlockedByDependents,  /**< Point still referenced by figures; use forceCascade or delete figures first. */
};

/**
 * @brief Maps a public figure ID to its FigureType and index in the per-type slot pool.
 */
struct FigureEntry {
    FigureType    type{};
    std::uint32_t slot{};

    constexpr FigureEntry() noexcept = default;
    constexpr FigureEntry(FigureType t, std::uint32_t s) noexcept : type(t), slot(s) {}
};

/**
 * @brief Aggregated coordinates for GeometryStorage::createFigure() (batch / descriptor path).
 */
struct FigureData {
    struct PointData {
        double x{};
        double y{};
    };
    std::vector<PointData> points{};
    PointData center{};
    double     radius{};
};

/**
 * @brief Cached handle for linear iteration: stable ID with a non-owning pointer into storage.
 *
 * Pointer remains valid while the object exists and is not removed from this storage.
 */
template <typename T>
struct FigureRef {
    ID         id{};
    const T*   ptr{nullptr};
};

/**
 * @brief ID-first 2D geometry store with slot pools and a bidirectional point↔figure dependency index.
 *
 * Dependency queries avoid scanning the whole scene: incident figures per point are O(degree)
 * in the index; each figure keeps a fixed small list of endpoint / center IDs.
 */
class GeometryStorage {
public:
    /** @brief Constructs an empty storage; IDs start from the generator default. */
    GeometryStorage() = default;
    /** @brief Releases all figures and clears indices. */
    ~GeometryStorage() = default;

    GeometryStorage(const GeometryStorage&) = delete;
    GeometryStorage& operator=(const GeometryStorage&) = delete;
    /** @brief Move-constructs; source is left in a valid empty state. */
    GeometryStorage(GeometryStorage&&) noexcept = default;
    /** @brief Move-assigns; source is left in a valid empty state. */
    GeometryStorage& operator=(GeometryStorage&&) noexcept = default;

    /**
     * @brief Creates a point at (x, y).
     * @return New unique ID for the point.
     */
    [[nodiscard]] ID createPoint(double x, double y);

    /**
     * @brief Creates a line through two existing points.
     * @param p1 First endpoint ID (must be ET_POINT2D).
     * @param p2 Second endpoint ID (must be ET_POINT2D).
     * @return Line ID, or std::nullopt if either ID is missing or not a point.
     */
    [[nodiscard]] std::optional<ID> createLine(ID p1, ID p2);

    /**
     * @brief Creates a circle with given center and radius.
     * @param center Center point ID (must be ET_POINT2D).
     * @param radius Non-negative radius (semantics validated by Circle type).
     * @return Circle ID, or std::nullopt if center is invalid.
     */
    [[nodiscard]] std::optional<ID> createCircle(ID center, double radius);

    /**
     * @brief Creates an arc through two points with a center point.
     * @param p1 First endpoint ID.
     * @param p2 Second endpoint ID.
     * @param center Center point ID.
     * @return Arc ID, or std::nullopt if any ID is missing or not a point.
     */
    [[nodiscard]] std::optional<ID> createArc(ID p1, ID p2, ID center);

    /**
     * @brief Creates a figure from aggregated FigureData (may create nested points).
     * @param type Figure kind (point, line, circle, arc).
     * @param data Coordinates and radius as required by @p type.
     * @return ID of the top-level created figure (for line/circle/arc) or point.
     * @throws std::runtime_error if @p data is inconsistent or creation fails.
     */
    [[nodiscard]] ID createFigure(FigureType type, const FigureData& data);

    /**
     * @brief Mutable access to a figure by ID and expected type.
     * @return Pointer into storage, or nullptr if ID missing or type mismatch.
     */
    template <SupportedFigure T>
    [[nodiscard]] T* get(ID id) noexcept;

    /**
     * @brief Const access to a figure by ID and expected type.
     * @return Pointer into storage, or nullptr if ID missing or type mismatch.
     */
    template <SupportedFigure T>
    [[nodiscard]] const T* get(ID id) const noexcept;

    /**
     * @brief Returns the stored FigureType for an ID, if present.
     */
    [[nodiscard]] std::optional<FigureType> getType(ID id) const noexcept;

    /**
     * @brief Returns the internal FigureEntry (type + pool slot) for diagnostics/tools.
     */
    [[nodiscard]] std::optional<FigureEntry> getEntry(ID id) const noexcept;

    /**
     * @brief True if @p id is currently a live object in this storage.
     */
    [[nodiscard]] bool contains(ID id) const noexcept;

    /**
     * @brief Removes an object by ID without throwing.
     *
     * For a point: if figures still reference it, returns BlockedByDependents unless
     * @p forceCascade is true, in which case dependent figures are removed first, then the point.
     * For a non-point figure: removes only that figure and updates the dependency index.
     *
     * @param id Object to remove.
     * @param forceCascade When true, delete incident figures before deleting a point.
     * @return RemoveResult describing success or reason for failure.
     */
    [[nodiscard]] RemoveResult remove(ID id, bool forceCascade = false) noexcept;

    /**
     * @brief Removes every object, resets ID generator, clears all indices and free lists.
     */
    void clear() noexcept;

    /**
     * @brief Lists all IDs of a given figure type (order matches internal cache order).
     * @note complexity O(n) for n of that type — intended for enumeration, not hot paths.
     */
    [[nodiscard]] std::vector<ID> getIDsByType(FigureType type) const;

    /** @brief Total number of live objects (all types). */
    [[nodiscard]] std::size_t size() const noexcept;
    /** @brief Number of live points. */
    [[nodiscard]] std::size_t pointCount() const noexcept;
    /** @brief Number of live lines. */
    [[nodiscard]] std::size_t lineCount() const noexcept;
    /** @brief Number of live circles. */
    [[nodiscard]] std::size_t circleCount() const noexcept;
    /** @brief Number of live arcs. */
    [[nodiscard]] std::size_t arcCount() const noexcept;
    /** @brief True if size() == 0. */
    [[nodiscard]] bool empty() const noexcept;

    /**
     * @brief Last issued ID from the internal generator (same as IDGenerator::current()).
     */
    [[nodiscard]] const ID& currentID() const noexcept;

    /**
     * @brief Figures incident on a point (lines/circles/arcs that use this point).
     * @param pointId Must reference a point; otherwise returns empty vector.
     * @return Copy of the index list; cost O(degree) for allocation and copy.
     */
    [[nodiscard]] std::vector<ID> getDependents(ID pointId) const;

    /**
     * @brief Point IDs this figure depends on (endpoints / center); empty for points.
     * @return Copy of stored arity list; lookup O(1), copy O(k) for k endpoints.
     */
    [[nodiscard]] std::vector<ID> getDependencies(ID id) const;

    /**
     * @brief Full object graph: all vertices plus edges between figures and their points.
     */
    [[nodiscard]] ObjectGraph buildObjectGraph() const;

    /**
     * @brief Local subgraph around @p id: the object, its neighbours, and edges among them.
     * @return std::nullopt if @p id is not in storage.
     */
    [[nodiscard]] std::optional<ObjectGraph> buildObjectSubgraph(ID id) const;

    /** @brief Read-only cache of all points for fast iteration (IDs + const pointers). */
    [[nodiscard]] const std::vector<FigureRef<Point2D>>& pointsWithIds() const noexcept { return m_pointsWithIds; }
    /** @brief Read-only cache of all lines. */
    [[nodiscard]] const std::vector<FigureRef<Line2D>>& linesWithIds() const noexcept { return m_linesWithIds; }
    /** @brief Read-only cache of all circles. */
    [[nodiscard]] const std::vector<FigureRef<Circle2D>>& circlesWithIds() const noexcept { return m_circlesWithIds; }
    /** @brief Read-only cache of all arcs. */
    [[nodiscard]] const std::vector<FigureRef<Arc2D>>& arcsWithIds() const noexcept { return m_arcsWithIds; }

#ifndef NDEBUG
    /**
     * @brief Validates index, caches, slot occupancy, and dependency arity in debug builds.
     * @return False if any invariant is violated.
     */
    [[nodiscard]] bool validate() const noexcept;

    /** @brief Point pool vector size (includes free slots); for tests/diagnostics only. */
    [[nodiscard]] std::size_t debugPointPoolSize() const noexcept { return m_pointSlots.size(); }
    /** @brief Line pool vector size (includes free slots); for tests/diagnostics only. */
    [[nodiscard]] std::size_t debugLinePoolSize() const noexcept { return m_lineSlots.size(); }
#endif

private:
    /**
     * @brief Removes a line, circle, or arc by ID; does not delete points.
     * @return NotFound if id missing, is a point, or unknown type.
     */
    [[nodiscard]] RemoveResult removeFigureOnly(ID id) noexcept;

    /**
     * @brief Returns @p id if it names a point; otherwise std::nullopt.
     */
    std::optional<ID> tryPointId(ID id) const noexcept;

    /** @brief Appends to per-type iteration cache and position map. */
    void registerPointCache(ID id, const Point2D* ptr);
    void registerLineCache(ID id, const Line2D* ptr);
    void registerCircleCache(ID id, const Circle2D* ptr);
    void registerArcCache(ID id, const Arc2D* ptr);

    /** @brief Swaps out of dense cache by ID (swap-with-last). */
    void erasePointCache(ID id) noexcept;
    void eraseLineCache(ID id) noexcept;
    void eraseCircleCache(ID id) noexcept;
    void eraseArcCache(ID id) noexcept;

    /**
     * @brief Removes one entry from @p cache and keeps @p posMap consistent.
     */
    template <typename T>
    static void eraseCachedById(
        ID id,
        std::vector<FigureRef<T>>& cache,
        std::unordered_map<ID, std::size_t>& posMap
    ) noexcept;

    /** @brief Allocates or reuses a point slot; returns slot index. */
    std::size_t allocPoint(double x, double y);
    /** @brief Allocates or reuses a line slot; line stores raw pointers @p a, @p b. */
    std::size_t allocLine(Point2D* a, Point2D* b);
    /** @brief Allocates or reuses a circle slot. */
    std::size_t allocCircle(Point2D* c, double r);
    /** @brief Allocates or reuses an arc slot. */
    std::size_t allocArc(Point2D* p1, Point2D* p2, Point2D* c);

    /** @brief Destroys point in slot and pushes slot index onto the point free list. */
    void freePoint(std::size_t slot);
    /** @brief Destroys line in slot and pushes slot index onto the line free list. */
    void freeLine(std::size_t slot);
    /** @brief Destroys circle in slot and pushes slot index onto the circle free list. */
    void freeCircle(std::size_t slot);
    /** @brief Destroys arc in slot and pushes slot index onto the arc free list. */
    void freeArc(std::size_t slot);

    /** @brief Maps C++ figure type to FigureType enum. */
    template <SupportedFigure T>
    static constexpr FigureType typeToEnum() noexcept;

    std::vector<std::unique_ptr<Point2D>> m_pointSlots;
    std::vector<std::size_t>              m_pointFree;
    std::vector<std::unique_ptr<Line2D>> m_lineSlots;
    std::vector<std::size_t>              m_lineFree;
    std::vector<std::unique_ptr<Circle2D>> m_circleSlots;
    std::vector<std::size_t>              m_circleFree;
    std::vector<std::unique_ptr<Arc2D>> m_arcSlots;
    std::vector<std::size_t>              m_arcFree;

    std::unordered_map<ID, FigureEntry> m_index;
    IDGenerator                         m_idGen;
    GeometryDependencyIndex             m_deps;

    std::vector<FigureRef<Point2D>>  m_pointsWithIds;
    std::vector<FigureRef<Line2D>>   m_linesWithIds;
    std::vector<FigureRef<Circle2D>> m_circlesWithIds;
    std::vector<FigureRef<Arc2D>>    m_arcsWithIds;

    std::unordered_map<ID, std::size_t> m_pointWithIdPos;
    std::unordered_map<ID, std::size_t> m_lineWithIdPos;
    std::unordered_map<ID, std::size_t> m_circleWithIdPos;
    std::unordered_map<ID, std::size_t> m_arcWithIdPos;
};

/** @brief Compile-time FigureType for template get(). */
template <SupportedFigure T>
constexpr FigureType GeometryStorage::typeToEnum() noexcept {
    if constexpr (std::same_as<T, Point2D>)  return FigureType::ET_POINT2D;
    if constexpr (std::same_as<T, Line2D>)   return FigureType::ET_LINE;
    if constexpr (std::same_as<T, Circle2D>) return FigureType::ET_CIRCLE;
    if constexpr (std::same_as<T, Arc2D>)    return FigureType::ET_ARC;
}

/** @brief Non-const get(); see class GeometryStorage::get. */
template <SupportedFigure T>
T* GeometryStorage::get(ID id) noexcept {
    auto it = m_index.find(id);
    if (it == m_index.end() || it->second.type != typeToEnum<T>()) {
        return nullptr;
    }
    const std::uint32_t s = it->second.slot;
    if constexpr (std::same_as<T, Point2D>) {
        return m_pointSlots[s].get();
    } else if constexpr (std::same_as<T, Line2D>) {
        return m_lineSlots[s].get();
    } else if constexpr (std::same_as<T, Circle2D>) {
        return m_circleSlots[s].get();
    } else {
        return m_arcSlots[s].get();
    }
}

/** @brief Const get(); see class GeometryStorage::get. */
template <SupportedFigure T>
const T* GeometryStorage::get(ID id) const noexcept {
    auto it = m_index.find(id);
    if (it == m_index.end() || it->second.type != typeToEnum<T>()) {
        return nullptr;
    }
    const std::uint32_t s = it->second.slot;
    if constexpr (std::same_as<T, Point2D>) {
        return m_pointSlots[s].get();
    } else if constexpr (std::same_as<T, Line2D>) {
        return m_lineSlots[s].get();
    } else if constexpr (std::same_as<T, Circle2D>) {
        return m_circleSlots[s].get();
    } else {
        return m_arcSlots[s].get();
    }
}

/**
 * @brief Erase-by-swap in the dense FigureRef cache; updates secondary position map.
 */
template <typename T>
void GeometryStorage::eraseCachedById(
    ID id,
    std::vector<FigureRef<T>>& cache,
    std::unordered_map<ID, std::size_t>& posMap
) noexcept {
    auto it = posMap.find(id);
    if (it == posMap.end()) {
        return;
    }
    const std::size_t idx = it->second;
    const std::size_t lastIdx = cache.size() - 1;
    if (idx != lastIdx) {
        cache[idx] = cache[lastIdx];
        posMap[cache[idx].id] = idx;
    }
    cache.pop_back();
    posMap.erase(it);
}

} // namespace OurPaintDCM::Figures

#endif
