#ifndef HEADERS_UTILS_ID_H
#define HEADERS_UTILS_ID_H
#include <cstddef>
#include <functional>
namespace OurPaintDCM::Utils {
/**
 * @brief Strongly typed wrapper for an unsigned long long identifier.
 *
 * The ID struct encapsulates an unsigned 64-bit integer identifier and
 * provides comparison and increment operators. It can be used as a safer,
 * more expressive alternative to raw integers when working with IDs.
 */
struct ID {
    unsigned long long id;///< Underlying identifier value.
    /**
     * @brief Default constructor.
     *
     * Initializes the ID to zero.
     */
    ID() : id(0) {}
    /**
     * @brief Explicit constructor.
     *
     * Initializes the ID with a given unsigned long long value.
     *
     * @param value The initial ID value.
     */
    explicit ID(unsigned long long value): id(value) {}
    /**
     * @brief Equality operator.
     *
     * @param other The other ID to compare with.
     * @return true if the IDs are equal, false otherwise.
     */
    bool operator==(const ID& other) const noexcept {
        return id == other.id;
    }
    /**
     * @brief Inequality operator.
     *
     * @param other The other ID to compare with.
     * @return true if the IDs are not equal, false otherwise.
     */
    bool operator!=(const ID& other) const noexcept {
        return id != other.id;
    }
    /**
     * @brief Less-than operator.
     *
     * Allows ordering of IDs, e.g., for use in sorted containers.
     *
     * @param other The other ID to compare with.
     * @return true if this ID is less than the other, false otherwise.
     */
    bool operator<(const ID& other) const noexcept {
        return id < other.id;
    }
    /**
    * @brief Greater-than operator.
    *
    * @param other The other ID to compare with.
    * @return true if this ID is greater than the other, false otherwise.
    */
    bool operator>(const ID& other) const noexcept {
        return id > other.id;
    }
    /**
     * @brief Prefix increment operator.
     *
     * Increments the ID value by 1 and returns a reference to this ID.
     *
     * @return Reference to the incremented ID.
     */
    ID& operator++() noexcept {
        id++;
        return *this;
    }
    /**
     * @brief Postfix increment operator.
     *
     * Increments the ID value by 1 but returns the previous value.
     *
     * @return Copy of the ID before increment.
     */
    ID operator++(int) noexcept {
        ID temp = *this;
        id++;
        return temp;
    }
};
}
namespace std {
template<>
struct hash<OurPaintDCM::Utils::ID> {
    std::size_t operator()(const OurPaintDCM::Utils::ID& key) const noexcept { // <- const&
        return std::hash<unsigned long long>{}(key.id);
    }
};
}

#endif //HEADERS_UTILS_ID_H
