#pragma once

/**
 * @file binary_set.hxx
 * @brief Compact bitset-backed set for small non-negative integers.
 * @version 1.1.0
 *
 * @details
 * `BinarySet` stores a dense set of non-negative integers in a flat array
 * of `uint64_t` words.  All single-element operations (`add`, `remove`,
 * `contains`) are O(1) bit-manipulation with no branching on the value.
 *
 * Set-algebra operators (`&`, `|`, `^`, `-`) operate word-at-a-time using
 * SIMD-friendly loops; `std::popcount` / `std::countr_zero` (C++20) handle
 * cardinality and iteration.  An iterator class supports range-for by
 * scanning for set bits via the `x & (x-1)` trick.
 *
 * Key design choices:
 *   - Capacity is fixed at construction time (rounded up to the next word).
 *   - The class deliberately avoids `std::bitset` to allow runtime-sized
 *     capacity and to expose the word array for low-level operations.
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2026 Matteo Zanella
 *
 * SPDX-License-Identifier: MIT
 */

#include <algorithm>  // std::all_of, std::fill, std::find
#include <bit>        // std::popcount, std::countr_zero  [C++20]
#include <cstddef>    // std::ptrdiff_t, std::size_t
#include <cstdint>    // uint64_t
#include <iterator>   // std::forward_iterator_tag
#include <ostream>    // std::ostream
#include <stdexcept>  // std::invalid_argument, std::domain_error, std::out_of_range
#include <string>     // std::string
#include <vector>     // std::vector

// ─────────────────────────────────────────────────────────────────────────────
// Internal constants
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {
inline constexpr unsigned int CHUNK_BITS = 64U;    ///< Bits per storage word
using chunk_t = uint64_t;                          ///< Storage word type
inline constexpr chunk_t CHUNK_ALL = ~chunk_t{0};  ///< All bits set
}  // namespace detail

// ═══════════════════════════════════════════════════════════════════════════
//  BinarySet
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief A space-efficient set of unsigned integers over a fixed universe.
 *
 * Elements are unsigned integers in the range [0, capacity-1].  Internally,
 * each element maps to one bit in a packed array of 64-bit words, so the
 * storage overhead is approximately capacity/8 bytes.
 *
 * ### Complexity summary
 * | Operation               | Time          | Notes                         |
 * |-------------------------|---------------|-------------------------------|
 * | add / remove / contains | O(1)          | Two arithmetic ops + bit test |
 * | size / empty            | O(1)          | Maintained as running counter |
 * | Set algebra (|, &, -, ^)| O(capacity/64)| Word-at-a-time                |
 * | Iteration               | O(capacity)   | One popcount per word         |
 * | sparse / string         | O(capacity)   | Full scan                     |
 *
 * ### Thread safety
 * No internal synchronisation.  Concurrent reads are safe; any concurrent
 * mutation requires external locking.
 *
 * ### Example
 * @code
 * BinarySet a(16), b(16);
 * a.add(1); a.add(3); a.add(5);
 * b.add(3); b.add(5); b.add(7);
 *
 * BinarySet u = a | b;   // {1,3,5,7}
 * BinarySet i = a & b;   // {3,5}
 * BinarySet d = a - b;   // {1}
 * BinarySet s = a ^ b;   // {1,7}
 *
 * for (unsigned int elem : a) { std::cout << elem << ' '; }   // 1 3 5
 * std::cout << a;                                             // [X-X-X-----------]
 * @endcode
 */
class BinarySet {
   public:
    // Forward declaration for begin()/end()
    class iterator;

    // ───────────────────────────────────────────────────────────────────────
    // Construction
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Default constructor — creates an unusable empty set with capacity 0.
     *
     * A default-constructed set throws std::domain_error on any mutating or
     * querying operation.  It exists to allow storage in containers before
     * assignment.
     */
    BinarySet() noexcept = default;

    /**
     * @brief Constructs a BinarySet with the given universe size.
     *
     * @param capacity  Number of distinct representable elements;
     *                  elements are integers in [0, capacity-1].
     * @param fill_all  If @c true the set is initialised with every element
     *                  present; if @c false (default) the set is empty.
     *
     * @throw std::invalid_argument if @p capacity is 0.
     *
     * ### Example
     * @code
     * BinarySet empty_set(100);          // {}
     * BinarySet full_set(100, true);     // {0,1,...,99}
     * @endcode
     */
    explicit BinarySet(unsigned int capacity, bool fill_all = false) : capacity_(capacity) {
        if (capacity == 0) {
            throw std::invalid_argument("Cannot explicitly create a BinarySet with capacity 0.");
        }
        const std::size_t n_chunks = chunks_needed(capacity_);
        chunks_.assign(n_chunks, fill_all ? detail::CHUNK_ALL : detail::chunk_t{0});

        if (fill_all) {
            mask_last_chunk();
            size_ = capacity_;
        }
    }

    // ───────────────────────────────────────────────────────────────────────
    // Element mutation
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Inserts @p element into the set.
     *
     * @param element  Must be in [0, capacity-1].
     * @return @c true  if the element was not already present and was inserted.
     * @return @c false if the element was already present (no-op).
     *
     * @throw std::domain_error   if capacity() == 0.
     * @throw std::out_of_range   if element >= capacity().
     */
    auto add(unsigned int element) -> bool {
        validate_element(element);
        if (test_bit(element)) {
            return false;
        }
        set_bit(element);
        ++size_;
        return true;
    }

    /**
     * @brief Removes @p element from the set.
     *
     * @param element  Must be in [0, capacity-1].
     * @return @c true  if the element was present and has been removed.
     * @return @c false if the element was absent (no-op).
     *
     * @throw std::domain_error   if capacity() == 0.
     * @throw std::out_of_range   if element >= capacity().
     */
    auto remove(unsigned int element) -> bool {
        validate_element(element);
        if (!test_bit(element)) {
            return false;
        }
        clear_bit(element);
        --size_;
        return true;
    }

    /**
     * @brief Removes all elements from the set.
     *
     * After this call size() == 0.  capacity() is unchanged.
     */
    void clear() noexcept {
        std::ranges::fill(chunks_, detail::chunk_t{0});
        size_ = 0;
    }

    /**
     * @brief Inserts every element in [0, capacity-1] into the set.
     *
     * After this call size() == capacity().  Equivalent to constructing with
     * @c fill_all = true.
     */
    void fill() {
        std::ranges::fill(chunks_, detail::CHUNK_ALL);
        mask_last_chunk();
        size_ = capacity_;
    }

    // ───────────────────────────────────────────────────────────────────────
    // Element queries
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Tests whether @p element is in the set.
     *
     * @param element  Must be in [0, capacity-1].
     * @return @c true if present, @c false otherwise.
     *
     * @throw std::domain_error  if capacity() == 0.
     * @throw std::out_of_range  if element >= capacity().
     */
    [[nodiscard]] auto contains(unsigned int element) const -> bool {
        validate_element(element);
        return test_bit(element);
    }

    /**
     * @brief Subscript read-only access — equivalent to contains().
     *
     * @param element  Must be in [0, capacity-1].
     *
     * @throw std::domain_error  if capacity() == 0.
     * @throw std::out_of_range  if element >= capacity().
     */
    [[nodiscard]] auto operator[](unsigned int element) const -> bool { return contains(element); }

    // ───────────────────────────────────────────────────────────────────────
    // Set-membership queries
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Tests whether every element of @p other is also in this set.
     *
     * Equivalently, tests @p other ⊆ *this.
     *
     * @param other  Must have the same capacity as *this.
     * @return @c true if @p other is a subset of this set.
     *
     * @throw std::invalid_argument if capacities differ.
     *
     * @see subset_of(), intersects()
     */
    [[nodiscard]] auto superset_of(const BinarySet& other) const -> bool {
        validate_same_capacity(other);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            // Bits present in other but absent in *this → not a superset
            if ((~chunks_[i] & other.chunks_[i]) != 0) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Tests whether every element of this set is also in @p other.
     *
     * Equivalently, tests *this ⊆ @p other.
     *
     * @param other  Must have the same capacity as *this.
     * @return @c true if this set is a subset of @p other.
     *
     * @throw std::invalid_argument if capacities differ.
     *
     * @see superset_of(), intersects()
     */
    [[nodiscard]] auto subset_of(const BinarySet& other) const -> bool { return other.superset_of(*this); }

    /**
     * @brief Tests whether this set and @p other share at least one element.
     *
     * @param other  Must have the same capacity as *this.
     * @return @c true if the intersection is non-empty.
     *
     * @throw std::invalid_argument if capacities differ.
     */
    [[nodiscard]] auto intersects(const BinarySet& other) const -> bool {
        validate_same_capacity(other);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            if ((chunks_[i] & other.chunks_[i]) != 0U) {
                return true;
            }
        }
        return false;
    }

    // ───────────────────────────────────────────────────────────────────────
    // Capacity and size
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Returns the fixed universe size (maximum number of elements).
     *
     * Elements are integers in [0, capacity()-1].
     */
    [[nodiscard]] auto capacity() const noexcept -> unsigned int { return capacity_; }

    /**
     * @brief Returns the number of elements currently in the set.
     *
     * Maintained as an O(1) counter; no bit-scan is performed.
     */
    [[nodiscard]] auto size() const noexcept -> std::size_t { return size_; }

    /**
     * @brief Tests whether the set is empty (size() == 0).
     *
     * O(1) — uses the cached size counter.
     */
    [[nodiscard]] auto empty() const noexcept -> bool { return size_ == 0; }

    // ───────────────────────────────────────────────────────────────────────
    // Bulk conversion
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Returns all present elements as a sorted vector.
     *
     * Elements are returned in strictly ascending order.
     *
     * @return std::vector<unsigned int> of elements in [0, capacity-1].
     *
     * @throw std::domain_error if capacity() == 0.
     */
    [[nodiscard]] auto sparse() const -> std::vector<unsigned int> {
        if (capacity_ == 0) {
            throw std::domain_error("This BinarySet has a capacity of 0.");
        }
        return {begin(), end()};
    }

    /**
     * @brief Returns a compact visual representation of the set.
     *
     * Format: @c '[' followed by @c 'X' for each present element and @c '-'
     * for each absent element, then @c ']'.
     *
     * @par Example
     * A set with elements {0, 3, 5} and capacity 8:
     * @code
     * "[X--X-X--]"
     * @endcode
     *
     * @note Use operator<< for direct streaming; this explicit conversion is
     *       provided for contexts that require a std::string value.
     */
    [[nodiscard]] explicit operator std::string() const {
        std::string result;
        result.reserve(static_cast<std::size_t>(capacity_) + 2);
        result.push_back('[');
        for (unsigned int i = 0; i < capacity_; ++i) {
            result.push_back(test_bit_unchecked(i) ? 'X' : '-');
        }
        result.push_back(']');
        return result;
    }

    /**
     * @brief Streams the set's string representation to @p os.
     *
     * @see operator std::string()
     */
    friend auto operator<<(std::ostream& ostr, const BinarySet& bin_set) -> std::ostream& { return ostr << static_cast<std::string>(bin_set); }

    // ───────────────────────────────────────────────────────────────────────
    // Set algebra — non-mutating
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Returns the intersection of *this and @p other  (A ∩ B).
     *
     * The result contains every element present in **both** sets.
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    [[nodiscard]] auto operator&(const BinarySet& other) const -> BinarySet {
        validate_same_capacity(other);
        BinarySet result(capacity_);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            result.chunks_[i] = chunks_[i] & other.chunks_[i];
        }
        result.recalculate_size();
        return result;
    }

    /**
     * @brief Returns the union of *this and @p other  (A ∪ B).
     *
     * The result contains every element present in **either** set.
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    [[nodiscard]] auto operator|(const BinarySet& other) const -> BinarySet {
        validate_same_capacity(other);
        BinarySet result(capacity_);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            result.chunks_[i] = chunks_[i] | other.chunks_[i];
        }
        result.recalculate_size();
        return result;
    }

    /**
     * @brief Returns the set difference  (A \ B).
     *
     * The result contains elements present in *this but **not** in @p other.
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    [[nodiscard]] auto operator-(const BinarySet& other) const -> BinarySet {
        validate_same_capacity(other);
        BinarySet result(capacity_);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            result.chunks_[i] = chunks_[i] & ~other.chunks_[i];
        }
        result.recalculate_size();
        return result;
    }

    /**
     * @brief Returns the symmetric difference  (A △ B).
     *
     * The result contains elements present in **exactly one** of the two sets.
     * Equivalent to (A | B) - (A & B), but computed in a single pass.
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    [[nodiscard]] auto operator^(const BinarySet& other) const -> BinarySet {
        validate_same_capacity(other);
        BinarySet result(capacity_);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            result.chunks_[i] = chunks_[i] ^ other.chunks_[i];
        }
        result.recalculate_size();
        return result;
    }

    /**
     * @brief Returns the complement of *this  (∁A).
     *
     * The result contains every element in [0, capacity-1] that is **not** in
     * *this.
     *
     * @note Unary @c ! is used because unary @c ~ is not overloadable on class
     *       types in C++.
     */
    [[nodiscard]] auto operator!() const -> BinarySet {
        if (capacity_ == 0) {
            throw std::domain_error("Cannot complement a BinarySet with capacity 0.");
        }

        BinarySet result(capacity_);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            result.chunks_[i] = ~chunks_[i];
        }
        result.mask_last_chunk();
        result.recalculate_size();
        return result;
    }

    // ───────────────────────────────────────────────────────────────────────
    // Set algebra — mutating (in-place)
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief In-place intersection  (*this = *this ∩ other).
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    auto operator&=(const BinarySet& other) -> BinarySet& {
        validate_same_capacity(other);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            chunks_[i] &= other.chunks_[i];
        }
        recalculate_size();
        return *this;
    }

    /**
     * @brief In-place union  (*this = *this ∪ other).
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    auto operator|=(const BinarySet& other) -> BinarySet& {
        validate_same_capacity(other);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            chunks_[i] |= other.chunks_[i];
        }
        recalculate_size();
        return *this;
    }

    /**
     * @brief In-place set difference  (*this = *this \ other).
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    auto operator-=(const BinarySet& other) -> BinarySet& {
        validate_same_capacity(other);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            chunks_[i] &= ~other.chunks_[i];
        }
        recalculate_size();
        return *this;
    }

    /**
     * @brief In-place symmetric difference  (*this = *this △ other).
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    auto operator^=(const BinarySet& other) -> BinarySet& {
        validate_same_capacity(other);
        for (std::size_t i = 0; i < chunks_.size(); ++i) {
            chunks_[i] ^= other.chunks_[i];
        }
        recalculate_size();
        return *this;
    }

    // ───────────────────────────────────────────────────────────────────────
    // Equality
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Tests whether *this and @p other contain exactly the same elements.
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    [[nodiscard]] auto operator==(const BinarySet& other) const -> bool {
        validate_same_capacity(other);
        return chunks_ == other.chunks_;
    }

    /**
     * @brief Tests whether *this and @p other differ in at least one element.
     *
     * @param other  Must have the same capacity as *this.
     * @throw std::invalid_argument if capacities differ.
     */
    [[nodiscard]] auto operator!=(const BinarySet& other) const -> bool {
        validate_same_capacity(other);
        return chunks_ != other.chunks_;
    }

    // ───────────────────────────────────────────────────────────────────────
    // Iteration
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Returns a forward iterator to the smallest element in the set.
     *
     * Iterates elements in strictly ascending order.  Invalidated by any
     * mutation (add, remove, clear, fill, in-place operators).
     *
     * @return iterator pointing to the first element, or end() if the set is
     *         empty.
     */
    [[nodiscard]] auto begin() const noexcept -> iterator {
        return iterator{this, 0U};  // 2-arg begin-constructor overload
    }

    /**
     * @brief Returns the past-the-end iterator.
     *
     * @return iterator representing the end position (value == capacity()).
     */
    [[nodiscard]] auto end() const noexcept -> iterator {
        return iterator{this, capacity_, true};  // end_tag overload
    }

    // ───────────────────────────────────────────────────────────────────────
    // Iterator
    // ───────────────────────────────────────────────────────────────────────

    /**
     * @brief Forward iterator over elements present in the set.
     *
     * Advances word-by-word using std::countr_zero to skip absent elements in
     * O(1) amortised per step instead of scanning bit-by-bit.
     *
     * @note The iterator stores a raw pointer to the parent BinarySet.  Any
     *       mutation of the parent invalidates all outstanding iterators.
     */
    class iterator {
       public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = unsigned int;
        using difference_type = std::ptrdiff_t;
        using pointer = const value_type*;
        using reference = const value_type&;

        // ── End sentinel constructor ───────────────────────────────────────────
        iterator(const BinarySet* set, unsigned int /*pos — end*/, bool /*end_tag*/) noexcept
            : bs_(set), chunk_idx_(set->chunks_.size()), current_chunk_(0), current_pos_(set->capacity_) {}

        // ── Begin constructor ──────────────────────────────────────────────────
        iterator(const BinarySet* set, unsigned int /*pos — begin*/) noexcept : bs_(set), chunk_idx_(0), current_chunk_(0), current_pos_(0) {
            // Find the first non-empty chunk.
            while (chunk_idx_ < bs_->chunks_.size()) {
                current_chunk_ = bs_->chunks_[chunk_idx_];
                if (current_chunk_ != 0) {
                    break;
                }
                ++chunk_idx_;
            }
            if (chunk_idx_ >= bs_->chunks_.size()) {
                // Empty set — become end.
                current_pos_ = bs_->capacity_;
                return;
            }
            // The lowest set bit of current_chunk_ is the first element.
            current_pos_ = (static_cast<unsigned int>(chunk_idx_) * detail::CHUNK_BITS) + static_cast<unsigned int>(std::countr_zero(current_chunk_));
            // Clear that bit so the next ++ sees the remainder.
            current_chunk_ &= current_chunk_ - 1U;
        }

        auto operator++() noexcept -> iterator& {
            // Fast path: more bits remain in the current cached chunk word.
            if (current_chunk_ != 0) {
                current_pos_ =
                    (static_cast<unsigned int>(chunk_idx_) * detail::CHUNK_BITS) + static_cast<unsigned int>(std::countr_zero(current_chunk_));
                current_chunk_ &= current_chunk_ - 1U;  // clear lowest set bit
                return *this;
            }
            // Slow path: move to the next non-empty chunk.
            ++chunk_idx_;
            while (chunk_idx_ < bs_->chunks_.size()) {
                current_chunk_ = bs_->chunks_[chunk_idx_];
                if (current_chunk_ != 0) {
                    current_pos_ =
                        (static_cast<unsigned int>(chunk_idx_) * detail::CHUNK_BITS) + static_cast<unsigned int>(std::countr_zero(current_chunk_));
                    current_chunk_ &= current_chunk_ - 1U;
                    return *this;
                }
                ++chunk_idx_;
            }
            // No more chunks — become end.
            current_pos_ = bs_->capacity_;
            return *this;
        }

        auto operator++(int) noexcept -> iterator {
            const iterator tmp{*this};
            ++(*this);
            return tmp;
        }

        [[nodiscard]] auto operator*() const noexcept -> value_type { return current_pos_; }
        [[nodiscard]] auto operator==(const iterator& other) const noexcept -> bool { return current_pos_ == other.current_pos_; }
        [[nodiscard]] auto operator!=(const iterator& other) const noexcept -> bool { return !(*this == other); }

       private:
        const BinarySet* bs_;
        std::size_t chunk_idx_;          ///< Index of the chunk currently being consumed
        detail::chunk_t current_chunk_;  ///< Remaining bits of chunks_[chunk_idx_], LSB-first
        unsigned int current_pos_;       ///< Value yielded by operator*
    };

   private:
    // ───────────────────────────────────────────────────────────────────────
    // Internal helpers
    // ───────────────────────────────────────────────────────────────────────

    static auto chunks_needed(unsigned int capacity) noexcept -> std::size_t {
        return (static_cast<std::size_t>(capacity) + detail::CHUNK_BITS - 1) / detail::CHUNK_BITS;
    }

    // Low-level bit accessors — no bounds checking.
    [[nodiscard]] auto test_bit_unchecked(unsigned int element) const noexcept -> bool {
        return ((chunks_[element / detail::CHUNK_BITS] >> (element % detail::CHUNK_BITS)) & detail::chunk_t{1}) != 0U;
    }
    [[nodiscard]] auto test_bit(unsigned int element) const noexcept -> bool { return test_bit_unchecked(element); }
    void set_bit(unsigned int element) noexcept { chunks_[element / detail::CHUNK_BITS] |= detail::chunk_t{1} << (element % detail::CHUNK_BITS); }
    void clear_bit(unsigned int element) noexcept {
        chunks_[element / detail::CHUNK_BITS] &= ~(detail::chunk_t{1} << (element % detail::CHUNK_BITS));
    }

    /**
     * @brief Clears the unused high bits of the last chunk.
     *
     * When capacity is not a multiple of CHUNK_BITS, the final chunk has
     * bits beyond index (capacity-1) that must remain zero.  This is called
     * after any operation that might set those bits (fill, complement).
     */
    void mask_last_chunk() noexcept {
        if (capacity_ == 0) {
            return;
        }
        const unsigned int tail = capacity_ % detail::CHUNK_BITS;
        if (tail != 0) {
            // Keep only the lowest `tail` bits of the last word
            chunks_.back() &= (detail::chunk_t{1} << tail) - 1U;
        }
    }

    /**
     * @brief Recomputes size_ by summing std::popcount over all chunks.
     *
     * Called after bulk operations (&=, |=, -=, ^=, complement).
     * std::popcount compiles to a single POPCNT instruction on modern CPUs.
     */
    void recalculate_size() noexcept {
        size_ = 0;
        for (detail::chunk_t chunk : chunks_) {
            size_ += static_cast<std::size_t>(std::popcount(chunk));
        }
    }

    // Validation helpers
    void validate_element(unsigned int element) const {
        if (capacity_ == 0) {
            throw std::domain_error("This BinarySet has a capacity of 0.");
        }
        if (element >= capacity_) {
            throw std::out_of_range("Specified element is outside the valid range [0, capacity-1].");
        }
    }

    void validate_same_capacity(const BinarySet& other) const {
        if (capacity_ != other.capacity_) {
            throw std::invalid_argument("The two BinarySets do not have the same capacity.");
        }
    }

    // ───────────────────────────────────────────────────────────────────────
    // Data members
    // ───────────────────────────────────────────────────────────────────────
    unsigned int capacity_{0};
    std::size_t size_{0};
    std::vector<detail::chunk_t> chunks_;
};