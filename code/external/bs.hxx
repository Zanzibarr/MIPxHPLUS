/**
 * @file binary_set.hxx
 * @brief Compact binary set and related classes
 * @version 1.0.0
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 * @see https://github.com/Zanzibarr/binary_set
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef BINARY_SET_HXX
#define BINARY_SET_HXX

#include <algorithm>  // std::all_of, std::fill, std::find
#include <cstddef>    // std::ptrdiff_t, std::size_t
#include <iterator>   // std::forward_iterator_tag
#include <memory>     // std::unique_ptr, std::make_unique
#include <stdexcept>  // std::invalid_argument, std::domain_error, std::out_of_range
#include <string>     // std::string
#include <vector>     // std::vector

/**
 * @brief A space-efficient binary set implementation using bit manipulation.
 *
 * The binary_set stores elements as bits, where each bit position represents
 * whether an element is present (1) or absent (0) in the set. Elements are
 * unsigned integers in the range [0, capacity-1].
 *
 * Features:
 * - Compact storage: Uses 1 bit per potential element
 * - Set operations: union, intersection, difference, complement
 * - Forward iteration over elements in ascending order
 * - Range-checked element access
 *
 * Example:
 * binary_set bs(16);  // Create set with capacity 16 (elements 0-15)
 * bs.add(5);
 * bs.add(10);
 * if (bs.contains(5)) { ... }
 * for (unsigned int elem : bs){ iterate over elements }
 */
class binary_set {
   public:
    // Iterator class forward declaration for use with begin()/end()
    class iterator;

    /**
     * @brief Default constructor creates an empty set with capacity 0.
     */
    binary_set() noexcept = default;

    /**
     * @brief Constructs a binary set with specified capacity.
     *
     * @param capacity Maximum number of distinct elements (elements range [0,
     * capacity-1])
     * @param fill If true, initializes the set with all elements present; if
     * false, set is empty
     *
     * @throw std::invalid_argument If capacity is 0
     */
    explicit binary_set(unsigned int capacity, bool fill = false) : capacity_(capacity) {
        if (capacity == 0) throw std::invalid_argument("Cannot explicitly create a binary_set with capacity 0.");

        if (fill) {
            size_ = capacity_;
        }

        set_.clear();
        set_.resize((capacity_ + 7) / 8, fill ? static_cast<unsigned char>(~0u) : 0);
        // Set appropriate bits in last byte if capacity is not a multiple of 8
        if (fill && capacity_ % 8 != 0) {
            set_[set_.size() - 1] &= static_cast<unsigned char>((1u << (capacity_ % 8)) - 1);
        }
    }

    /**
     * @brief Adds an element to the set.
     *
     * @param element Element to add (must be in range [0, capacity-1])
     * @return true if element was added (wasn't already present)
     * @return false if element was already in the set
     *
     * @throw std::domain_error If this binary_set's capacity is 0
     * @throw std::out_of_range If element >= capacity
     */
    bool add(unsigned int element) {
        validate_element(element);
        if (contains(element)) return false;

        set_[element / 8] |= (1u << (element % 8));
        ++size_;
        return true;
    }

    /**
     * @brief Removes an element from the set.
     *
     * @param element Element to remove (must be in range [0, capacity-1])
     * @return true if element was removed (was present)
     * @return false if element wasn't in the set
     *
     * @throw std::domain_error If this binary_set's capacity is 0
     * @throw std::out_of_range If element >= capacity
     */
    bool remove(unsigned int element) {
        validate_element(element);
        if (!contains(element)) return false;

        set_[element / 8] &= ~(1u << (element % 8));
        --size_;
        return true;
    }

    /**
     * @brief Removes all elements from the set.
     */
    void clear() noexcept {
        std::fill(set_.begin(), set_.end(), 0);
        size_ = 0;
    }

    /**
     * @brief Adds all possible elements to the set (fills to capacity).
     */
    void fill() {
        std::fill(set_.begin(), set_.end(), static_cast<unsigned char>(~0u));
        // Clear extra bits in last byte if capacity is not a multiple of 8
        if (capacity_ % 8 != 0 && capacity_ != 0) {
            set_[set_.size() - 1] &= static_cast<unsigned char>((1u << (capacity_ % 8)) - 1);
        }
        size_ = capacity_;
    }

    /**
     * @brief Checks if an element is in the set.
     *
     * @param element Element to look up (must be in range [0, capacity-1])
     * @return true if element is present
     * @return false if element is not present
     *
     * @throw std::domain_error If this binary_set's capacity is 0
     * @throw std::out_of_range If element >= capacity
     */
    [[nodiscard]]
    bool contains(unsigned int element) const {
        validate_element(element);
        return (set_[element / 8] & (1u << (element % 8))) != 0;
    }

    /**
     * @brief Subscript operator for element lookup (read-only).
     *
     * @param element Element to check (must be in range [0, capacity-1])
     * @return true if element is present
     * @return false if element is not present
     *
     * @throw std::domain_error If this binary_set's capacity is 0
     * @throw std::out_of_range If element >= capacity
     */
    [[nodiscard]]
    bool operator[](unsigned int element) const {
        return contains(element);
    }

    /**
     * @brief Returns the capacity of this set.
     *
     * @return The maximum number of distinct elements this set can hold
     */
    [[nodiscard]]
    unsigned int capacity() const noexcept {
        return capacity_;
    }

    /**
     * @brief Returns the number of elements currently in the set.
     *
     * @return Number of elements in the set
     */
    [[nodiscard]]
    std::size_t size() const noexcept {
        return size_;
    }

    /**
     * @brief Checks if the set is empty.
     *
     * @return true if the set contains no elements
     * @return false if the set contains at least one element
     */
    [[nodiscard]]
    bool empty() const noexcept {
        return std::all_of(set_.begin(), set_.end(), [](unsigned char byte) { return byte == 0; });
    }

    /**
     * @brief Returns a vector of all elements in the set.
     *
     * @return std::vector<unsigned int> containing all elements in ascending
     * order
     *
     * @throw std::domain_error If this binary_set's capacity is 0
     */
    [[nodiscard]]
    std::vector<unsigned int> sparse() const {
        if (capacity_ == 0) throw std::domain_error("This binary set has a capacity of 0.");
        return std::vector<unsigned int>{this->begin(), this->end()};
    }

    /**
     * @brief Returns a string representation of the set.
     *
     * Format: '[' followed by 'X' for present elements and '-' for absent
     * elements, then ']' Example: "[X--X-X]" represents a set with elements {0,
     * 3, 5}
     *
     * @return std::string representation of the set
     */
    [[nodiscard]]
    explicit operator std::string() const {
        std::string result;
        result.reserve(capacity_ + 2);

        result.push_back('[');
        for (unsigned int i = 0; i < capacity_; ++i) {
            result.push_back(contains(i) ? 'X' : '-');
        }
        result.push_back(']');

        return result;
    }

    // Set operations

    /**
     * @brief Computes the intersection of two sets.
     *
     * @param other Set to intersect with
     * @return binary_set containing elements present in both sets
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    [[nodiscard]]
    binary_set operator&(const binary_set &other) const {
        validate_same_capacity(other);

        binary_set result{*this};
        for (std::size_t i = 0; i < set_.size(); ++i) {
            result.set_[i] &= other.set_[i];
        }
        result.recalculate_size();
        return result;
    }

    /**
     * @brief Performs intersection in-place.
     *
     * @param other Set to intersect with
     * @return binary_set& Reference to this set after the operation
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    binary_set &operator&=(const binary_set &other) {
        validate_same_capacity(other);

        for (std::size_t i = 0; i < set_.size(); ++i) {
            set_[i] &= other.set_[i];
        }
        recalculate_size();
        return *this;
    }

    /**
     * @brief Computes the union of two sets.
     *
     * @param other Set to union with
     * @return binary_set containing elements present in either set
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    [[nodiscard]]
    binary_set operator|(const binary_set &other) const {
        validate_same_capacity(other);

        binary_set result{*this};
        for (std::size_t i = 0; i < set_.size(); ++i) {
            result.set_[i] |= other.set_[i];
        }
        result.recalculate_size();
        return result;
    }

    /**
     * @brief Performs union in-place.
     *
     * @param other Set to union with
     * @return binary_set& Reference to this set after the operation
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    binary_set &operator|=(const binary_set &other) {
        validate_same_capacity(other);

        for (std::size_t i = 0; i < set_.size(); ++i) {
            set_[i] |= other.set_[i];
        }
        recalculate_size();
        return *this;
    }

    /**
     * @brief Computes the set difference (elements in this set but not in
     * other).
     *
     * @param other Set to subtract
     * @return binary_set containing elements in this set but not in other
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    [[nodiscard]]
    binary_set operator-(const binary_set &other) const {
        validate_same_capacity(other);

        binary_set result{*this};
        for (std::size_t i = 0; i < set_.size(); ++i) {
            result.set_[i] &= ~other.set_[i];
        }
        result.recalculate_size();
        return result;
    }

    /**
     * @brief Performs set difference in-place.
     *
     * @param other Set to subtract
     * @return binary_set& Reference to this set after the operation
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    binary_set &operator-=(const binary_set &other) {
        validate_same_capacity(other);

        for (std::size_t i = 0; i < set_.size(); ++i) {
            set_[i] &= ~other.set_[i];
        }
        recalculate_size();
        return *this;
    }

    /**
     * @brief Computes the complement of the set.
     *
     * Returns a set containing all elements in [0, capacity-1] that are not in
     * this set.
     *
     * @return binary_set The complement of this set
     */
    [[nodiscard]]
    binary_set operator!() const {
        binary_set result{capacity_};

        for (std::size_t i = 0; i < set_.size(); ++i) {
            result.set_[i] = ~set_[i];
        }

        // Mask extra bits in the last byte if needed
        if (capacity_ % 8 != 0 && capacity_ != 0) {
            result.set_[set_.size() - 1] &= static_cast<unsigned char>((1u << (capacity_ % 8)) - 1);
        }

        result.recalculate_size();
        return result;
    }

    /**
     * @brief Checks if two sets are equal.
     *
     * @param other Set to compare with
     * @return true if both sets contain exactly the same elements
     * @return false otherwise
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    [[nodiscard]]
    bool operator==(const binary_set &other) const {
        validate_same_capacity(other);
        return set_ == other.set_;
    }

    /**
     * @brief Checks if two sets are different.
     *
     * @param other Set to compare with
     * @return true if the sets differ in at least one element
     * @return false if the sets are equal
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    [[nodiscard]]
    bool operator!=(const binary_set &other) const {
        validate_same_capacity(other);
        return set_ != other.set_;
    }

    /**
     * @brief Checks if two sets have any common elements.
     *
     * @param other Set to check intersection with
     * @return true if the sets have at least one element in common
     * @return false if the sets are disjoint
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    [[nodiscard]]
    bool intersects(const binary_set &other) const {
        validate_same_capacity(other);

        for (std::size_t i = 0; i < set_.size(); ++i) {
            if (set_[i] & other.set_[i]) return true;
        }
        return false;
    }

    /**
     * @brief Checks if another set is a subset of this set.
     *
     * @param other The set to check
     * @return true if all elements in other are also in this set
     * @return false otherwise
     *
     * @throw std::invalid_argument If the sets have different capacities
     */
    [[nodiscard]]
    bool contains(const binary_set &other) const {
        validate_same_capacity(other);

        for (std::size_t i = 0; i < set_.size(); ++i) {
            // If there's any bit in other that's not in this set, other is not
            // a subset
            if ((~set_[i] & other.set_[i]) != 0) return false;
        }
        return true;
    }

    /**
     * @brief Returns an iterator to the first element in the set.
     *
     * Iterates over elements in ascending order.
     *
     * @return iterator to the first element, or end() if empty
     */
    [[nodiscard]]
    iterator begin() const noexcept {
        return {this, 0};
    }

    /**
     * @brief Returns an iterator to one past the last element.
     *
     * @return iterator representing the end position
     */
    [[nodiscard]]
    iterator end() const noexcept {
        return {this, capacity_};
    }

    /**
     * @brief Forward iterator for binary_set.
     *
     * Iterates over elements present in the set in ascending order.
     */
    class iterator {
       public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = unsigned int;
        using difference_type = std::ptrdiff_t;
        using pointer = const value_type *;
        using reference = const value_type &;

        iterator(const binary_set *bs, unsigned int pos) noexcept : bs_(bs), current_pos_(pos) {
            // Advance to first set element if starting position is not set
            if (current_pos_ < bs_->capacity() && !bs_->contains(current_pos_)) {
                ++(*this);
            }
        }

        iterator &operator++() {
            do {
                ++current_pos_;
            } while (current_pos_ < bs_->capacity() && !bs_->contains(current_pos_));
            return *this;
        }

        iterator operator++(int) {
            const iterator tmp{*this};
            ++(*this);
            return tmp;
        }

        [[nodiscard]]
        value_type operator*() const noexcept {
            return current_pos_;
        }

        [[nodiscard]]
        bool operator==(const iterator &other) const noexcept {
            return current_pos_ == other.current_pos_;
        }

        [[nodiscard]]
        bool operator!=(const iterator &other) const noexcept {
            return !(*this == other);
        }

       private:
        const binary_set *bs_;
        unsigned int current_pos_;
    };

   private:
    unsigned int capacity_{0};
    std::size_t size_{0};
    std::vector<unsigned char> set_;

    // Helper methods for validation
    void validate_element(unsigned int element) const {
        if (capacity_ == 0) {
            throw std::domain_error("This binary set has a capacity of 0.");
        }
        if (element >= capacity_) {
            throw std::out_of_range(
                "Specified element is outside of the possible "
                "range for this binary_set.");
        }
    }

    void validate_same_capacity(const binary_set &other) const {
        if (capacity_ != other.capacity_) {
            throw std::invalid_argument("The two binary_set don't have the same capacity.");
        }
    }

    // Recalculates the size of the set by counting the bits.
    void recalculate_size() noexcept {
        size_ = 0;
        for (unsigned char byte : set_) {
            // Brian Kernighan's algorithm for counting set bits
            unsigned char b = byte;
            while (b) {
                b &= b - 1;
                ++size_;
            }
        }
    }
};

/**
 * @brief Efficiently searches for subsets within a collection of binary sets.
 *
 * The bs_searcher uses a binary tree structure where each level corresponds to
 * an element position in the binary_set. This allows efficient lookup of all
 * stored sets that are subsets of a query set.
 *
 * Time complexity:
 * - add: O(capacity)
 * - remove: O(capacity)
 * - find_subsets: O(capacity * number_of_matching_paths)
 *
 * Example:
 * @code
 * bs_searcher searcher(10);  // Create searcher for sets of capacity 10
 * binary_set bs1(10);
 * bs1.add(1); bs1.add(3);
 * searcher.add(101, bs1);  // Associate ID 101 with bs1
 *
 * binary_set query(10);
 * query.add(1); query.add(3); query.add(5);
 * auto results = searcher.find_subsets(query);  // Returns {101}
 * @endcode
 */
class bs_searcher {
   private:
    struct treenode {
        std::vector<unsigned int> values;
        std::unique_ptr<treenode> left;
        std::unique_ptr<treenode> right;

        treenode() = default;
    };

   public:
    /**
     * @brief Constructs a searcher for binary_sets with the specified capacity.
     *
     * @param capacity The capacity that all managed binary_sets must have
     */
    explicit bs_searcher(unsigned int capacity) : root_(std::make_unique<treenode>()), capacity_(capacity) {}

    /**
     * @brief Adds a binary_set to the search structure.
     *
     * Multiple sets with the same value or structure can be added.
     *
     * @param value Identifier/alias for this set (need not be unique)
     * @param bs The binary_set to add
     *
     * @throw std::invalid_argument If bs has a different capacity than
     * specified in constructor
     */
    void add(unsigned int value, const binary_set &bs) {
        validate_capacity(bs);

        treenode *leaf = root_.get();

        // Traverse the tree according to the binary_set (present -> right,
        // absent
        // -> left)
        for (unsigned int i = 0; i < capacity_; ++i) {
            if (bs[i]) {
                if (!leaf->right) {
                    leaf->right = std::make_unique<treenode>();
                }
                leaf = leaf->right.get();
            } else {
                if (!leaf->left) {
                    leaf->left = std::make_unique<treenode>();
                }
                leaf = leaf->left.get();
            }
        }

        // Store the value at the leaf
        leaf->values.push_back(value);
    }

    /**
     * @brief Removes a binary_set from the search structure.
     *
     * If duplicates exist, only the first occurrence is removed.
     *
     * @param value The identifier of the set to remove
     * @param bs The binary_set to remove
     * @return true if a matching set was found and removed
     * @return false if no matching set was found
     *
     * @throw std::invalid_argument If bs has a different capacity than
     * specified in constructor
     */
    bool remove(unsigned int value, const binary_set &bs) {
        validate_capacity(bs);

        std::vector<treenode *> path;
        std::vector<bool> is_right_child;
        path.reserve(capacity_);
        is_right_child.reserve(capacity_);

        treenode *node = root_.get();

        // Traverse to the leaf node containing the value
        for (unsigned int i = 0; i < capacity_ && node; ++i) {
            path.push_back(node);
            is_right_child.push_back(bs[i]);
            node = (bs[i] ? node->right.get() : node->left.get());
        }

        // If we didn't reach a node, the element wasn't in the tree
        if (!node) return false;

        // Find and remove the value using efficient swap-and-pop
        auto it = std::find(node->values.begin(), node->values.end(), value);
        if (it == node->values.end()) return false;

        // Swap with last element and pop (more efficient than erase)
        if (it != node->values.end() - 1) {
            *it = node->values.back();
        }
        node->values.pop_back();

        // Prune empty branches from leaf to root
        if (node->values.empty() && !node->left && !node->right) {
            for (std::size_t i = path.size(); i > 0; --i) {
                treenode *parent = path[i - 1];
                bool is_right = is_right_child[i - 1];

                if (is_right) {
                    parent->right.reset();
                } else {
                    parent->left.reset();
                }

                // Stop pruning if parent has values or other children
                if (!parent->values.empty() || parent->left || parent->right) {
                    break;
                }
            }
        }

        return true;
    }

    /**
     * @brief Finds all stored sets that are subsets of the query set.
     *
     * A stored set S is a subset of query set Q if every element in S is also
     * in Q.
     *
     * @param bs The query binary_set
     * @return std::vector<unsigned int> Identifiers of all stored sets that are
     * subsets of bs
     *
     * @throw std::invalid_argument If bs has a different capacity than
     * specified in constructor
     */
    [[nodiscard]]
    std::vector<unsigned int> find_subsets(const binary_set &bs) const {
        validate_capacity(bs);

        // Use two vectors for level-by-level tree traversal
        std::vector<const treenode *> current_level;
        std::vector<const treenode *> next_level;
        current_level.reserve(capacity_);
        next_level.reserve(capacity_ * 2);

        if (root_) current_level.push_back(root_.get());

        // Traverse the tree level by level
        for (unsigned int i = 0; i < capacity_ && !current_level.empty(); ++i) {
            next_level.clear();

            for (const auto *node : current_level) {
                if (bs[i]) {
                    // If element is in query set, a subset could have it or not
                    if (node->left) next_level.push_back(node->left.get());
                    if (node->right) next_level.push_back(node->right.get());
                } else {
                    // If element is not in query set, subset must not have it
                    // either
                    if (node->left) next_level.push_back(node->left.get());
                }
            }

            current_level.swap(next_level);
        }

        // Calculate total size needed for result vector
        std::size_t total_values = 0;
        for (const auto *node : current_level) {
            total_values += node->values.size();
        }

        // Pre-allocate and collect all values from leaves
        std::vector<unsigned int> result;
        result.reserve(total_values);

        for (const auto *node : current_level) {
            result.insert(result.end(), node->values.begin(), node->values.end());
        }

        return result;
    }

   private:
    std::unique_ptr<treenode> root_;
    unsigned int capacity_;

    void validate_capacity(const binary_set &bs) const {
        if (capacity_ != bs.capacity()) {
            throw std::invalid_argument("The binary_set has an unexpected capacity.");
        }
    }
};

#endif  // BINARY_SET_HXX