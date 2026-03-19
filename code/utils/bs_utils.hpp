/**
 * Utilities related to the BinarySet
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <algorithm>
#include <memory>

#include "bs.hxx"

static inline auto operator|=(BinarySet &bs, const std::vector<unsigned int> &vec) -> BinarySet & {
    for (const auto &val : vec) {
        bs.add(val);
    }
    return bs;
}

static inline auto bs_contains(const BinarySet &bs, const std::vector<unsigned int> &vec) -> bool {
    return std::ranges::all_of(vec, [&bs](unsigned int val) { return bs[val]; });
}

static inline auto bs_intersects(const BinarySet &bs, const std::vector<unsigned int> &vec) -> bool {
    return std::ranges::any_of(vec, [&bs](unsigned int val) { return bs[val]; });
}

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
     * @brief Adds a BinarySet to the search structure.
     *
     * Multiple sets with the same value or structure can be added.
     *
     * @param value Identifier/alias for this set (need not be unique)
     * @param bs The BinarySet to add
     *
     * @throw std::invalid_argument If bs has a different capacity than
     * specified in constructor
     */
    void add(unsigned int value, const BinarySet &bs) {
        validate_capacity(bs);

        treenode *leaf = root_.get();

        // Traverse the tree according to the BinarySet (present -> right,
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
     * @brief Removes a BinarySet from the search structure.
     *
     * If duplicates exist, only the first occurrence is removed.
     *
     * @param value The identifier of the set to remove
     * @param bs The BinarySet to remove
     * @return true if a matching set was found and removed
     * @return false if no matching set was found
     *
     * @throw std::invalid_argument If bs has a different capacity than
     * specified in constructor
     */
    auto remove(unsigned int value, const BinarySet &bs) -> bool {
        validate_capacity(bs);

        std::vector<treenode *> path;
        std::vector<bool> is_right_child;
        path.reserve(capacity_);
        is_right_child.reserve(capacity_);

        treenode *node = root_.get();

        // Traverse to the leaf node containing the value
        for (unsigned int i = 0; i < capacity_ && (node != nullptr); ++i) {
            path.push_back(node);
            is_right_child.push_back(bs[i]);
            node = (bs[i] ? node->right.get() : node->left.get());
        }

        // If we didn't reach a node, the element wasn't in the tree
        if (node == nullptr) {
            return false;
        }

        // Find and remove the value using efficient swap-and-pop
        auto iter = std::ranges::find(node->values, value);
        if (iter == node->values.end()) {
            return false;
        }

        // Swap with last element and pop (more efficient than erase)
        if (iter != node->values.end() - 1) {
            *iter = node->values.back();
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
     * @param bs The query BinarySet
     * @return std::vector<unsigned int> Identifiers of all stored sets that are
     * subsets of bs
     *
     * @throw std::invalid_argument If bs has a different capacity than
     * specified in constructor
     */
    [[nodiscard]]
    auto find_subsets(const BinarySet &bs) const -> std::vector<unsigned int> {
        validate_capacity(bs);

        // Use two vectors for level-by-level tree traversal
        std::vector<const treenode *> current_level;
        std::vector<const treenode *> next_level;
        current_level.reserve(capacity_);
        next_level.reserve(capacity_ * 2);

        if (root_) {
            current_level.push_back(root_.get());
        }

        // Traverse the tree level by level
        for (unsigned int i = 0; i < capacity_ && !current_level.empty(); ++i) {
            next_level.clear();

            for (const auto *node : current_level) {
                if (bs[i]) {
                    // If element is in query set, a subset could have it or not
                    if (node->left) {
                        next_level.push_back(node->left.get());
                    }
                    if (node->right) {
                        next_level.push_back(node->right.get());
                    }
                } else {
                    // If element is not in query set, subset must not have it
                    // either
                    if (node->left) {
                        next_level.push_back(node->left.get());
                    }
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

    void validate_capacity(const BinarySet &bs) const {
        if (capacity_ != bs.capacity()) {
            throw std::invalid_argument("The BinarySet has an unexpected capacity.");
        }
    }
};