/**
 * @file bs.hpp
 * @brief Compact binary set and other related classes
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef BS_HXX
#define BS_HXX

#ifndef INTCHECK_BS
	#define INTCHECK_BS true
#endif

#include <algorithm>
#include <bitset>
#include <concepts>
#include <deque>
#include <memory>
#include <optional>
#include <queue>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

/**
 * @brief binary_set class for compact binary sets.
 * The binary_set has a capacity to specify the maximum range of possible values to be added in the set [0, capacity - 1].
 * Elements either ARE or AREN'T in the set, there's no multiplicity of elements, priorities, or other things.
 * Iterating with iterators loops among elements that are in the set, in ascending order.
 * The output of the sparse() method, returns an ORDERED std::vector of elements that are in the set.
 */
class binary_set {
public:
	// Iterator class forward declaration for use with begin()/end()
	class iterator;

	// Default constructor
	binary_set() noexcept = default;

	/**
	 * @brief Construct a new binary set object with elements identified as integers from 0 to capacity-1
	 *
	 * @param capacity Capacity of the binary set (elements will range in [0, capacity-1])
	 * @param fill Full flag: if true, the set will be constructed as full (all elements present)
	 *
	 * @throw std::invalid_argument If capacity is 0
	 */
	explicit binary_set(size_t capacity, bool fill = false)
		: capacity_(capacity) {
#if INTCHECK_BS
		if (capacity == 0)
			throw std::invalid_argument("Cannot explicitly create a binary_set with capacity 0.");
#endif

		set_.resize((capacity_ + 7) / 8, fill ? static_cast<unsigned char>(~0u) : 0);

		// Set appropriate bits in last byte if capacity is not a multiple of 8
		if (fill && capacity_ % 8 != 0 && capacity_ != 0) {
			set_[set_.size() - 1] &= static_cast<unsigned char>((1u << (capacity_ % 8)) - 1);
		}
	}

	/**
	 * @brief Add an element to the set
	 *
	 * @param element Element to be added
	 * @return true If the set has changed (element wasn't in the set)
	 * @return false If the set didn't change (element was in the set)
	 *
	 * @throw std::domain_error If this binary_set's capacity is 0
	 * @throw std::out_of_range If the specified element is outside of the possible range for this binary_set
	 */
	bool add(size_t element) {
		validate_element(element);

		if (contains(element)) {
			return false;
		}

		set_[element / 8] |= (1u << (element % 8));
		return true;
	}

	/**
	 * @brief Remove an element from the set
	 *
	 * @param element Element to be removed
	 * @return true If the set has changed (element was in the set)
	 * @return false If the set didn't change (element wasn't in the set)
	 *
	 * @throw std::domain_error If this binary_set's capacity is 0
	 * @throw std::out_of_range If the specified element is outside of the possible range for this binary_set
	 */
	bool remove(size_t element) {
		validate_element(element);

		if (!contains(element)) {
			return false;
		}

		set_[element / 8] &= ~(1u << (element % 8));
		return true;
	}

	/**
	 * @brief Remove all the elements from the set
	 */
	void clear() noexcept {
		std::fill(set_.begin(), set_.end(), 0);
	}

	/**
	 * @brief Add all the elements to the set
	 */
	void fill() {
		std::fill(set_.begin(), set_.end(), static_cast<unsigned char>(~0u));

		// Set appropriate bits in last byte if capacity is not a multiple of 8
		if (capacity_ % 8 != 0 && capacity_ != 0) {
			set_[set_.size() - 1] &= static_cast<unsigned char>((1u << (capacity_ % 8)) - 1);
		}
	}

	/**
	 * @brief Check if element is in the set
	 *
	 * @param element The element to lookup
	 * @return true If the element is in the set
	 * @return false If the element isn't in the set
	 *
	 * @throw std::domain_error If this binary_set's capacity is 0
	 * @throw std::out_of_range If the specified element is outside of the possible range for this binary_set
	 */
	[[nodiscard]]
	bool contains(size_t element) const {
		validate_element(element);
		return (set_[element / 8] & (1u << (element % 8))) != 0;
	}

	/**
	 * @brief Alias for contains to maintain backward compatibility
	 */
	[[nodiscard]]
	bool operator[](size_t element) const {
		return contains(element);
	}

	/**
	 * @brief Get the capacity of this binary_set
	 *
	 * @return size_t The capacity of the binary_set
	 */
	[[nodiscard]]
	size_t capacity() const noexcept {
		return capacity_;
	}

	/**
	 * @brief Check whether the binary_set is empty
	 *
	 * @return true If the binary_set is empty
	 * @return false If the binary_set isn't empty
	 */
	[[nodiscard]]
	bool empty() const noexcept {
		return std::all_of(set_.begin(), set_.end(), [](unsigned char byte) {
			return byte == 0;
		});
	}

	/**
	 * @brief Get the sparse version of this binary_set
	 *
	 * @return std::vector<size_t> The sparse version of this binary_set (ascending order)
	 *
	 * @throw std::domain_error If this binary_set's capacity is 0
	 */
	[[nodiscard]]
	std::vector<size_t> sparse() const {
#if INTCHECK_BS
		if (capacity_ == 0)
			throw std::domain_error("This binary set has a size of 0.");
#endif

		std::vector<size_t> result;
		result.reserve(capacity_); // Worst case, but avoids reallocations

		for (auto element : *this) {
			result.push_back(element);
		}

		return result;
	}

	/**
	 * @brief Get the string representation of this binary_set
	 *
	 * @return std::string The string representation of this binary_set
	 */
	[[nodiscard]]
	explicit operator std::string() const {
		std::string repr = "[";
		repr.reserve(capacity_ + 2); // [X...X]

		for (size_t i = 0; i < capacity_; i++) {
			repr.push_back(contains(i) ? 'X' : ' ');
		}

		repr.push_back(']');
		return repr;
	}

	// Set operations

	/**
	 * @brief Perform set intersection between two sets
	 *
	 * @param other Binary set to perform intersection with
	 * @return binary_set The result of the intersection
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	binary_set operator&(const binary_set& other) const {
		validate_same_capacity(other);

		binary_set result(*this);
		for (size_t i = 0; i < set_.size(); i++) {
			result.set_[i] &= other.set_[i];
		}

		return result;
	}

	/**
	 * @brief Perform set intersection between two sets and stores it in the first set
	 *
	 * @param other Binary set to perform the intersection with
	 * @return binary_set& Reference to this set after the operation
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	binary_set& operator&=(const binary_set& other) {
		validate_same_capacity(other);

		for (size_t i = 0; i < set_.size(); i++) {
			set_[i] &= other.set_[i];
		}

		return *this;
	}

	/**
	 * @brief Perform set union between two sets
	 *
	 * @param other Binary set to perform union with
	 * @return binary_set The result of the union
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	binary_set operator|(const binary_set& other) const {
		validate_same_capacity(other);

		binary_set result(*this);
		for (size_t i = 0; i < set_.size(); i++) {
			result.set_[i] |= other.set_[i];
		}

		return result;
	}

	/**
	 * @brief Perform set union between two sets and stores it in the first set
	 *
	 * @param other Binary set to perform the union with
	 * @return binary_set& Reference to this set after the operation
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	binary_set& operator|=(const binary_set& other) {
		validate_same_capacity(other);

		for (size_t i = 0; i < set_.size(); i++) {
			set_[i] |= other.set_[i];
		}

		return *this;
	}

	/**
	 * @brief Perform set difference between two sets
	 *
	 * @param other Binary set to perform difference with
	 * @return binary_set The result of the difference
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	binary_set operator-(const binary_set& other) const {
		validate_same_capacity(other);

		binary_set result(*this);
		for (size_t i = 0; i < set_.size(); i++) {
			result.set_[i] &= ~other.set_[i];
		}

		return result;
	}

	/**
	 * @brief Perform set difference between two sets and stores it in the first set
	 *
	 * @param other Binary set to perform the difference with
	 * @return binary_set& Reference to this set after the operation
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	binary_set& operator-=(const binary_set& other) {
		validate_same_capacity(other);

		for (size_t i = 0; i < set_.size(); i++) {
			set_[i] &= ~other.set_[i];
		}

		return *this;
	}

	/**
	 * @brief Get the complementary set
	 *
	 * @return binary_set The complementary of this set
	 */
	[[nodiscard]]
	binary_set operator!() const {
		binary_set result(capacity_);

		for (size_t i = 0; i < set_.size(); i++) {
			result.set_[i] = ~set_[i];
		}

		// Mask extra bits in the last byte if needed
		if (capacity_ % 8 != 0 && capacity_ != 0) {
			result.set_[set_.size() - 1] &= static_cast<unsigned char>((1u << (capacity_ % 8)) - 1);
		}

		return result;
	}

	/**
	 * @brief Check if two binary_set are equal (if they have the same elements)
	 *
	 * @param other binary_set to compare to
	 * @return true If the binary_set are equal
	 * @return false If the binary_set are different
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	bool operator==(const binary_set& other) const {
		validate_same_capacity(other);
		return set_ == other.set_;
	}

	/**
	 * @brief Check if two binary_set are different (if they don't have the same elements)
	 *
	 * @param other binary_set to compare to
	 * @return true If the binary_set are different
	 * @return false If the binary_set are equal
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	bool operator!=(const binary_set& other) const {
		validate_same_capacity(other);
		return set_ != other.set_;
	}

	/**
	 * @brief Check if two binary_set have an intersection
	 *
	 * @param other The set to check intersection with
	 * @return true If the two sets have an intersection
	 * @return false Otherwise
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	bool intersects(const binary_set& other) const {
		validate_same_capacity(other);

		size_t i = 0;
		for (i = 0; i < set_.size() && !(set_[i] & other.set_[i]); i++)
			;
		return i != set_.size();
	}

	/**
	 * @brief Check if another binary_set is a subset of this one
	 *
	 * @param other The other binary_set
	 * @return true If other is a subset of this binary_set
	 * @return false If other isn't a subset of this binary_set
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	bool contains(const binary_set& other) const {
		validate_same_capacity(other);

		for (size_t i = 0; i < set_.size(); i++) {
			// If there's any bit in other that's not in this set, other is not a subset
			if ((~set_[i] & other.set_[i]) != 0) {
				return false;
			}
		}

		return true;
	}

	/**
	 * @brief Get the begin() iterator of this binary_set to loop through elements that are in this set
	 *
	 * @return iterator The begin() iterator
	 */
	[[nodiscard]]
	iterator begin() const noexcept {
		return iterator(this, 0);
	}

	/**
	 * @brief Get the end() iterator of this binary_set to loop through elements that are in this set
	 *
	 * @return iterator The end() iterator
	 */
	[[nodiscard]]
	iterator end() const noexcept {
		return iterator(this, capacity_);
	}

	// Iterator implementation
	class iterator {
	public:
		using iterator_category = std::forward_iterator_tag;
		using value_type = size_t;
		using difference_type = std::ptrdiff_t;
		using pointer = const value_type*;
		using reference = const value_type&;

		iterator(const binary_set* bs, size_t pos) noexcept
			: bs_(bs), current_pos_(pos) {
			// Advance to first set element if starting position is not set
			if (current_pos_ < bs_->capacity() && !bs_->contains(current_pos_)) {
				++(*this);
			}
		}

		iterator& operator++() {
			do {
				++current_pos_;
			} while (current_pos_ < bs_->capacity() && !bs_->contains(current_pos_));

			return *this;
		}

		iterator operator++(int) {
			iterator tmp = *this;
			++(*this);
			return tmp;
		}

		[[nodiscard]]
		value_type operator*() const noexcept {
			return current_pos_;
		}

		[[nodiscard]]
		bool operator==(const iterator& other) const noexcept {
			return current_pos_ == other.current_pos_;
		}

		[[nodiscard]]
		bool operator!=(const iterator& other) const noexcept {
			return !(*this == other);
		}

	private:
		const binary_set* bs_;
		size_t			  current_pos_;
	};

private:
	size_t					   capacity_ = 0;
	std::vector<unsigned char> set_;

	// Helper methods for validation
	void validate_element(size_t element) const {
#if INTCHECK_BS
		if (element >= capacity_) {
			if (capacity_ == 0)
				throw std::domain_error("This binary set has a size of 0.");
			else
				throw std::out_of_range("Specified element is outside of the possible range for this binary_set.");
		}
#endif
	}

	void validate_same_capacity(const binary_set& other) const {
#if INTCHECK_BS
		if (capacity_ != other.capacity_)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
	}
};

/**
 * @brief Class used to find, among a predefined list of binary_set, all those that are subsets of a specified one.
 */
class bs_searcher {
private:
	struct treenode {
		std::vector<size_t>		  values;
		std::unique_ptr<treenode> left;
		std::unique_ptr<treenode> right;

		treenode() = default;
	};

public:
	/**
	 * @brief Construct a new binary_set searcher object
	 *
	 * @param capacity The capacity of the binary_set that are going to be used
	 */
	explicit bs_searcher(size_t capacity)
		: root_(std::make_unique<treenode>()), capacity_(capacity) {}

	// Ensure proper cleanup with move semantics
	bs_searcher(bs_searcher&& other) noexcept = default;
	bs_searcher& operator=(bs_searcher&& other) noexcept = default;

	// Disallow copying (tree is complex to copy)
	bs_searcher(const bs_searcher&) = delete;
	bs_searcher& operator=(const bs_searcher&) = delete;

	/**
	 * @brief Add a binary_set to the search tree
	 *
	 * @param value The alias of that binary_set (not necessarily unique)
	 * @param bs The binary_set to add to the search tree (not necessarily unique)
	 *
	 * @throw std::invalid_argument If the binary_set has a different capacity than the one specified in the constructor
	 */
	void add(size_t value, const binary_set& bs) {
		validate_capacity(bs);

		treenode* leaf = root_.get();

		for (size_t i = 0; i < bs.capacity(); i++) {
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

		leaf->values.push_back(value);
	}

	/**
	 * @brief Remove a binary_set from the search tree
	 *
	 * @param value The alias of that binary_set (if duplicates are present, only the first occurrence it finds is removed)
	 * @param bs The binary_set to remove to the search tree (if duplicates are present, only the first occurrence it finds is removed)
	 * @return true If a matching binary_set was in the search tree, hence removed
	 * @return false If there was no matching binary_set in the search tree
	 *
	 * @throw std::invalid_argument If the binary_set has a different capacity than the one specified in the constructor
	 */
	bool remove(size_t value, const binary_set& bs) {
		validate_capacity(bs);

		// Helper function to check if a node is empty (no values and no children)
		auto is_empty_node = [](const treenode* node) -> bool {
			return node && node->values.empty() && !node->left && !node->right;
		};

		// Helper function to remove value from a vector
		auto remove_value = [](std::vector<size_t>& values, size_t v) -> bool {
			auto it = std::find(values.begin(), values.end(), v);
			if (it != values.end()) {
				values.erase(it);
				return true;
			}
			return false;
		};

		// Stack to keep track of nodes we've visited for pruning
		std::vector<std::pair<std::unique_ptr<treenode>*, treenode*>> path;
		path.reserve(bs.capacity());

		std::unique_ptr<treenode>* current = &root_;
		treenode*				   node = root_.get();

		// Traverse to the leaf node containing the value
		for (size_t i = 0; i < bs.capacity() && node; i++) {
			path.push_back({ current, node });

			if (bs[i]) {
				current = &(node->right);
				node = node->right.get();
			} else {
				current = &(node->left);
				node = node->left.get();
			}
		}

		// If we didn't reach a node or value isn't in the leaf, the element wasn't in the tree
		if (!node || !remove_value(node->values, value)) {
			return false;
		}

		// Prune empty branches by walking back up the path
		for (auto it = path.rbegin(); it != path.rend(); ++it) {
			auto [parent_ptr, current_node] = *it;

			// If both children are empty and this node has no values, remove it
			if (is_empty_node(current_node->left.get()) && is_empty_node(current_node->right.get()) && current_node->values.empty()) {
				// Reset unique_ptr will invoke destructor
				if (current_node->left)
					current_node->left.reset();
				if (current_node->right)
					current_node->right.reset();

				// Only reset if it's not the root node
				if (parent_ptr != &root_ || (parent_ptr == &root_ && current_node != root_.get())) {
					parent_ptr->reset();
				}
			}
		}

		return true;
	}

	/**
	 * @brief Find all subsets of a specified binary_set among the ones added to the search tree
	 *
	 * @param bs The binary_set whose subset we want to find
	 * @return std::vector<size_t> The list of aliases of the binary_set that are subsets of bs
	 *
	 * @throw std::invalid_argument If the binary_set has a different capacity than the one specified in the constructor
	 */
	[[nodiscard]]
	std::vector<size_t> find_subsets(const binary_set& bs) const {
		validate_capacity(bs);

		std::deque<treenode*> open_nodes;
		if (root_) {
			open_nodes.push_back(root_.get());
		}

		for (size_t i = 0; i < bs.capacity() && !open_nodes.empty(); i++) {
			size_t level_size = open_nodes.size();

			for (size_t j = 0; j < level_size; j++) {
				treenode* node = open_nodes.front();
				open_nodes.pop_front();

				if (bs[i]) {
					// If element is in the queried set, we can follow both paths
					if (node->left) {
						open_nodes.push_back(node->left.get());
					}
					if (node->right) {
						open_nodes.push_back(node->right.get());
					}
				} else {
					// If element is not in the queried set, we can only follow the left path (0)
					if (node->left) {
						open_nodes.push_back(node->left.get());
					}
				}
			}
		}

		std::vector<size_t> result;
		for (auto node : open_nodes) {
			result.insert(result.end(), node->values.begin(), node->values.end());
		}

		return result;
	}

private:
	std::unique_ptr<treenode> root_;
	size_t					  capacity_;

	// Helper method for validation
	void validate_capacity(const binary_set& bs) const {
		if constexpr (INTCHECK_BS) {
			if (capacity_ != bs.capacity()) {
				throw std::invalid_argument("The binary_set has an unexpected capacity.");
			}
		}
	}
};

#endif /* BS_HXX */