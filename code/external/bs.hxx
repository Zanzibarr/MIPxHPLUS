/**
 * @file bs.hpp
 * @brief Compact binary set and other related classes
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef BS_H
#define BS_H

#include <string>
#include <vector>

#ifndef _ASSERT
	#include <iostream>
	#define _ASSERT(cond)                                                                                            \
		{                                                                                                            \
			if (!(cond)) [[unlikely]] {                                                                              \
				std::cerr << "Assert check failed at " << __func__ << "(): " << __FILE__ << ":" << __LINE__ << "\n"; \
				exit(1);                                                                                             \
			}                                                                                                        \
		}
#endif

class binary_set {
public:
	constexpr binary_set()
		: capacity(0) {}
	/** Construct a binary set for the domain of integer from 0 to _n (set _f to true to default the set to full) */
	explicit constexpr binary_set(size_t _n, bool _f = false)
		: capacity(_n) {
		this->set = std::vector<unsigned char>((this->capacity + 7) / 8, _f ? static_cast<unsigned char>(~0u) : 0);
		if (_f && this->capacity % 8 != 0 && this->capacity != 0)
			this->set[this->set.size() - 1] &= (1u << this->capacity % 8) - 1; // mask for the bits outside the size
	}
	/** Copy constructor */
	constexpr binary_set(const binary_set& _bs)
		: capacity(_bs.capacity) {
		this->set = std::vector<unsigned char>(_bs.set);
	}
	/** Add element _e to the set (if already present does nothing) */
	constexpr void add(size_t _e) {
		_ASSERT(_e < this->capacity);
		this->set[_e / 8] |= (1 << _e % 8);
	}
	/** Remove element _e from the set (if not present does nothing) */
	constexpr void remove(size_t _e) {
		_ASSERT(_e < this->capacity);
		this->set[_e / 8] &= ~(1 << _e % 8);
	}
	/** Remove all elements from the set */
	constexpr void clear() { this->set = std::vector<unsigned char>((this->capacity + 7) / 8, 0); }
	/** Add all elements to the set */
	constexpr void fill() {
		this->set = std::vector<unsigned char>((this->capacity + 7) / 8, static_cast<unsigned char>(~0u));
		if (this->capacity % 8 != 0 && this->capacity != 0)
			this->set[this->set.size() - 1] &= (1u << this->capacity % 8) - 1; // mask for the bits outside the size
	}
	/** Compute intersection of this and _bs sets and returns it */
	[[nodiscard]]
	inline binary_set operator&(const binary_set& _bs) const {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		binary_set new_bs(*this);
		for (size_t i = 0; i < this->set.size(); i++)
			new_bs.set[i] &= _bs.set[i];
		return new_bs;
	}
	/** Compute intersection of this and _bs sets and stores it in this set - returns itself */
	constexpr void operator&=(const binary_set& _bs) {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		for (size_t i = 0; i < this->set.size(); i++)
			this->set[i] &= _bs.set[i];
	}
	/** Compute the union of this and _bs sets and returns it */
	[[nodiscard]]
	inline binary_set operator|(const binary_set& _bs) const {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		binary_set new_bs(*this);
		for (size_t i = 0; i < this->set.size(); i++)
			new_bs.set[i] |= _bs.set[i];
		return new_bs;
	}
	/** Compute the union of this and _bs sets and stores it in this set - returns itself */
	constexpr void operator|=(const binary_set& _bs) {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		for (size_t i = 0; i < this->set.size(); i++)
			this->set[i] |= _bs.set[i];
	}
	/** Compute the set difference of this and _bs sets and returns it */
	[[nodiscard]]
	inline binary_set operator-(const binary_set& _bs) const {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		binary_set new_bs(*this);
		for (size_t i = 0; i < this->set.size(); i++)
			new_bs.set[i] &= ~_bs.set[i];
		return new_bs;
	}
	/** Compute the set difference of this and _bs sets and stores it in this set - returns itself */
	constexpr void operator-=(const binary_set& _bs) {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		for (size_t i = 0; i < this->set.size(); i++)
			this->set[i] &= ~_bs.set[i];
	}
	/** Compute the complementary of this set and returns it */
	[[nodiscard]]
	inline binary_set operator!() const {
		binary_set new_bs(this->capacity);
		for (size_t i = 0; i < this->set.size(); i++)
			new_bs.set[i] = ~this->set[i];
		if (this->capacity % 8 != 0 && this->capacity != 0)
			new_bs.set[this->set.size() - 1] &= (1u << this->capacity % 8) - 1; // mask for the bits outside the size
		return new_bs;
	}
	/** Check if _e is in the set */
	[[nodiscard]]
	constexpr bool operator[](size_t _e) const {
		_ASSERT(_e < this->capacity);
		return this->set[_e / 8] & (1 << _e % 8);
	}
	/** Returns the capacity of the set */
	[[nodiscard]]
	constexpr size_t size() const { return this->capacity; }
	/** Tells wether the set is empty or not */
	[[nodiscard]]
	constexpr bool empty() const { return (*this) == binary_set(this->capacity); }
	/** Confront this and _bs sets */
	[[nodiscard]]
	constexpr bool operator==(const binary_set& _bs) const {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		size_t i = 0;
		for (i = 0; i < this->set.size() && this->set[i] == _bs.set[i]; i++)
			;
		return i == this->set.size();
	}
	/** Confront this and _bs sets */
	[[nodiscard]]
	constexpr bool operator!=(const binary_set& _bs) const { return !((*this) == _bs); }
	/** Checks if this and _bs have a non-empty intersection */
	[[nodiscard]]
	constexpr bool intersects(const binary_set& _bs) const {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		size_t i = 0;
		for (i = 0; i < this->set.size() && !(this->set[i] & _bs.set[i]); i++)
			;
		return i != this->set.size();
	}
	/** Check if this set contains _bs */
	[[nodiscard]]
	constexpr bool contains(const binary_set& _bs) const {
		_ASSERT(this->capacity == _bs.capacity && this->capacity != 0 && _bs.capacity != 0);
		size_t i = 0;
		for (i = 0; i < this->set.size() && !(~this->set[i] & _bs.set[i]); i++)
			;
		return i == this->set.size();
	}
	/** Get the sparse version of the set (by default this will be sorted) */
	[[nodiscard]]
	inline std::vector<size_t> sparse() const {
		std::vector<size_t> sparse;
		sparse.reserve(this->capacity);
		std::copy(this->begin(), this->end(), std::back_inserter(sparse));
		return sparse;
	}
	/** Get the string representation of the set */
	[[nodiscard]]
	inline operator std::string() const {
		std::string repr = "[";
		for (size_t i = 0; i < this->capacity; i++)
			repr.append((*this)[i] ? "X" : " ");
		repr.append("]");
		return repr;
	}
	class iterator {
	public:
		constexpr iterator(const binary_set* _bs, size_t _s)
			: bs(_bs), c(_s) {
			if (this->c < this->bs->size() && !(*this->bs)[this->c])
				++(*this);
		}
		constexpr void operator++() {
			do {
				++this->c;
			} while (this->c < this->bs->size() && !(*this->bs)[this->c]);
		}
		[[nodiscard]]
		constexpr size_t operator*() const { return this->c; }
		[[nodiscard]]
		constexpr bool operator!=(const iterator& _it) const { return this->c != _it.c; }

	protected:
		const binary_set* bs;
		size_t			  c;
	};
	/** begin iterator */
	[[nodiscard]]
	constexpr iterator begin() const { return iterator(this, 0); }
	/** end iterator */
	[[nodiscard]]
	constexpr iterator end() const { return iterator(this, this->capacity); }

private:
	size_t					   capacity;
	std::vector<unsigned char> set;
};

/**
 * @brief Subset searcher to find all subsets of a binary set among a given domain of binary sets
 *
 * @author Matteo Zanella <matteo.zanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#include <queue>

class bs_searcher {
public:
	/** Construct a binary subset searcher for binary set or size _c */
	explicit inline bs_searcher(size_t _c) : root(new treenode()), capacity(_c) {}
	// TODO: Add copy constructor and operator=
	/** Add to the search a new binary set _bs and assign to it value _v */
	inline void add(size_t _v, const binary_set& _bs) {
		_ASSERT(this->capacity == _bs.size());
		treenode* leaf = this->root;
		for (size_t i = 0; i < _bs.size(); i++) {
			if (_bs[i]) {
				if (leaf->right == nullptr)
					leaf->right = new treenode();
				leaf = leaf->right;
			} else {
				if (leaf->left == nullptr)
					leaf->left = new treenode();
				leaf = leaf->left;
			}
		}
		leaf->values.push_back(_v);
	}
	inline void remove(size_t _v, const binary_set& _bs) {
		_ASSERT(this->capacity == _bs.size());
		// Helper function to check if a node is empty (no values and no children)
		auto is_empty_node = [](treenode* node) -> bool { return node != nullptr && node->values.empty() && node->left == nullptr && node->right == nullptr; };
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
		std::vector<std::pair<treenode**, treenode*>> path;
		path.reserve(_bs.size());
		treenode** current = &this->root;
		treenode*  node = this->root;
		// Traverse to the leaf node containing the value
		for (size_t i = 0; i < _bs.size() && node != nullptr; i++) {
			path.push_back({ current, node });
			if (_bs[i]) {
				current = &(node->right);
				node = node->right;
			} else {
				current = &(node->left);
				node = node->left;
			}
		}
		// If we didn't reach a node or value isn't in the leaf, the element wasn't in the tree
		if (node == nullptr || !remove_value(node->values, _v))
			return;
		// Prune empty branches by walking back up the path
		while (!path.empty()) {
			auto [parent_ptr, current_node] = path.back();
			path.pop_back();
			// If both children are empty and this node has no values, remove it
			if (is_empty_node(current_node->left) && is_empty_node(current_node->right) && current_node->values.empty()) {
				delete current_node->left;
				delete current_node->right;
				delete current_node;
				*parent_ptr = nullptr;
			}
		}
	}
	/** Find all binary subsets of _bs among the one added in the search. @returns a std::vector<size_t> with all the keys of the binary subsets found */
	[[nodiscard]]
	inline std::vector<size_t> find_subsets(const binary_set& _bs) const {
		std::deque<treenode*> open_nodes;
		open_nodes.push_back(this->root);
		for (size_t i = 0; i < _bs.size() && !open_nodes.empty(); i++) {
			size_t level_size = open_nodes.size();
			for (size_t _ = 0; _ < level_size; _++) {
				treenode* node = open_nodes.front();
				open_nodes.pop_front();
				if (_bs[i]) {
					if (node->left != nullptr)
						open_nodes.push_back(node->left);
					if (node->right != nullptr)
						open_nodes.push_back(node->right);
				} else {
					if (node->left != nullptr)
						open_nodes.push_back(node->left);
				}
			}
		}
		std::vector<size_t> result;
		for (auto node : open_nodes)
			result.insert(result.end(), node->values.begin(), node->values.end());
		return result;
	}
	inline ~bs_searcher() {
		if (this->root == nullptr)
			return;
		std::deque<treenode*> rem_nodes;
		rem_nodes.push_back(this->root);
		while (!rem_nodes.empty()) {
			treenode* node = rem_nodes.front();
			rem_nodes.pop_front();
			if (node->left != nullptr)
				rem_nodes.push_back(node->left);
			if (node->right != nullptr)
				rem_nodes.push_back(node->right);
			delete node;
		}
	}

private:
	struct treenode {
		std::vector<size_t> values;
		treenode*			left;
		treenode*			right;
		treenode() {
			this->left = nullptr;
			this->right = nullptr;
		}
	}*	   root;
	size_t capacity;
};

#endif /* BS_H */