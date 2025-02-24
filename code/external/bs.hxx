/**
 * @file bs.hpp
 * @brief Compact binary set and other related classes
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef BS_H
#define BS_H

#ifndef INTCHECK_BS
	#define INTCHECK_BS true
#endif

#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * @brief binary_set class for compact binary sets.
 * The binary_set has a capacity to specify the maximum range of possible values to be added in the set [0, capacity - 1].
 * Elements either ARE or AREN'T in the set, there's no multiplicity of elements, priorities, or other things.
 * Iterating with iterators loops among elements that are in the set, in ascending order.
 * The output of the sparse() method, returns an ORDERED std::vector of elements that are in the set.
 *
 */
class binary_set {
public:
	inline binary_set()
		: n(0) {}
	/**
	 * @brief Construct a new binary set object with elements identified as integers from 0 to _n-1
	 *
	 * @param _n Capacity of the binary set (elements will range in [0, _n-1])
	 * @param _f Full flag : if set to true the size will be constructed as full
	 *
	 * @throw std::invalid_argument If _n is 0
	 */
	explicit inline binary_set(size_t _n, bool _f = false)
		: n(_n) {
#if INTCHECK_BS
		if (_n == 0)
			throw std::invalid_argument("Cannot explicitly create a binary_set with capacity 0.");
#endif
		this->set = std::vector<unsigned char>((this->n + 7) / 8, _f ? static_cast<unsigned char>(~0u) : 0);
		if (_f && this->n % 8 != 0 && this->n != 0)
			this->set[this->set.size() - 1] &= (1u << this->n % 8) - 1; // mask for the bits outside the size
	}

	/**
	 * @brief Construct a new binary set object
	 *
	 * @param _bs Binary set to copy (deep copy)
	 */
	inline binary_set(const binary_set& _bs)
		: n(_bs.n) {
		this->set = std::vector<unsigned char>(_bs.set);
	}

	/**
	 * @brief Add an element to the set
	 *
	 * @param _e Element to be added
	 * @return true If the set has changed (_e wasn't in the set)
	 * @return false If the set didn't change (_e was in the set)
	 *
	 * @throw std::domain_error If this binary_set's capacity is 0
	 * @throw std::out_of_range If the specified element is outside of the possible range for this binary_set
	 */
	inline bool add(size_t _e) {
#if INTCHECK_BS
		if (_e >= this->n) {
			if (this->n == 0)
				throw std::domain_error("This binary set has a size of 0.");
			else
				throw std::out_of_range("Specified element is outside of the possible range for this binary_set.");
		}
#endif
		if ((*this)[_e])
			return false;
		this->set[_e / 8] |= (1 << _e % 8);
		return true;
	}

	/**
	 * @brief Remove an element from the set
	 *
	 * @param _e Element to be removed
	 * @return true If the set has changed (_e was in the set)
	 * @return false If the set didn't change (_e wasn't in the set)
	 *
	 * @throw std::domain_error If this binary_set's capacity is 0
	 * @throw std::out_of_range If the specified element is outside of the possible range for this binary_set
	 */
	inline bool remove(size_t _e) {
#if INTCHECK_BS
		if (_e >= this->n) {
			if (this->n == 0)
				throw std::domain_error("This binary set has a size of 0.");
			else
				throw std::out_of_range("Specified element is outside of the possible range for this binary_set.");
		}
#endif
		if (!(*this)[_e])
			return false;
		this->set[_e / 8] &= ~(1 << _e % 8);
		return true;
	}

	/**
	 * @brief Remove all the elements from the set
	 *
	 */
	inline void clear() { this->set = std::vector<unsigned char>((this->n + 7) / 8, 0); }

	/**
	 * @brief Add all the elements to the set
	 *
	 */
	inline void fill() {
		this->set = std::vector<unsigned char>((this->n + 7) / 8, static_cast<unsigned char>(~0u));
		if (this->n % 8 != 0 && this->n != 0)
			this->set[this->set.size() - 1] &= (1u << this->n % 8) - 1; // mask for the bits outside the size
	}

	/**
	 * @brief Perform set intersection between two sets.
	 * CAREFUL: This is not the same as the method binary_set::intersects()
	 *
	 * @param _bs Binary set to perform intersection with
	 * @return binary_set The result of the intersection
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	inline binary_set operator&(const binary_set& _bs) const {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		binary_set new_bs(*this);
		for (size_t i = 0; i < this->set.size(); i++)
			new_bs.set[i] &= _bs.set[i];
		return new_bs;
	}

	/**
	 * @brief Perform set intersection between two sets and stores it in the first set (bs1 &= bs2 it's the same of bs1 = bs1 & bs2, but the intersection happens in-place)
	 * <p>CAREFUL: This is not the same as the method binary_set::intersects()<\p>
	 *
	 * @param _bs Binary set to perform the intersection with
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	inline void operator&=(const binary_set& _bs) {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		for (size_t i = 0; i < this->set.size(); i++)
			this->set[i] &= _bs.set[i];
	}

	/**
	 * @brief Perform set union between two sets
	 *
	 * @param _bs Binary set to perform union with
	 * @return binary_set The result of the union
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	inline binary_set operator|(const binary_set& _bs) const {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		binary_set new_bs(*this);
		for (size_t i = 0; i < this->set.size(); i++)
			new_bs.set[i] |= _bs.set[i];
		return new_bs;
	}

	/**
	 * @brief Perform set union between two sets and stores it in the first set (bs1 |= bs2 it's the same of bs1 = bs1 | bs2, but the union happens in-place)
	 *
	 * @param _bs Binary set to perform the union with
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	inline void operator|=(const binary_set& _bs) {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		for (size_t i = 0; i < this->set.size(); i++)
			this->set[i] |= _bs.set[i];
	}

	/**
	 * @brief Perform set difference between two sets
	 *
	 * @param _bs Binary set to perform difference with
	 * @return binary_set The result of the difference
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	inline binary_set operator-(const binary_set& _bs) const {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		binary_set new_bs(*this);
		for (size_t i = 0; i < this->set.size(); i++)
			new_bs.set[i] &= ~_bs.set[i];
		return new_bs;
	}

	/**
	 * @brief Perform set difference between two sets and stores it in the first set (bs1 -= bs2 it's the same of bs1 = bs1 - bs2, but the difference happens in-place)
	 *
	 * @param _bs Binary set to perform the difference with
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	inline void operator-=(const binary_set& _bs) {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		for (size_t i = 0; i < this->set.size(); i++)
			this->set[i] &= ~_bs.set[i];
	}

	/**
	 * @brief Get the complementary set
	 *
	 * @return binary_set The complementary of this set
	 */
	[[nodiscard]]
	inline binary_set operator!() const {
		binary_set new_bs(this->n);
		for (size_t i = 0; i < this->set.size(); i++)
			new_bs.set[i] = ~this->set[i];
		if (this->n % 8 != 0 && this->n != 0)
			new_bs.set[this->set.size() - 1] &= (1u << this->n % 8) - 1; // mask for the bits outside the size
		return new_bs;
	}

	/**
	 * @brief Check if _e is in the set
	 *
	 * @param _e The element to lookup
	 * @return true If the element is in the set
	 * @return false If the element isn't in the set
	 *
	 * @throw std::domain_error If this binary_set's capacity is 0
	 * @throw std::out_of_range If the specified element is outside of the possible range for this binary_set
	 */
	[[nodiscard]]
	inline bool operator[](size_t _e) const {
#if INTCHECK_BS
		if (_e >= this->n) {
			if (this->n == 0)
				throw std::domain_error("This binary set has a size of 0.");
			else
				throw std::out_of_range("Specified element is outside of the possible range for this binary_set.");
		}
#endif
		return this->set[_e / 8] & (1 << _e % 8);
	}

	/**
	 * @brief Get the capacity of this binary_set
	 *
	 * @return size_t The capacity of the binary_set
	 */
	[[nodiscard]]
	inline size_t capacity() const { return this->n; }

	/**
	 * @brief Check wether the binary_set is empty
	 *
	 * @return true If the binary_set is empty
	 * @return false If the binary_set isn't empty
	 */
	[[nodiscard]]
	inline bool empty() const { return (*this) == binary_set(this->n); }

	/**
	 * @brief Check if two binary_set are equal (if they have the same elements)
	 *
	 * @param _bs binary_set to compare to
	 * @return true If the binary_set are equal
	 * @return false If the binary_set are different
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	inline bool operator==(const binary_set& _bs) const {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		size_t i = 0;
		for (i = 0; i < this->set.size() && this->set[i] == _bs.set[i]; i++)
			;
		return i == this->set.size();
	}

	/**
	 * @brief Check if two binary_set are different (if they have different elements)
	 *
	 * @param _bs binary_set to compare to
	 * @return true If the binary_set are different
	 * @return false If the binary_set are equal
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	inline bool operator!=(const binary_set& _bs) const { return !((*this) == _bs); }

	/**
	 * @brief Check if two binary_set have an intersection
	 *
	 * @param _bs The set to check intersection with
	 * @return true If the two sets have an intersection
	 * @return false Otherwise
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	inline bool intersects(const binary_set& _bs) const {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		size_t i = 0;
		for (i = 0; i < this->set.size() && !(this->set[i] & _bs.set[i]); i++)
			;
		return i != this->set.size();
	}

	/**
	 * @brief Check if another binary_set is a subset of this one
	 *
	 * @param _bs The other binary_set
	 * @return true If _bs is a subset of this binary_set
	 * @return false If _bs isn't a subset of this binary_set
	 *
	 * @throw std::invalid_argument If the two binary_set don't have the same capacity
	 */
	[[nodiscard]]
	inline bool contains(const binary_set& _bs) const {
#if INTCHECK_BS
		if (this->n != _bs.n)
			throw std::invalid_argument("The two binary_set don't have the same capacity.");
#endif
		size_t i = 0;
		for (i = 0; i < this->set.size() && !(~this->set[i] & _bs.set[i]); i++)
			;
		return i == this->set.size();
	}

	/**
	 * @brief Get the sparse version of this binary_set
	 *
	 * @return std::vector<size_t> The sparse version of this binary_set (ascending order)
	 *
	 * @throw std::domain_error If this binary_set's capacity is 0
	 */
	[[nodiscard]]
	inline std::vector<size_t> sparse() const {
#if INTCHECK_BS
		if (this->n == 0)
			throw std::domain_error("This binary set has a size of 0.");
#endif
		std::vector<size_t> sparse;
		sparse.reserve(this->n);
		for (auto x : *this)
			sparse.push_back(x);
		return sparse;
	}

	/**
	 * @brief Get the string representation of this binary_set
	 *
	 * @return std::string The sting representation of this binary_set
	 */
	[[nodiscard]]
	inline operator std::string() const {
		std::string repr = "[";
		for (size_t i = 0; i < this->n; i++)
			repr.append((*this)[i] ? "X" : " ");
		repr.append("]");
		return repr;
	}

	class iterator {
	public:
		inline iterator(const binary_set* _bs, size_t _s)
			: bs(_bs), c(_s) {
			if (this->c < this->bs->capacity() && !(*this->bs)[this->c])
				++(*this);
		}
		inline void operator++() {
			do {
				++this->c;
			} while (this->c < this->bs->capacity() && !(*this->bs)[this->c]);
		}
		[[nodiscard]]
		inline size_t operator*() const { return this->c; }
		[[nodiscard]]
		inline bool operator!=(const iterator& _it) const { return this->c != _it.c; }

	private:
		const binary_set* bs;
		size_t			  c;
	};

	/**
	 * @brief Get the begin() iterator of this binary_set to loop through elements that are in this set
	 *
	 * @return iterator The begin() iterator
	 */
	[[nodiscard]]
	inline iterator begin() const { return iterator(this, 0); }

	/**
	 * @brief Get the end() iterator of this binary_set to loop through elements that are in this set
	 *
	 * @return iterator The end() iterator
	 */
	[[nodiscard]]
	inline iterator end() const { return iterator(this, this->n); }

private:
	size_t					   n;
	std::vector<unsigned char> set;
};

/**
 * @brief Class used to find, among a predefined list of binary_set, all those that are subsets of a specified one.
 *
 */
class bs_searcher {
public:
	/**
	 * @brief Construct a new binary_set searcher object
	 *
	 * @param _c The capacity of the binary_set that are gonna be used
	 */
	explicit inline bs_searcher(size_t _c)
		: root(new treenode()), capacity(_c) {}

	/**
	 * @brief Add a binary_set to the search tree
	 *
	 * @param _v The alias of that binary_set (not necessarily unique)
	 * @param _bs The binary_set to add to the search tree (not necessarily unique)
	 *
	 * @throw std::invalid_argument If the binary_set has a different capacity than the one specified in the constructor
	 */
	inline void add(size_t _v, const binary_set& _bs) {
#if INTCHECK_BS
		if (this->capacity != _bs.capacity())
			throw std::invalid_argument("The binary_set has an unexpected capacity.");
#endif
		treenode* leaf = this->root;
		for (size_t i = 0; i < _bs.capacity(); i++) {
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

	/**
	 * @brief Remove a binary_set to the search tree
	 *
	 * @param _v The alias of that binary_set (if duplicates are present, only the first occurrence it finds is removed)
	 * @param _bs The binary_set to remove to the search tree (if duplicates are present, only the first occurrence it finds is removed)
	 * @return true If a matching binary_set was in the search tree, hence removed
	 * @return false If there was no matching binary_set in the search tree
	 *
	 * @throw std::invalid_argument If the binary_set has a different capacity than the one specified in the constructor
	 */
	inline bool remove(size_t _v, const binary_set& _bs) {
#if INTCHECK_BS
		if (this->capacity != _bs.capacity())
			throw std::invalid_argument("The binary_set has an unexpected capacity.");
#endif
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
		path.reserve(_bs.capacity());
		treenode** current = &this->root;
		treenode*  node = this->root;
		// Traverse to the leaf node containing the value
		for (size_t i = 0; i < _bs.capacity() && node != nullptr; i++) {
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
			return false;
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
		return true;
	}

	/**
	 * @brief Find all subsets of a specified binary_set among the ones added to the search tree
	 *
	 * @param _bs The binary_set whose subset we want to find
	 * @return std::vector<size_t> The list of aliases of the binary_set that are subsets of _bs
	 *
	 * @throw std::invalid_argument If the binary_set has a different capacity than the one specified in the constructor
	 */
	[[nodiscard]]
	inline std::vector<size_t> find_subsets(const binary_set& _bs) const {
#if INTCHECK_BS
		if (this->capacity != _bs.capacity())
			throw std::invalid_argument("The binary_set has an unexpected capacity.");
#endif
		std::deque<treenode*> open_nodes;
		open_nodes.push_back(this->root);
		for (size_t i = 0; i < _bs.capacity() && !open_nodes.empty(); i++) {
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