/**
 * @file pq.hxx
 * @brief Priority Queue for a set of integers
 *
 * @author Domenico Salvagnin <dominiqs at gmail dot com>
 * Copyright 2014 Domenico Salvagnin
 */

#ifndef PQ_H
#define PQ_H

#ifndef INTCHECK_PQ
	#define INTCHECK_PQ true
#endif

#include <stdexcept> // For std::domain_error
#include <string>	 // For std::string
#include <vector>	 // For std::vector

template <typename score_type>
class priority_queue {
public:
	/** Construct a priority queue for the integer form 0 to n - 1 */
	explicit priority_queue(const size_t n)
		: n(n), cnt(0), data(n), prior(n), position(n, -1) {}
	/** Return the integer with minimal priority (throws exception if empty) */
	[[nodiscard]]
	size_t top() const {
		check_empty();
		return data[0];
	}
	/** Check whether the queue is empty */
	[[nodiscard]]
	bool empty() const { return cnt == 0; }
	/** Checks whether an integer j is in the queue */
	[[nodiscard]]
	bool has(const size_t j) const { return (position[j] >= 0); }
	/** Clear content */
	void clear() {
		std::fill_n(position.begin(), cnt, -1);
		cnt = 0;
	}
	/** Restore heap structure */
	void heapify() {
		int start{ (cnt - 2) / 2 };
		while (start >= 0) {
			score_type j = data[start];
			position[j] = -1;
			score_type p = prior[j];
			int		   gap{ start };
			sift_down(gap, p);
			data[gap] = j;
			position[j] = gap;
			start--;
		}
	}
	/** Insert integer j into the queue with a priority p */
	void push(const size_t j, const score_type p, const bool mantain_heap = true) {
		validate_element(j);
#if INTCHECK_PQ
		if (position[j] != -1)
			throw std::invalid_argument("Element was already in the priority queue.");
#endif
		prior[j] = p;
		// put gap at last position
		int gap{ static_cast<int>(cnt++) };
		if (mantain_heap) // move gap down to the proper position
			sift_up(gap, p);
		// fill gap with new element
		data[gap] = j;
		position[j] = gap;
	}
	/** Removes the integer with minimal priority */
	void pop() {
		check_empty();
		remove_at(0, true);
	}
	/** Removes integer j from the queue */
	void remove(const size_t j, const bool mantain_heap = true) {
#if INTCHECK_PQ
		if (position[j] < 0)
			throw std::invalid_argument("Element wasn't in the priority queue.");
#endif
		remove_at(position[j], mantain_heap);
#if INTCHECK_PQ
		if (position[j] != -1)
			throw std::invalid_argument("Element hasn't been removed from the priority queue.");
#endif
	}
	/** Changes the score of an integer j already in the queue to p */
	void change(const size_t j, const score_type p, const bool mantain_heap = true) {
#if INTCHECK_PQ
		if (position[j] < 0)
			throw std::invalid_argument("Element wasn't in the priority queue.");
#endif
		int gap{ position[j] };
		if (mantain_heap) {
			score_type oldp = prior[gap];
			// if the priority didn't change, return immediately
			if (oldp == p)
				return;

			// percolate up or down depending on new priority value
			if (p < oldp)
				sift_up(gap, p);
			else
				sift_down(gap, p);

			// fill gap with last element
			data[gap] = j;
			position[j] = gap;
		}
		prior[j] = p;
	}
	[[nodiscard]]
	explicit operator std::string() const {
		std::string repr;
		for (size_t k = 0; k < cnt; k++)
			repr.append("\t").append(std::to_string(k)).append("\t").append(std::to_string(data[k])).append("\t").append(std::to_string(prior[data[k]])).append("\n");
		return repr;
	}

private:
	size_t					n;	 //< size of arrays heap and position
	size_t					cnt; //< number of elements in the queue
	std::vector<size_t>		data;
	std::vector<score_type> prior;
	std::vector<int>		position;
	/**
	 * Moves the gap up from the current position gap to the proper place for a priority of value p
	 * The final position of the gap is stored in gap
	 */
	void sift_up(int& gap, const score_type p) {
		while (gap > 0) {
			const int parent{ (gap - 1) / 2 };
			if (p < prior[data[parent]]) {
				data[gap] = data[parent];
				position[data[gap]] = gap;
				gap = parent;
			} else
				break;
		}
	}
	/**
	 * Moves the gap down from the current position gap
	 * The final position of the gap is stored in gap
	 */
	void sift_down(int& gap, const score_type p) {
		int newgap;
		while (true) {
			int left{ 2 * gap + 1 };
			int right{ left + 1 };
			if (right < static_cast<int>(cnt)) {
				if (prior[data[left]] < prior[data[right]])
					newgap = left;
				else
					newgap = right;
				if (p < prior[data[newgap]])
					break;
				else {
					data[gap] = data[newgap];
					position[data[gap]] = gap;
					gap = newgap;
				}
			} else if (right == static_cast<int>(cnt)) {
				newgap = left;
				if (p < prior[data[newgap]])
					break;
				data[gap] = data[newgap];
				position[data[gap]] = gap;
				gap = newgap;
			} else
				break;
		}
	}
	/** Removes the integer in position gap */
	void remove_at(int gap, const bool mantain_heap) {
		const size_t last{ --cnt };
		position[data[gap]] = -1;
		if (gap == static_cast<int>(last))
			return;
		score_type gapP = prior[data[gap]];
		// get last element priority
		score_type lastP = prior[data[last]];
		if (mantain_heap) {
			// we have a gap in position pos,
			// that we will fill with an element of priority lastP
			// Depending this value, we may need to percolate up or down!!!
			if (lastP < gapP)
				sift_up(gap, lastP);
			else
				sift_down(gap, lastP);
		}
		// fill gap with last element
		data[gap] = data[last];
		position[data[gap]] = gap;
	}
	void check_empty() const {
#if INTCHECK_PQ
		if (empty())
			throw std::domain_error("The priority queue is empty");
#endif
	}
	void validate_element(const size_t j) const {
#if INTCHECK_PQ
		if (j >= n)
			throw std::invalid_argument("Element is outside of predefined range.");
#endif
	}
};

#endif /* PQ_H */