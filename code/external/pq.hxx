/**
 * @file pq.hpp
 * @brief Priority Queue for a set of integers
 *
 * @author Domenico Salvagnin <dominiqs at gmail dot com>
 * Copyright 2014 Domenico Salvagnin
 */

#ifndef PQ_H
#define PQ_H

#include <vector>

#ifndef _ASSERT
#include <iostream>
#define _ASSERT(cond) {                                                                                     \
if (!(cond)) {                                                                                              \
    std::cerr << "Assert check failed at " << __func__ << "(): " << __FILE__ << ":"<< __LINE__ << "\n";     \
    exit(1);                                                                                                \
}                                                                                                           \
}
#endif

template <typename score_type>
class priority_queue {
    public:
        /** Construct a priority queue for the integer form 0 to _n - 1 */
        priority_queue(int _n) : n(_n), cnt(0), data(n), prior(n), position(n, -1) {}
        /** Return the integer with minimal priority (throws exception if empty) */
        inline int top() const {
            _ASSERT(!is_empty());
            return data[0];
        }
        /** Check whether the queue is empty */
        inline bool is_empty() const { return (cnt == 0); }
        /** Checks whether an integer j is in the queue */
        inline bool has(int j) const { return (position[j] >= 0); }
        /** Clear content */
        inline void clear() {
            std::fill(position.begin(), position.begin() + cnt, -1);
            cnt = 0;
        }
        /** Restore heap structure */
        inline void heapify() {
            int start = (cnt - 2) / 2;
            while (start >= 0) {
                int j = data[start];
                position[j] = -1;
                score_type p = prior[j];
                int gap = start;
                sift_down(gap, p);
                data[gap] = j;
                position[j] = gap;
                start--;
            }
        }
        /** Insert integer j into the queue with a priority p */
        inline void push(int j, score_type p, bool mantain_heap = true) {
            _ASSERT((j >= 0) && (j < n));
            _ASSERT(position[j] == -1);
            prior[j] = p;
            // put gap at last position
            int gap = cnt++;
            if (mantain_heap) {
                // move gap down to the proper position
                sift_up(gap, p);
            }
            // fill gap with new element
            data[gap] = j;
            position[j] = gap;
        }
        /** Removes the integer with minimal priority */
        inline void pop() {
            _ASSERT(!is_empty());
            remove_at(0, true);
        }
        /** Removes integer j from the queue */
        inline void remove(int j, bool mantain_heap = true) {
            _ASSERT(position[j] >= 0);
            remove_at(position[j], mantain_heap);
            _ASSERT(position[j] == -1);
        }
        /** Changes the score of an integer j already in the queue to p */
        inline void change(int j, score_type p, bool mantain_heap = true) {
            _ASSERT(position[j] >= 0);
            int gap = position[j];
            if (mantain_heap) {
                score_type oldp = prior[gap];
                // if the priority didn't change, return immediately
                if (oldp == p)
                    return;
                if (mantain_heap) {
                    // percolate up or down depending on new priority value
                    if (p < oldp)
                        sift_up(gap, p);
                    else
                        sift_down(gap, p);
                }
                // fill gap with last element
                data[gap] = j;
                position[j] = gap;
            }
            prior[j] = p;
        }
        operator std::string() const {
            std::string repr;
            for (int k = 0; k < cnt; k++) {
                repr.append("\t").append(k).append("\t").append(data[k]).append("\t").append(prior[data[k]]).append(std::endl);
            }
            return repr;
        }
    protected:
        int n;   //< size of arrays heap and position
        int cnt; //< number of elements in the queue
        std::vector<int> data;
        std::vector<score_type> prior;
        std::vector<int> position;
        /**
         * Moves the gap up from the current position gap to the proper place for a priority of value p
         * The final position of the gap is stored in gap
         */
        inline void sift_up(int &gap, score_type p) {
            while (gap > 0) {
                int parent = (gap - 1) / 2;
                if (p < prior[data[parent]]) {
                    data[gap] = data[parent];
                    position[data[gap]] = gap;
                    gap = parent;
                }
                else
                    break;
            }
        }
        /**
         * Moves the gap down from the current position gap
         * The final position of the gap is stored in gap
         */
        inline void sift_down(int &gap, score_type p) {
            int newgap;
            int left;
            int right;
            while (true) {
                left = 2 * gap + 1;
                right = left + 1;
                if (right < cnt) {
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
                } else if (right == cnt) {
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
        inline void remove_at(int gap, bool mantain_heap) {
            int last = --cnt;
            position[data[gap]] = -1;
            if (gap == last)
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
};

#endif /* PQ_H */