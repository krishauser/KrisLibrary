#ifndef UTIL_MUXSET_H
#define UTIL_MUXSET_H

#include <vector>
using namespace std;

/** @ingroup Utils
 * @brief Calculate a minimal subset of items that has a non-empty
 * intersection with each of the given sets.
 *
 * Each bit vector in the sets vector marks whether or
 * not an item is included.
 *
 * The greedy/incremental algorithm is an approximation algorithm,
 * with degree of approximation H(s) where s is the number
 * of subsets, and H is the harmonic number.
 */
vector<bool> CalculateCoverset_BruteForce(const vector<vector<bool> >& sets);
vector<bool> CalculateCoverset_Incremental(const vector<vector<bool> >& sets);
vector<bool> CalculateCoverset_IncrementalSorted(const vector<vector<bool> >& sets);
vector<bool> CalculateCoverset_Greedy(const vector<vector<bool> >& sets);

/** @ingroup Utils
 * @brief Picks the smallest subset of the given sets that 
 * covers the whole space.
 *
 * Each bit vector in the sets vector marks whether or
 * not an item is included.
 *
 * Very similar to the coverset problem.
 *
 * The greedy algorithm is an approximation algorithm,
 * with degree of approximation H(s) where s is the number
 * of subsets is the harmonic number.
 */
vector<int> CalculateSetCover_Greedy(const vector<vector<bool> >& sets);


#endif
