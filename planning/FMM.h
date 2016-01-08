#ifndef PLANNING_FMM_H
#define PLANNING_FMM_H

#include <KrisLibrary/structs/arraynd.h>
#include <KrisLibrary/math/vector.h>
using namespace Math;

/** @brief Performs an N-dimensional Fast Marching Method search on a variable-
 * cost grid.
 * 
 * Input a start and goal index, an N-D grid of costs.  Goal can be empty.
 * Output: distances will contain the solution to the Eikonal equation (shortest path
 * costs), up to the cost of the shortest path from the start to the goal
 */
bool FMMSearch(const std::vector<int>& start,const std::vector<int>& goal,const ArrayND<Real>& costs,ArrayND<Real>& distances);

/** @brief Performs an N-dimensional Fast Marching Method search on a variable-
 * cost grid.
 * 
 * Input start coordinates, goal coordinates, an N-D grid of costs.  Goal can be empty.
 *    The domain of the coordinates is [0,0,...,0] -> [d1-1,...,dN-1] where costs has size d1 x ... x dN.
 * Output: distances will contain the solution to the Eikonal equation (shortest path
 * costs), up to the cost of the shortest path from the start to the goal
 */
bool FMMSearch(const Vector& start,const Vector& goal,const ArrayND<Real>& costs,ArrayND<Real>& distances);

/** @brief Performs an N-dimensional Fast Marching Method search in an N-D
 * box with a variable cost function.  
 * 
 * Input start coordinates, goal coordinates, bounds on the box, a grid discretization
 *   resolution, and a cost function.  Goal can be empty.
 * Output: distances will contain the solution to the Eikonal equation (shortest path
 * costs), up to the cost of the shortest path from the start to the goal
 */
bool FMMSearch(const Vector& start,const Vector& goal,
	       const Vector& bmin,const Vector& bmax,const Vector& res,
	       Real (*costFn) (const Vector& coords),
	       ArrayND<Real>& distances);

/** @brief Perform gradient descent on an ND field, starting from some coordinates.
 * Returns the path traced, ending at a local minimum.
 * 
 * The domain of the descent is the bounding box [0,M-1]x[0,N-1]x...x[0,Q-1] where
 * the field array is of size MxNx...xQ.
 *
 * Robust to Inf's in the array (as produced by FMMSearch)
 */
std::vector<Vector> GradientDescent(const ArrayND<Real>& field,const Vector& start);

#endif
