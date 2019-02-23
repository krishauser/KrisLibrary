#ifndef MILESTONE_PATH_H
#define MILESTONE_PATH_H

#include "CSpace.h"
#include "EdgePlanner.h"
#include <list>

//forward declaration
class ObjectiveFunctionalBase;

/** @ingroup MotionPlanning
 * @brief A sequence of locally planned paths between milestones
 *
 * Milestones are indexed M0...Mn+1, such that segment k goes from Mk to Mk+1
 *
 * Note that the parameterization of the path is uniform on the range [0,1].
 */
class MilestonePath : public Interpolator
{
public:
  MilestonePath();
  ~MilestonePath();

  virtual void Eval(Real u,Config& q) const { Eval2(u,q); }
  virtual Real Length() const;
  virtual const Config& Start() const { return edges.front()->Start(); }
  virtual const Config& End() const { return edges.back()->End(); }

  const Config& GetMilestone(int milestone) const;
  void SetMilestone(int milestone,const Config& x);
  /// Sets the milestone to x only if x and the paths to adjoining 
  /// milestones are feasible 
  bool CheckSetMilestone(int milestone,const Config& x);
  inline CSpace* Space(int i=0) const { assert(!edges.empty()); return edges[i]->Space(); }
  inline int NumMilestones() const { return (int)edges.size()+1; }
  inline int NumEdges() const { return (int)edges.size(); }
  bool IsValid();
  /// Adds the path onto the end of this one
  void Concat(const MilestonePath& path);
  /// Create the path that connects the milestones in the given workspace
  void CreateEdgesFromMilestones(CSpace* space,const std::vector<Config>& milestones);
  /// Checks the feasibility of all edges, returns true if they all succeed
  bool InitializeEdgePlans();
  /// Checks the feasibility of all milestones and edges, returns true if so
  bool IsFeasible();
  /// Supposing all milestones have equal time spacings, evaluates the
  /// point on the path at time t in [0,1].
  /// Returns the edge of time t.
  int Eval2(Real t, Config& c) const;
  /// Tries to shorten the path by connecting subsequent milestones.
  /// Returns # of shortcuts made.
  int Shortcut();
  /// Tries to reduce path cost by connecting subsequent milestones.
  /// Returns # of shortcuts made.
  int Shortcut(ObjectiveFunctionalBase* objective);
  /// Tries to shorten the path by connecting random points
  /// with a shortcut, for numIters iterations.  Returns # of shortcuts
  int Reduce(int numIters);
  /// Tries to reduce the path cost by connecting random points
  /// with a shortcut, for numIters iterations.  Returns # of shortcuts
  int Reduce(int numIters,ObjectiveFunctionalBase* objective);
  /// Replaces the section of the path between milestones
  /// start and goal with a new path.  If the index is negative,
  /// erases the corresponding start/goal milestones too.
  void Splice(int start,int goal,const MilestonePath& path); 
  /// Discretizes the path such that each edge is no longer than h.
  /// Assumes straight-line path segments.
  void Discretize(Real h);
  /// Discretizes only the given edge.  Returns the number of new segments.
  int DiscretizeEdge(int e,Real h);
  /// Discretizes the given edge with the specified interpolation.
  void DiscretizeEdge(int e,const std::vector<Real>& u);
  /// Loads the intermediate milestones, and creates the edges from the given space
  bool Load(std::istream& in,CSpace* space);
  /// Saves the intermediate milestones
  bool Save(std::ostream& out);

  std::vector<EdgePlannerPtr> edges;
};

#endif
