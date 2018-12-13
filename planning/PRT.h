#ifndef ROBOTICS_PRT_H
#define ROBOTICS_PRT_H

#include "CSpace.h"

/** @ingroup MotionPlanning
 * @brief The PRT (Probabilistic Roadmap of Trees) planner.
 *
 * Sometimes called SRT (Sampling-Based Roadmap of Trees).
 */
class PRTPlanner
{
 public:
  typedef Graph::Tree<Config,EdgePlanner> Node;
  struct Tree
  {
    Node* root;
    Config centroid;
  };
  struct TreeEdge
  {
    Node* start, *end;
    MilestonePath path;
  };

  PRTPlanner();
  virtual ~PRTPlanner();
  virtual void Cleanup();
  virtual void Seed();
  virtual void AddAndGrowTree(const Config& x);
  virtual void Expand();
  virtual bool ConnectTrees(int i,int j);

  Graph::Graph<Tree,TreeEdge> treeGraph;

  ///these flags govern the number of seeded trees and their size
  int numSeedTrees,seedTreeSize;
  ///these flags govern the choice of trees to connect per expansion operation
  int numClosestTreesToConnect,numRandomTreesToConnect;
  int numStraightLineAttempts,numSingleQueryIterations;
};

#endif
