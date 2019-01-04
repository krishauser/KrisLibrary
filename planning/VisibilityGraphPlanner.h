#ifndef VISIBILITY_GRAPH_PLANNER_H
#define VISIBILITY_GRAPH_PLANNER_H

#include "Geometric2DCSpace.h"
#include "Path.h"
#include <KrisLibrary/graph/UndirectedGraph.h>

struct VisibilityGraphPlanner
{
  VisibilityGraphPlanner(Geometric2DCSpace* space);
  void Init();
  bool Plan(const Vector2& a,const Vector2& b,std::vector<Vector2>& path);
  bool Plan(const Config& a,const Config& b,MilestonePath& path);
  Real Distance(const Vector2& a,const Vector2& b);
  Real Distance(const Config& a,const Config& b);
  void AllDistances(const Vector2& a,vector<Real>& distances);
  void AllDistances(const Config& a,vector<Real>& distances);

  struct Vertex {
    enum Type { Endpoint, AABB, Triangle, Circle };
    Type type;
    int obj;
    int idata;
    Real fdata;
    Config q;
  };

  int TestAndConnectVertex(const Vertex& v);

  Geometric2DCSpace* space;
  Real offset;                //offset from obstacle boundaries
  Graph::UndirectedGraph<Vertex,EdgePlannerPtr> vertexGraph;
  Graph::UndirectedGraph<Vertex,EdgePlannerPtr> tempGraph;
};


#endif
