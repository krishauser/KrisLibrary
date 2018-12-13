#include "VisibilityGraphPlanner.h"
#include <graph/ShortestPaths.h>
#include <graph/Path.h>

VisibilityGraphPlanner::VisibilityGraphPlanner(Geometric2DCSpace* _space)
  :space(_space),offset(1e-5)
{}

void VisibilityGraphPlanner::Init()
{
  Assert(space->circles.empty());
  vertexGraph.Cleanup();
  Vertex v;
  v.q.resize(2);
  v.type = Vertex::AABB;
  for(size_t i=0;i<space->aabbs.size();i++) {
    v.obj = (int)i;
    v.idata = 0;
    v.q(0) = space->aabbs[i].bmin.x-offset;
    v.q(1) = space->aabbs[i].bmin.y-offset;
    TestAndConnectVertex(v);
    v.idata = 1;
    v.q(0) = space->aabbs[i].bmin.x-offset;
    v.q(1) = space->aabbs[i].bmax.y+offset;
    int v1=TestAndConnectVertex(v);
    v.idata = 2;
    v.q(0) = space->aabbs[i].bmax.x+offset;
    v.q(1) = space->aabbs[i].bmax.y+offset;
    TestAndConnectVertex(v);
    v.idata = 3;
    v.q(0) = space->aabbs[i].bmax.x+offset;
    v.q(1) = space->aabbs[i].bmin.y-offset;
    TestAndConnectVertex(v);
  }
  v.type = Vertex::Triangle;
  for(size_t i=0;i<space->triangles.size();i++) {
    Vector2 c = (space->triangles[i].a+space->triangles[i].b+space->triangles[i].c)/3.0;
    Vector2 da = (space->triangles[i].a-c);
    Vector2 db = (space->triangles[i].b-c);
    Vector2 dc = (space->triangles[i].c-c);
    da.inplaceNormalize();
    db.inplaceNormalize();
    dc.inplaceNormalize();
    v.obj = (int)i;
    v.idata = 0;
    (space->triangles[i].a + offset*da).get(v.q(0),v.q(1));
    TestAndConnectVertex(v);
    v.idata = 1;
    (space->triangles[i].b + offset*db).get(v.q(0),v.q(1));
    TestAndConnectVertex(v);
    v.idata = 2;
    (space->triangles[i].c + offset*dc).get(v.q(0),v.q(1));
    TestAndConnectVertex(v);
  }
  //construct tempgraph -- last 2 nodes are endpoints
  tempGraph = vertexGraph;
  v.type = Vertex::Endpoint;
  v.obj = 0;
  tempGraph.AddNode(v);
  v.obj = 1;
  tempGraph.AddNode(v);
}

int VisibilityGraphPlanner::TestAndConnectVertex(const Vertex& v)
{
  if(!space->IsFeasible(v.q)) return -1;
  int index=vertexGraph.AddNode(v);
  for(int i=0;i<index;i++) {
    EdgePlannerPtr e = IsVisible(space,vertexGraph.nodes[i].q,v.q);
    if(e) {
      vertexGraph.AddEdge(i,index,e);
    }
  }
  return index;
}

double LengthWeightFunc(const EdgePlannerPtr& e,int s,int t)
{
  Assert(e);
  return e->Length();
}

bool VisibilityGraphPlanner::Plan(const Vector2& a,const Vector2& b,vector<Vector2>& path)
{
  Config qa(2),qb(2);
  qa.copy(a);
  qb.copy(b);
  MilestonePath mpath;
  if(!Plan(qa,qb,mpath)) {
    path.resize(0);
    return false;
  }
  path.resize(mpath.NumMilestones());
  for(size_t i=0;i<path.size();i++)
    path[i].set(mpath.GetMilestone(i));
  return true;
}


bool VisibilityGraphPlanner::Plan(const Config& a,const Config& b,MilestonePath& path)
{
  Assert(a.n == 2);
  Assert(b.n == 2);
  int vstart = tempGraph.NumNodes()-2;
  int vgoal = tempGraph.NumNodes()-1;
  tempGraph.nodes[vstart].q = a;
  tempGraph.nodes[vgoal].q = b;
  tempGraph.DeleteIncomingEdges(vstart);
  tempGraph.DeleteIncomingEdges(vgoal);

  //test straight line
  EdgePlannerPtr e = IsVisible(space,tempGraph.nodes[vstart].q,tempGraph.nodes[vgoal].q);
  if(e) {
    path.edges.resize(1);
    path.edges[0] = e;
    return true;
  }
  //connect vstart
  for(int i=0;i<vstart;i++) {
    EdgePlannerPtr e = IsVisible(space,tempGraph.nodes[i].q,tempGraph.nodes[vstart].q);
    if(e)
      tempGraph.AddEdge(i,vstart,e);
  }
  //connect vgoal
  for(int i=0;i<vstart;i++) {
    EdgePlannerPtr e = IsVisible(space,tempGraph.nodes[i].q,tempGraph.nodes[vgoal].q);
    if(e)
      tempGraph.AddEdge(i,vgoal,e);
  }
  //search
  Graph::ShortestPathProblem<Vertex,EdgePlannerPtr> spp(tempGraph);
  spp.InitializeSource(vstart);
  spp.FindPath_Undirected(vgoal,LengthWeightFunc);
  list<int> nodes;
  bool res=Graph::GetAncestorPath(spp.p,vgoal,vstart,nodes);
  if(!res) return false;
  path.edges.clear();
  for(list<int>::const_iterator i=nodes.begin();i!=--nodes.end();++i) {
    list<int>::const_iterator n=i; ++n;
    EdgePlannerPtr* e=tempGraph.FindEdge(*i,*n);
    Assert(e != NULL);
    if(*i < *n) {
      path.edges.push_back((*e)->Copy());
    }
    else {
      path.edges.push_back((*e)->ReverseCopy());
    }
  }
  return true;
}

Real VisibilityGraphPlanner::Distance(const Vector2& a,const Vector2& b)
{
  Config qa(2),qb(2);
  qa.copy(a);
  qb.copy(b);
  return Distance(qa,qb);
}

Real VisibilityGraphPlanner::Distance(const Config& a,const Config& b)
{
  Assert(a.n == 2);
  Assert(b.n == 2);
  int vstart = tempGraph.NumNodes()-2;
  int vgoal = tempGraph.NumNodes()-1;
  tempGraph.nodes[vstart].q = a;
  tempGraph.nodes[vgoal].q = b;
  tempGraph.DeleteIncomingEdges(vstart);
  tempGraph.DeleteIncomingEdges(vgoal);

  //test straight line
  EdgePlannerPtr e = IsVisible(space,tempGraph.nodes[vstart].q,tempGraph.nodes[vgoal].q);
  if(e) {
    return LengthWeightFunc(e,vstart,vgoal);
  }
  //connect vstart
  for(int i=0;i<vstart;i++) {
    EdgePlannerPtr e = IsVisible(space,tempGraph.nodes[i].q,tempGraph.nodes[vstart].q);
    if(e)
      tempGraph.AddEdge(i,vstart,e);
  }
  //connect vgoal
  for(int i=0;i<vstart;i++) {
    EdgePlannerPtr e = IsVisible(space,tempGraph.nodes[i].q,tempGraph.nodes[vgoal].q);
    if(e)
      tempGraph.AddEdge(i,vgoal,e);
  }
  //search
  Graph::ShortestPathProblem<Vertex,EdgePlannerPtr> spp(tempGraph);
  spp.InitializeSource(vstart);
  spp.FindPath_Undirected(vgoal,LengthWeightFunc);
  return spp.d[vgoal];
}

void VisibilityGraphPlanner::AllDistances(const Vector2& a,vector<Real>& distances)
{
  Config qa(2);
  qa.copy(a);
  AllDistances(qa,distances);
}

void VisibilityGraphPlanner::AllDistances(const Config& a,vector<Real>& distances)
{
  Assert(a.n == 2);
  int vstart = tempGraph.NumNodes()-2;
  int vgoal = tempGraph.NumNodes()-1;
  tempGraph.nodes[vstart].q = a;
  tempGraph.DeleteIncomingEdges(vstart);
  tempGraph.DeleteIncomingEdges(vgoal);

  //connect vstart
  for(int i=0;i<vstart;i++) {
    EdgePlannerPtr e = IsVisible(space,tempGraph.nodes[i].q,tempGraph.nodes[vstart].q);
    if(e)
      tempGraph.AddEdge(i,vstart,e);
  }
  //search
  Graph::ShortestPathProblem<Vertex,EdgePlannerPtr> spp(tempGraph);
  spp.InitializeSource(vstart);
  spp.FindAllPaths_Undirected(LengthWeightFunc);
  distances = spp.d;
  distances.resize(vstart);
}
