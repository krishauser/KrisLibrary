#include <stdlib.h>
#include "NeighborGraph.h"
#include "GridSubdivision.h"
#include "KDTree.h"

namespace Geometry {

void NeighborGraph(const Meshing::PointCloud3D& pc,Real R,Graph::UndirectedGraph<int,int>& G)
{
  NeighborGraph(pc.points,R,G);
}

void NeighborGraph(const vector<Vector3>& pc,Real R,Graph::UndirectedGraph<int,int>& G)
{
  GridSubdivision grid(3,R);
  Vector temp(3);
  GridSubdivision::Index index;
  for(size_t i=0;i<pc.size();i++) {
    pc[i].get(temp);
    grid.PointToIndex(temp,index);
    grid.Insert(index,(void*)&pc[i]);
  }
  G.Resize(pc.size());
  for(size_t i=0;i<pc.size();i++) 
    G.nodes[i] = (int)i;
  for(size_t i=0;i<pc.size();i++) {
    pc[i].get(temp);
    GridSubdivision::ObjectSet items;
    grid.BallItems(temp,R,items);
    for(GridSubdivision::ObjectSet::const_iterator j=items.begin();j!=items.end();j++) {
      const Vector3* ptr = (const Vector3*)(*j);
      if(ptr->distanceSquared(pc[i]) > R*R) continue;
      int jindex = int(ptr - &pc[0]);
      if(jindex <= (int)i) continue;
      G.AddEdge((int)i,jindex,0);
    }
  }
}
void NearestNeighborGraph(const Meshing::PointCloud3D& pc,int k,Graph::Graph<int,int>& G)
{
  NearestNeighborGraph(pc.points,k,G);
}

void NearestNeighborGraph(const vector<Vector3>& pc,int k,Graph::Graph<int,int>& G)
{
  vector<Vector> copy(pc.size());
  for(size_t i=0;i<copy.size();i++) {
    copy[i].resize(3);
    pc[i].get(copy[i]);
  }
  G.Resize(pc.size());
  for(size_t i=0;i<pc.size();i++) 
    G.nodes[i] = (int)i;
  KDTree* tree = KDTree::Create(copy,3,pc.size());
  vector<Real> dist(k);
  vector<int> inds(k);
  for(size_t i=0;i<pc.size();i++) {
    tree->KClosestPoints(copy[i],k,&dist[0],&inds[0]);
    for(int j=0;j<k;j++) {
      if(inds[j] == (int)i) continue;
      G.AddEdge((int)i,inds[j],0);
    }
  }
}

} //namespace Geometry
