#ifndef NEIGHBOR_GRAPH_H
#define NEIGHBOR_GRAPH_H

#include <KrisLibrary/graph/UndirectedGraph.h>
#include <KrisLibrary/meshing/PointCloud.h>

namespace Geometry {
  using namespace std;
  using namespace Math3D;

///Computes a graph where each node is an index of a point in the point cloud
///and each edge connects points within distance R (in Euclidean space)
void NeighborGraph(const Meshing::PointCloud3D& pc,Real R,Graph::UndirectedGraph<int,int>& G);

///Computes a graph where each node is an index of a point in the point cloud
///and each edge connects points within distance R (in Euclidean space)
void NeighborGraph(const vector<Vector3>& pc,Real R,Graph::UndirectedGraph<int,int>& G);

///Computes a graph where each node is an index of a point in the point cloud
///and each edge connects points (i,j) for which j is a k-nearest neighbor
///(in Euclidean space)
void NearestNeighborGraph(const Meshing::PointCloud3D& pc,int k,Graph::Graph<int,int>& G);

///Computes a graph where each node is an index of a point in the point cloud
///and each edge connects points (i,j) for which j is a k-nearest neighbor
///(in Euclidean space)
void NearestNeighborGraph(const vector<Vector3>& pc,int k,Graph::Graph<int,int>& G);

} //Geometyr

#endif
