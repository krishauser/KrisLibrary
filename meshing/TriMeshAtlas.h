#ifndef MESHING_TRIMESH_ATLAS_H
#define MESHING_TRIMESH_ATLAS_H

#include "TriMeshTopology.h"
#include <map>

namespace Meshing {

/** @ingroup Meshing
 * @brief A chart maps a genus 0 triangle mesh to a 2d disk
 */
struct TriMeshChart
{
  TriMeshChart(TriMeshWithTopology& _mesh) : mesh(_mesh) {}

  TriMeshWithTopology& mesh;
  vector<Vector2> coordinates;
};

/** @ingroup Meshing
 * @brief TODO: a non-genus 0 mesh partitioned into charts */
struct TriMeshAtlas
{
  //vector<TriMeshChart> charts;
};

} //namespace Meshing

#endif
