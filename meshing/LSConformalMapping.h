#ifndef MESHING_LS_CONFORMAL_MAPPING_H
#define MESHING_LS_CONFORMAL_MAPPING_H

#include "TriMeshAtlas.h"

namespace Meshing {

/** @ingroup Meshing
 * @brief An approximately-conformal parameterization a genus-0
 * mesh, found by a least-squares method.
 *
 * Implements the method "Least Squares Conformal Maps for Automatic
 * Texture Atlas Generation by Levy et.al. in Siggraph 2002.
 */
class LSConformalMapping
{
 public:
  LSConformalMapping(TriMeshWithTopology& mesh,TriMeshChart& chart);
  bool Calculate();

  TriMeshWithTopology& mesh;
  TriMeshChart& chart;
};

} //namespace Meshing

#endif

