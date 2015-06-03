#ifndef CSPACE_HELPERS_H
#define CSPACE_HELPERS_H

#include "CSpace.h"

/** @brief A helper class that assists with selective overriding
 * of another cspace's methods.
 *
 * All CSpace methods are copied except those that are overridden
 * by the subclass of PiggybackCSpace. 
 *
 * Warning: unexpected behavior may be observed when using the 
 * baseSpace's local planners, because they often contain pointers
 * to the CSpace that created them, not the piggybacking CSpace.
 */
class PiggybackCSpace : public CSpace
{
public:
  PiggybackCSpace(CSpace* _baseSpace=NULL)
    :baseSpace(_baseSpace)
  {}
  virtual void Sample(Config& x) {
    Assert(baseSpace!=NULL);
    baseSpace->Sample(x);
  }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) {
    if(baseSpace!=NULL) baseSpace->SampleNeighborhood(c,r,x);
    else CSpace::SampleNeighborhood(c,r,x);
  }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) {
    Assert(baseSpace != NULL);
    return baseSpace->LocalPlanner(a,b);
  }
  virtual bool IsFeasible(const Config& x) {
    if(baseSpace) return baseSpace->IsFeasible(x);
    else return true;
  }
  virtual Real Distance(const Config& x, const Config& y) {
    if(baseSpace) return baseSpace->Distance(x,y);
    else return CSpace::Distance(x,y);
  }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) {
    if(baseSpace) baseSpace->Interpolate(x,y,u,out);
    else CSpace::Interpolate(x,y,u,out);
  }
  virtual void Midpoint(const Config& x,const Config& y,Config& out) {
    if(baseSpace) baseSpace->Midpoint(x,y,out);
    else CSpace::Midpoint(x,y,out);
  }
  virtual void Properties(PropertyMap& map) const {
    if(baseSpace) baseSpace->Properties(map);
    else CSpace::Properties(map);
  }

  CSpace* baseSpace;
};

/** @brief A helper class that defines a feasible set around
 * the 'radius' neighborhood of 'center'.  Often used for endgame
 * regions.
 */
class NeighborhoodCSpace : public PiggybackCSpace
{
public:
  NeighborhoodCSpace()
    :radius(0)
  {}
  NeighborhoodCSpace(const Config& _center,Real _radius)
    :center(_center),radius(_radius)
  {}
  NeighborhoodCSpace(CSpace* _baseSpace,const Config& _center,Real _radius)
    :PiggybackCSpace(_baseSpace),center(_center),radius(_radius)
  {}
  virtual void Sample(Config& x) { SampleNeighborhood(center,radius,x); }
  virtual bool IsFeasible(const Config& x) { 
    return (Distance(x,center) < radius); 
  }
  virtual void Properties(PropertyMap& map) const {
    PiggybackCSpace::Properties(map);
    map.set("volume",Pow(2.0*radius,center.n));
    Vector vmin=center-Vector(center.n,radius);
    Vector vmax=center+Vector(center.n,radius);
    map.setArray("minimum",vector<double>(vmin));
    map.setArray("maximum",vector<double>(vmax));
  }

  Config center;
  Real radius;
};

/** @brief A helper class that defines a feasible set as the visible
 * set of a given set of points.
 */
class VisibilityCSpace : public PiggybackCSpace
{
public:
  VisibilityCSpace()
  {}
  VisibilityCSpace(const Config& _center)
    :centers(1,_center)
  {}
  VisibilityCSpace(CSpace* _baseSpace,const Config& _center)
    :PiggybackCSpace(_baseSpace),centers(1,_center)
  {}
  virtual bool IsFeasible(const Config& x)
  { 
    //may want to check in order of increasing distance?
    for(size_t i=0;i<centers.size();i++) {
      EdgePlanner* e=baseSpace->LocalPlanner(x,centers[i]);
      bool res=e->IsVisible();
      delete e;
      if(res) return true;
    }
    return false;
  }

  std::vector<Config> centers;
};

#endif
