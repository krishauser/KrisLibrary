#ifndef PLANNING_CSET_HELPERS_H
#define PLANNING_CSET_HELPERS_H

#include "CSet.h"
#include "CSpace.h"

/** @ingroup MotionPlanning
 * A standard axis-aligned box set
 */
class BoxSet : public CSet
{
public:
  BoxSet(Real xmin,Real xmax,int d=1);
  BoxSet(const Vector& bmin,const Vector& bmax);
  virtual ~BoxSet() {}
  virtual int NumDimensions() const;
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsConvex() const { return true; }
  virtual bool IsSampleable() const { return true; }
  virtual void Sample(Config& x);
  virtual Real ObstacleDistance(const Config& x);
  virtual Optimization::NonlinearProgram* Numeric();

  Vector bmin,bmax;
};

/** @ingroup MotionPlanning
 * An axis range set low <= x[i] <= high
 */
class AxisRangeSet : public CSet
{
public:
  AxisRangeSet(int i,Real xmin,Real xmax);
  virtual ~AxisRangeSet() {}
  virtual int NumDimensions() const;
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsConvex() const { return true; }
  virtual bool IsSampleable() const { return false; }
  virtual Real ObstacleDistance(const Config& x);
  virtual Optimization::NonlinearProgram* Numeric();

  int i;
  Real low,high;
};


class NeighborhoodSet : public CSet
{
public:
  NeighborhoodSet(CSpace* space,const Config& c,Real r);
  virtual ~NeighborhoodSet() {}
  virtual int NumDimensions() const;
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsConvex() const { return true; }
  virtual bool IsSampleable() const { return true; }
  virtual void Sample(Config& x);
  virtual Real ObstacleDistance(const Config& x);
  virtual Optimization::NonlinearProgram* Numeric();

  CSpace* space;
  Config center;
  Real radius;
};

/** @brief Tests whether the range of values from x[imin:imax] is
 * contained within base.
 */
class SubspaceSet : public CSet
{
public:
  SubspaceSet(const std::shared_ptr<CSet>& base,int imin,int imax);
  virtual ~SubspaceSet() {}
  virtual int NumDimensions() const { return -1; }
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsConvex() const { return base->IsConvex(); }
  virtual Real ObstacleDistance(const Config& x);
  virtual Optimization::NonlinearProgram* Numeric();

  std::shared_ptr<CSet> base;
  int imin,imax;
};

class FiniteSet : public CSet
{
public:
  FiniteSet();
  FiniteSet(const Vector& item);
  FiniteSet(const Vector& item1,const Vector& item2);
  FiniteSet(const Vector& item1,const Vector& item2,const Vector& item3);
  FiniteSet(const std::vector<Vector>& items);
  virtual ~FiniteSet() {}
  virtual int NumDimensions() const;
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsSampleable() const { return true; }
  virtual void Sample(Config& x);
  virtual Real ObstacleDistance(const Config& x);
  virtual Optimization::NonlinearProgram* Numeric();

  std::vector<Vector> items;
};

class VisibilitySet : public CSet
{
public:
  VisibilitySet(CSpace* space,const Vector& x);
  virtual ~VisibilitySet() {}
  virtual int NumDimensions() const;
  virtual bool Contains(const Config& x);

  CSpace* space;
  Vector center;
};

class UnionSet : public CSet
{
public:
  UnionSet(const std::shared_ptr<CSet>& a,const std::shared_ptr<CSet>& b);
  UnionSet(const std::vector<std::shared_ptr<CSet> >& items);
  virtual ~UnionSet() {}
  virtual int NumDimensions() const;
  virtual bool Contains(const Config& x);
  virtual bool Project(Config& x);
  virtual bool IsSampleable() const;
  virtual void Sample(Config& x);

  std::vector<std::shared_ptr<CSet> > items;
};

class IntersectionSet : public CSet
{
public:
  IntersectionSet(const std::shared_ptr<CSet>& a,const std::shared_ptr<CSet>& b);
  IntersectionSet(const std::vector<std::shared_ptr<CSet> >& items);
  virtual ~IntersectionSet() {}
  virtual int NumDimensions() const;
  virtual bool Contains(const Config& x);
  virtual bool IsSampleable() const;
  virtual void Sample(Config& x);
  virtual bool IsConvex() const;
  virtual Real ObstacleDistance(const Config& x);
  virtual Optimization::NonlinearProgram* Numeric();

  std::vector<std::shared_ptr<CSet> > items;
};


#endif
