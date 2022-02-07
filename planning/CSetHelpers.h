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
  virtual int NumDimensions() const override;
  virtual bool Contains(const Config& x) override;
  virtual bool Project(Config& x) override;
  virtual bool IsConvex() const override { return true; }
  virtual bool IsSampleable() const override { return true; }
  virtual void Sample(Config& x) override;
  virtual Real ObstacleDistance(const Config& x) override;
  virtual Optimization::NonlinearProgram* Numeric() override;

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
  virtual int NumDimensions() const override;
  virtual bool Contains(const Config& x) override;
  virtual bool Project(Config& x) override;
  virtual bool IsConvex() const override { return true; }
  virtual bool IsSampleable() const override { return false; }
  virtual Real ObstacleDistance(const Config& x) override;
  virtual Optimization::NonlinearProgram* Numeric() override;

  int i;
  Real low,high;
};


class NeighborhoodSet : public CSet
{
public:
  NeighborhoodSet(CSpace* space,const Config& c,Real r);
  virtual ~NeighborhoodSet() {}
  virtual int NumDimensions() const override;
  virtual bool Contains(const Config& x) override;
  virtual bool Project(Config& x) override;
  virtual bool IsConvex() const override { return true; }
  virtual bool IsSampleable() const override { return true; }
  virtual void Sample(Config& x) override;
  virtual Real ObstacleDistance(const Config& x) override;
  virtual Optimization::NonlinearProgram* Numeric() override;

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
  virtual int NumDimensions() const override { return -1; }
  virtual bool Contains(const Config& x) override;
  virtual bool Project(Config& x) override;
  virtual bool IsConvex() const override { return base->IsConvex(); }
  virtual Real ObstacleDistance(const Config& x) override;
  virtual Optimization::NonlinearProgram* Numeric() override;

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
  virtual int NumDimensions() const override;
  virtual bool Contains(const Config& x) override;
  virtual bool Project(Config& x) override;
  virtual bool IsSampleable() const override { return true; }
  virtual void Sample(Config& x) override;
  virtual Real ObstacleDistance(const Config& x) override;
  virtual Optimization::NonlinearProgram* Numeric() override;

  std::vector<Vector> items;
};

class VisibilitySet : public CSet
{
public:
  VisibilitySet(CSpace* space,const Vector& x);
  virtual ~VisibilitySet() {}
  virtual int NumDimensions() const override;
  virtual bool Contains(const Config& x) override;

  CSpace* space;
  Vector center;
};

class UnionSet : public CSet
{
public:
  UnionSet(const std::shared_ptr<CSet>& a,const std::shared_ptr<CSet>& b);
  UnionSet(const std::vector<std::shared_ptr<CSet> >& items);
  virtual ~UnionSet() {}
  virtual int NumDimensions() const override;
  virtual bool Contains(const Config& x) override;
  virtual bool Project(Config& x) override;
  virtual bool IsSampleable() const override;
  virtual void Sample(Config& x) override;

  std::vector<std::shared_ptr<CSet> > items;
};

class IntersectionSet : public CSet
{
public:
  IntersectionSet(const std::shared_ptr<CSet>& a,const std::shared_ptr<CSet>& b);
  IntersectionSet(const std::vector<std::shared_ptr<CSet> >& items);
  virtual ~IntersectionSet() {}
  virtual int NumDimensions() const override;
  virtual bool Contains(const Config& x) override;
  virtual bool IsSampleable() const override;
  virtual void Sample(Config& x) override;
  virtual bool IsConvex() const override;
  virtual Real ObstacleDistance(const Config& x) override;
  virtual Optimization::NonlinearProgram* Numeric() override;

  std::vector<std::shared_ptr<CSet> > items;
};


#endif
