#ifndef SPLINE_SPLINE_H
#define SPLINE_SPLINE_H

#include "basis.h"
#include <KrisLibrary/math3d/primitives.h>  //only used for TCB
#include <assert.h>
#include <vector>
class File;
using namespace Math3D;

const Real Third = Real(1.0/3.0);

struct SplineIterator
{
  SplineIterator();
  SplineIterator(Real t);
  SplineIterator(Real t, int seg);

  Real t;
  int seg;
};

class SplineTimeBase
{
 public:
  SplineTimeBase();

  enum InfinityBehavior {
    InfinityEnd = 0x0,
    InfinityLoop = 0x1
  };

  enum TimeStatus {
    Before,
    During,
    After
  };

  virtual void init(int numKeys);
  virtual void resize(int numKeys);
  virtual void cleanup();

  const SplineTimeBase& operator = (const SplineTimeBase&);
  void copyTimeBase(const SplineTimeBase&);
  virtual bool Read(File&);
  virtual bool Write(File&) const;

  //seek is pretty important -- it sets the iterator to an infinity mapped time, and the correct segment for that time
  TimeStatus seek(SplineIterator&) const;

  inline int getNumKeys() const { return (int)times.size(); }
  inline int getNumSegments() const { return (int)times.size()-1; }
  Real& getTime(int i) { return times[i]; }
  const Real& getTime(int i) const { return times[i]; }

  inline Real beginTime() const { return times[0]; }
  inline Real endTime() const { return times[times.size()-1]; }
  inline Real length() const { return endTime() - beginTime(); }
  inline int isLooping() const { return flags & InfinityLoop; }
  inline void setInfinityBehavior(InfinityBehavior b) { flags = b; }
  void timeTransform(Real scale, Real offset);

 protected:
  //evaluation helpers
  inline Real mapSegmentU(int seg, Real t) const
    {
      return (t - times[seg])/(times[seg+1] - times[seg]);
    }
  Real infinityMap(Real t) const;

  std::vector<Real> times;
  int flags;
};


template <class Point>
struct KeyHermite
{
  Point pt;
  Point tin,tout;
};

template <class Point>
struct KeyCardinal
{
  Point pt;
  Real tension;
};

template <class Point>
struct KeyBezierCubic
{
  Point pt;
  Point cpin,cpout;
};

template <class Point>
struct KeyTCB
{
  Point pt;
  Real t,c,b;
};


template <class Key,class Point>
class SplineBase : public SplineTimeBase
{
 public:
  SplineBase();
  void init(int numKeys);
  void cleanup();
  int insertKey(Real time,int pos=-1);
  void deleteKey(int key);

  void evaluate(SplineIterator&, Point& out) const;
  inline Key& getKey(int i) { return keys[i]; }
  inline const Key& getKey(int i) const { return keys[i]; }

  void operator = (const SplineBase<Key,Point>&);
  virtual bool Read(File&);
  virtual bool Write(File&) const;

  virtual void eval(int seg, Real u, Point& out) const = 0;

  std::vector<Key> keys;
};

template <class Point>
class SplineLinear : public SplineBase<Point,Point>
{
 public:
  typedef SplineBase<Point,Point> ParentT;
  inline Point& getPoint(int i) { return ParentT::points[i]; }
  inline const Point& getPoint(int i) const { return ParentT::points[i]; }
  inline int getNumKeys() const { return ParentT::getNumKeys(); }
  inline int getNumSegments() const { return ParentT::getNumSegments(); }

  virtual void eval(int seg, Real u, Point& out) const {
    out = ParentT::keys[seg] + u*(ParentT::keys[seg+1]-ParentT::keys[seg]);
  }
};

template <class Point>
class SplineHermite : public SplineBase<KeyHermite<Point>,Point>
{
 public:
  typedef SplineBase<KeyHermite<Point>,Point> ParentT;
  inline Point& getPoint(int i) { return ParentT::keys[i].pt; }
  inline const Point& getPoint(int i) const { return ParentT::keys[i].pt; }
  inline int getNumKeys() const { return ParentT::getNumKeys(); }
  inline int getNumSegments() const { return ParentT::getNumSegments(); }

  inline Point& getTangentIn(int i) { return ParentT::keys[i].tin; }
  inline const Point& getTangentIn(int i) const { return ParentT::keys[i].tin; }
  inline Point& getTangentOut(int i) { return ParentT::keys[i].tout; }
  inline const Point& getTangentOut(int i) const { return ParentT::keys[i].tout; }

  virtual void eval(int seg, Real u, Point& out) const {
    HermiteSplineBasis basis;
    Real b[4];
    basis.EvalBasis(u, b);
    //p0 p1 t0 t1
    out = b[0]*getPoint(seg) + b[1]*getPoint(seg+1) + b[2]*getTangentOut(seg) + b[3]*getTangentIn(seg+1);
  }
};

template <class Point>
class SplineCardinal: public SplineBase<KeyCardinal<Point>,Point>
{
 public:
  typedef SplineBase<KeyCardinal<Point>,Point> ParentT;

  SplineCardinal();

  inline Point& getPoint(int i) { return ParentT::keys[i].pt; }
  inline const Point& getPoint(int i) const { return ParentT::keys[i].pt; }
  inline int getNumKeys() const { return ParentT::getNumKeys(); }
  inline int getNumSegments() const { return ParentT::getNumSegments(); }

  inline Real& getTension(int i) { return ParentT::keys[i].tension; }
  inline const Real& getTension(int i) const { return ParentT::keys[i].tension; }

  void toHermite(SplineHermite<Point>& spline) const;

  virtual void eval(int seg, Real u, Point& out) const {
    int a = (seg > 0 ? seg-1 : 0);
    int d = (seg+2 < ParentT::getNumKeys() ? seg+2 : getNumKeys()-1);

    const Point& P0 = getPoint(seg);
    const Point& P1 = getPoint(seg+1);
    Point T0,T1;
    T0 = getTension(seg)*(P1 - getPoint(a));
    T1 = getTension(seg)*(getPoint(d) - P0);

    HermiteSplineBasis basis;
    Real b[4];
    basis.EvalBasis(u, b);
    out = b[0]*P0 + b[1]*P1 + b[2]*T0 + b[3]*T1;
  }
};

template <class Point>
class SplineBezierCubic : public SplineBase<KeyBezierCubic<Point>,Point>
{
 public:
  typedef SplineBase<KeyBezierCubic<Point>,Point> ParentT;

  inline Point& getPoint(int i) { return ParentT::keys[i].pt; }
  inline const Point& getPoint(int i) const { return ParentT::keys[i].pt; }
  inline int getNumKeys() const { return ParentT::getNumKeys(); }
  inline int getNumSegments() const { return ParentT::getNumSegments(); }

  inline Point& getCPIn(int i) { return ParentT::keys[i].cpin; }
  inline Point& getCPOut(int i) { return ParentT::keys[i].cpin; }
  inline const Point& getCPIn(int i) const { return ParentT::keys[i].cpout; }
  inline const Point& getCPOut(int i) const { return ParentT::keys[i].cpout; }

  void toHermite(SplineHermite<Point>& s) const;
  void fromHermite(const SplineHermite<Point>& s);

  virtual void eval(int seg, Real u, Point& out) const {
    BezierCubicSplineBasis basis;
    Real b[4];
    basis.EvalBasis(u, b);
    //p0 c0 c1 p1
    out = b[0]*getPoint(seg) + b[1]*getCPOut(seg) + b[2]*getCPIn(seg+1) + b[3]*getPoint(seg+1);
  }
};

template <class Point>
class SplineTCB: public SplineBase<KeyTCB<Point>,Point>
{
 public:
  typedef SplineBase<KeyTCB<Point>,Point> ParentT;

  SplineTCB();

  void toHermite(SplineHermite<Point>& s) const;
  //void fromHermite(const SplineHermite<Point>& s);

  inline Point& getPoint(int i) { return ParentT::keys[i].pt; }
  inline const Point& getPoint(int i) const { return ParentT::keys[i].pt; }
  inline int getNumKeys() const { return ParentT::getNumKeys(); }
  inline int getNumSegments() const { return ParentT::getNumSegments(); }

  inline Real& getTension(int i) { return ParentT::keys[i].t; };
  inline Real& getContinuity(int i) { return ParentT::keys[i].c; };
  inline Real& getBias(int i) { return ParentT::keys[i].b; };
  inline const Real& getTension(int i) const { return ParentT::keys[i].t; };
  inline const Real& getContinuity(int i) const { return ParentT::keys[i].c; };
  inline const Real& getBias(int i) const { return ParentT::keys[i].b; };

  virtual void eval(int seg, Real u, Point& out) const {
    int a = (seg > 0 ? seg-1 : 0);
    int d = (seg+2 < getNumKeys() ? seg+2 : getNumKeys()-1);

    const Point& P_1 = getPoint(a);
    const Point& P0 = getPoint(seg);
    const Point& P1 = getPoint(seg+1);
    const Point& P2 = getPoint(d);

    Point T0,T1;
    T0 = outgoingTangent(seg, P_1, P0, P1);
    T1 = incomingTangent(seg, P0, P1, P2);

    HermiteSplineBasis basis;
    Real b[4];
    basis.EvalBasis(u, b);
    out = b[0]*P0 + b[1]*P1 + b[2]*T0 + b[3]*T1;
  }

  Point incomingTangent(int seg, const Point& P_1, const Point& P0, const Point& P1) const
  {
    Real t,c,b;
    t = getTension(seg); c = getContinuity(seg); b = getBias(seg);
    return ((1-t)*(1-c)*(1+b)*Half)*(P0 - P_1) + ((1-t)*(1+c)*(1-b)*Half)*(P1 - P0);
  }

  Point outgoingTangent(int seg, const Point& P_1, const Point& P0, const Point& P1) const
  {
    Real t,c,b;
    t = getTension(seg); c = getContinuity(seg); b = getBias(seg);
    return ((1-t)*(1+c)*(1+b)*Half)*(P0 - P_1) + ((1-t)*(1-c)*(1-b)*Half)*(P1 - P0);
  }
};



#endif
