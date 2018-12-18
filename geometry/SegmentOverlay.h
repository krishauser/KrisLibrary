#ifndef GEOMETRY_SEGMENT_OVERLAY_H
#define GEOMETRY_SEGMENT_OVERLAY_H

#include <KrisLibrary/math3d/Segment2D.h>
#include <KrisLibrary/structs/RedBlack.h>
#include <vector>

namespace Geometry {

using namespace std;

using namespace Math3D;

//sweep line goes down and to the right
inline bool SweepLineOrder(const Point2D& a,const Point2D& b)
{
  return (a.y > b.y || (a.y==b.y && a.x < b.x));
}

/** @ingroup Geometry
 * @brief Computes the overlay of a set of segments using a sweep line
 * algorithm.
 *
 * I'm not sure if this works completely accurately or not -- uses floating
 * point numerical precision.
 */
class SegmentOverlay
{
public:
  SegmentOverlay(const vector<Segment2D>& S);
  void GetSegments(vector<Segment2D>& Snew) const;

  //output
  struct IntersectionInfo {
    Vector2 x;
    //L is the set of segments containing x as lower endpoint
    //U " as upper endpoint
    //C " in the interior
    vector<int> L,U,C;
  };

  vector<IntersectionInfo> output;

private:
  struct Event {
    Vector2 p;
    vector<int> U;  //list of segments of which p is the upper endpoint
  };

  struct EventCmp {
    inline int operator()(const Event& a,const Event& b) const {
      if(SweepLineOrder(a.p,b.p)) return -1;
      else if(SweepLineOrder(b.p,a.p)) return 1;
      else return 0;
    }
  };

  struct StatusCmp {
    int operator()(int a,int b) const;

    const vector<Segment2D>* S;
    Vector2 p;
    bool epsLower;  //true if we should compare as if we were right below p
    Real tol; //tolerance on segment distance
  };

  void Solve();
  void HandleEvent(const Event& e);
  void FindNewEvent(int sl,int sr,const Vector2& p);

  //helpers
  typedef RedBlack::Tree<Event,EventCmp> EventQueue;
  typedef RedBlack::Tree<int,StatusCmp> StatusTree;

  void GetContaining(const Vector2& p,StatusTree::iterator& l,StatusTree::iterator& r);
  void GetLeftmostRightmost(const vector<int>& L,const vector<int>& C,int& l,int& r);
  bool StatusUpperNeighbor(int s,int &ub);
  bool StatusLowerNeighbor(int s,int &lb);

  const vector<Segment2D>& S;
  EventQueue Q;
  StatusTree status;
  int verbose;
};

} //namespace Geometry

#endif
