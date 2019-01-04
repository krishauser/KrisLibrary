#ifndef ARRANGEMENT_1D_H
#define ARRANGEMENT_1D_H

#include <map>
#include <list>
#include <vector>
#include <KrisLibrary/math/math.h>

namespace Geometry { 

  using namespace Math;
  using namespace std;

/** @ingroup Geometry
 * @brief An arrangement of 1-D intervals.  Intervals identified by an
 * integer ID.
 */
class Arrangement1D
{
 public:
  typedef pair<Real,Real> Interval;
  typedef list<int> IDList;
  struct LeftInterval {
    Interval interval;
    IDList pointIDs;    //id list of left endpoint
    IDList intervalIDs; //id list of interval
  };
  typedef map<Real,LeftInterval> SortedIntervals;

  Arrangement1D();
  void Insert(Real imin,Real imax,int id);
  void InsertUnique(Real imin,Real imax,int id);
  void GetIntervals(vector<Interval>& intervals,vector<const IDList*>& ids) const;
  void GetAllIntervals(vector<Interval>& intervals,vector<const IDList*>& ids) const;
  void GetOverlapIntervals(Real imin,Real imax,vector<Interval>& intervals,vector<const IDList*>& ids) const;

  //helpers
  SortedIntervals::iterator LocateInterval(Real x);
  SortedIntervals::const_iterator LocateInterval(Real x) const;
  void Split(SortedIntervals::iterator interval,Real x);

  SortedIntervals intervals;
};

} //namespace Geometry

#endif
