#include "AngleSet.h"
using namespace Math;
using namespace std;

void AngleSet::Intersect(const vector<AngleInterval>& s)
{
  AngleSet newIntervals;
  AngleInterval temp;
  for(size_t i=0; i<size(); i++) {
    for(size_t j=0;j<s.size();j++) {
      temp.setIntersection(operator[](i),s[j]);
      if(!temp.isEmpty())
	newIntervals.push_back(temp);
    }
  }
  ::swap(*this,newIntervals);
}

//void AngleSet::Union(const AngleInterval& r);

void AngleSet::Intersect(const AngleInterval& r)
{
  AngleSet newIntervals;
  AngleInterval temp;
  for(size_t i=0; i<size(); i++) {
    temp.setIntersection(operator[](i),r);
    if(!temp.isEmpty())
      newIntervals.push_back(temp);
  }
  ::swap(*this,newIntervals);
}

//void AngleSet::Subtract(const AngleInterval& r);

