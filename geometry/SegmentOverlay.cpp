#include <KrisLibrary/Logger.h>
#include "SegmentOverlay.h"
#include "primitives.h"
#include <math3d/Plane2D.h>
using namespace Geometry;

//returns -1,0,1 if p is left/on/right of the segment (u,l) (in sweep order)
int PtSegCmp(const Vector2& p,const Vector2& u,const Vector2& l,Real tol)
{
  assert(p.y >= l.y);
  assert(p.y <= u.y);
  if(p.x < Min(u.x,l.x)) return -1;
  if(p.x > Max(u.x,l.x)) return 1;
  if(u==p || l==p) return 0;
  Plane2D plane;
  plane.setPoints(u,l);
  Real d=plane.distance(p); //+ side is to the right
  if(d < -tol) return -1; 
  else if(d > tol) return 1; 
  else return 0; 
}

//returns 1 if p is left of the segment s
int PtSegCmp(const Vector2& p,const Segment2D& s,Real tol)
{
  //get upper and lower
  if(!SweepLineOrder(s.a,s.b)) return PtSegCmp(p,s.b,s.a,tol);
  else return PtSegCmp(p,s.a,s.b,tol);
}

//same as below, but u.y > l.y
Real LeastIntersection(const Vector2& p,const Vector2& u,const Vector2& l)
{
  if(u==p) return p.x;
  if(l==p) return p.x;
  if(l.y==u.y) {
    assert(l.y == p.y);
    //if p contained in interval (u.x,l.x), return p.x
    if(u.x <= p.x) {
      assert(l.x >= p.x);
      return p.x;
    }
    assert(l.x > p.x);
    return u.x;
  }
  Real den=u.y-l.y;
  return ((p.y-l.y)*u.x + (u.y-p.y)*l.x)/den;
}

//return intersection of y=p.y (minus x < p.x)
Real LeastIntersection(const Vector2& p,const Segment2D& s)
{
  if(!SweepLineOrder(s.a,s.b)) return LeastIntersection(p,s.b,s.a);
  else return LeastIntersection(p,s.a,s.b);
}

int OrientCmp(const Vector2& p,const Vector2& a,const Vector2& b)
{
  int ori=(int)Sign(Orient2D(p,a,b));
  if(ori==0) { //overlapping
    if(SweepLineOrder(a,b)) return -1;
    else if(SweepLineOrder(b,a)) return 1;
    else //identitcal segs
      return 0;
  }
  return ori;
}

int SegmentOverlay::StatusCmp::operator()(int a,int b) const
{
  if(a==-1 || b==-1) { //we're checking for intersection with p
    assert(!epsLower);
    assert(a==-1);
    int res=PtSegCmp(p,(*S)[b],tol);
    //LOG4CXX_INFO(KrisLibrary::logger(),"  Final res: "<<res);
    return res;
  }
  const Segment2D &u=(*S)[a], &v=(*S)[b];
  //LOG4CXX_INFO(KrisLibrary::logger(),"  At "<<p<<", cmp segs "<<u.a<<" -> "<<u.b<<" vs "<<v.a<<" -> "<<v.b<<" = ");
  Real ux,vx;
  //if both intersect at p, check the left/right orientation
  ux = LeastIntersection(p,u);
  vx = LeastIntersection(p,v);
  if(FuzzyEquals(ux,vx,tol)) {
    if(FuzzyEquals(ux,p.x,tol)) { //intersect at p, check orientation
      //LOG4CXX_INFO(KrisLibrary::logger(),"\n"<<"Orientation test about p: ");
	    //orientation test about p
	    const Vector2 *a, *b;
	    a = (SweepLineOrder(p,u.a) ? &u.a :&u.b);
	    b = (SweepLineOrder(p,v.a) ? &v.a :&v.b);
	    int res=OrientCmp(p,*b,*a);
      //LOG4CXX_INFO(KrisLibrary::logger(),res);
      return res;
    }
    //don't ever return 0 -- which one is oriented about the intersection pt?
    Vector2 p2(ux,p.y);
    //LOG4CXX_INFO(KrisLibrary::logger(),"\n"<<"Orientation test about different p "<<p2<<": ");
    //orientation test about p2
    const Vector2 *a, *b;
    a = (SweepLineOrder(p2,u.a) ? &u.a :&u.b);
    b = (SweepLineOrder(p2,v.a) ? &v.a :&v.b);
    int res=OrientCmp(p2,*b,*a);
    //LOG4CXX_INFO(KrisLibrary::logger(),res);
    return res;
  }
  else if(ux < vx) { /*LOG4CXX_INFO(KrisLibrary::logger(),-1); */return -1; }
  else { /*LOG4CXX_INFO(KrisLibrary::logger(),1); */ return 1; }
}


SegmentOverlay::SegmentOverlay(const vector<Segment2D>& S_)
  :S(S_),verbose(0)
{
  status.cmpFunc.S = &S;
  status.cmpFunc.tol = 1e-7;
  Solve();
}

void SegmentOverlay::Solve()
{
  typedef EventQueue::iterator EventIterator;
  //initialize
  for(size_t i=0;i<S.size();i++) {
    const Vector2* u=&S[i].a;
    const Vector2* l=&S[i].b;
    if(!SweepLineOrder(*u,*l))
      swap(u,l);
    Event temp;
    temp.p=*u;
    EventIterator e=Q.find(temp);
    if(!e) 
      e=Q.insert(temp);
    e.curp->key.U.push_back((int)i);    //HACK! should just make (key,data) pairs
    temp.p = *l;
    Q.insert(temp);
  }

  //run
  while(!Q.empty()) {
    EventIterator e=Q.begin(); 
    Event temp=*e;
    Q.erase(e);
    HandleEvent(temp);
  }
}

void SegmentOverlay::HandleEvent(const Event& e)
{
  const Vector2& p=e.p;
  const vector<int>& U=e.U;
  StatusTree::iterator begin,end,i,next;
  if(verbose >= 1) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Handling "<<p);
    LOG4CXX_INFO(KrisLibrary::logger(),"  Status is ");
    for(i=status.begin();i!=status.end();i++)
      LOG4CXX_INFO(KrisLibrary::logger(),*i<<" ");
    LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  }
  //Get points containing p into L,C
  vector<int> L,C;
  GetContaining(p,begin,end);
  //get all points from begin to end
  for(i=begin;i!=end;i++) {
    const Segment2D& s=S[*i];
    if(s.a == p) {
      assert(SweepLineOrder(s.b,s.a));
      L.push_back(*i);
    }
    else if(s.b == p) {
      assert(SweepLineOrder(s.a,s.b));
      L.push_back(*i);
    }
    else
      C.push_back(*i);
  }

  if(verbose >= 2)
    LOG4CXX_INFO(KrisLibrary::logger(),"  |L|,|U|,|C| = "<<L.size()<<" "<<U.size()<<" "<<C.size());
  if(L.size()+U.size()+C.size() > 1) {
    size_t index=output.size();
    output.resize(output.size()+1);
    output[index].x=p;
    output[index].L=L;
    output[index].C=C;
    output[index].U=U;
  }
  assert(L.size()+U.size()+C.size() >= 1);
  //erase all points that contain p from status
  i=begin;
  while(i!=end) {
    next=i; next++;
    status.erase(i);
    i=next;
  }
  //for(size_t i=0;i<L.size();i++) status.erase(L[i]);
  //for(size_t i=0;i<C.size();i++) status.erase(C[i]);

  //insert e->U and C to status under p
  //(first update status' ordering)
  status.cmpFunc.p=p;
  status.cmpFunc.epsLower=true;
  for(size_t i=0;i<U.size();i++) status.insert(U[i]);
  for(size_t i=0;i<C.size();i++) status.insert(C[i]);
  if(U.empty() && C.empty()) {
    status.cmpFunc.epsLower=false;
    //find the left and right neighbors of p in status
    int sl,sr;
    if(StatusLowerNeighbor(-1,sl) && StatusUpperNeighbor(-1,sr)) {
      FindNewEvent(sl,sr,p);
    }
  }
  else {
    int l,r,sl,sr;
    //get the leftmost and rightmost segments in U union C
    GetLeftmostRightmost(U,C,l,r);
    if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"  Leftmost, rightmost are "<<l<<", "<<r);

    if(StatusLowerNeighbor(l,sl)) {
      if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"  Lower neighbor "<<sl);
      FindNewEvent(sl,l,p);
    }
    if(StatusUpperNeighbor(r,sr)) {
      if(verbose >= 2) LOG4CXX_INFO(KrisLibrary::logger(),"  Upper neighbor "<<sr);
      FindNewEvent(r,sr,p);
    }
  }
}

void SegmentOverlay::FindNewEvent(int sl,int sr,const Vector2& p)
{
  Vector2 x;
  if(S[sl].intersects(S[sr],x)) {
    if(SweepLineOrder(p,x)) {  //x comes after p in the ordering
      if(verbose >= 1) LOG4CXX_INFO(KrisLibrary::logger(),"New intersection at "<<x);
      //insert x as a new event
      Event e;
      e.p=x;
      Q.insert(e);
    }
  }
}

void SegmentOverlay::GetContaining(const Vector2& p,StatusTree::iterator& l,StatusTree::iterator& r)
{
  status.cmpFunc.p=p;
  status.cmpFunc.epsLower=false;
  StatusTree::iterator x;
  x=status.find(-1);
  if(!x) {
    r=l=status.end();
    return;
  }
  l=r=x;
  while(!l.end()&&status.cmpFunc(-1,*l)==0) --l;
  if(l.end()) l=status.front();
  else ++l;
  ++r;
  while(!r.end()&&status.cmpFunc(-1,*r)==0) ++r;
}

void SegmentOverlay::GetLeftmostRightmost(const vector<int>&U,const vector<int>& C,int& l,int& r)
{
  assert(!U.empty() || !C.empty());
  if(!U.empty()) l=r=U[0];
  else l=r=C[0];
  for(size_t i=1;i<U.size();i++) {
    if(status.cmpFunc(l,U[i]) == 1) l=U[i];
    else if(status.cmpFunc(r,U[i]) == -1) r=U[i];
  }
  for(size_t i=0;i<C.size();i++) {
    if(status.cmpFunc(l,C[i]) == 1) l=C[i];
    else if(status.cmpFunc(r,C[i]) == -1) r=C[i];
  }
}

bool SegmentOverlay::StatusLowerNeighbor(int s,int& lb)
{
  StatusTree::iterator b=status.lookup(RedBlack::Less,s);
  if(b) {
    lb=*b;
    return true;
  }
  return false;
}

bool SegmentOverlay::StatusUpperNeighbor(int s,int& ub)
{
  StatusTree::iterator b=status.lookup(RedBlack::Greater,s);
  if(b) {
    ub=*b;
    return true;
  }
  return false;
}


