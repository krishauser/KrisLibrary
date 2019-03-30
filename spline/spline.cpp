#include "spline.h"
#include <KrisLibrary/File.h>
#include <KrisLibrary/math/complex.h>
#include <assert.h>

_DEFINE_READ_WRITE_FILE_BASIC(KeyHermite<Real>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyHermite<Vector2>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyHermite<Vector3>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyHermite<Vector4>);

_DEFINE_READ_WRITE_FILE_BASIC(KeyBezierCubic<Real>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyBezierCubic<Vector2>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyBezierCubic<Vector3>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyBezierCubic<Vector4>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyBezierCubic<Quaternion>);

_DEFINE_READ_WRITE_FILE_BASIC(KeyCardinal<Real>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyCardinal<Vector2>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyCardinal<Vector3>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyCardinal<Vector4>);

_DEFINE_READ_WRITE_FILE_BASIC(KeyTCB<Real>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyTCB<Vector2>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyTCB<Vector3>);
_DEFINE_READ_WRITE_FILE_BASIC(KeyTCB<Vector4>);

template class SplineBase<Real,Real>;
template class SplineBase<Vector2,Vector2>;
template class SplineBase<Vector3,Vector3>;
template class SplineBase<Vector4,Vector4>;

template class SplineBase<KeyHermite<Real>,Real>;
template class SplineBase<KeyHermite<Vector2>,Vector2>;
template class SplineBase<KeyHermite<Vector3>,Vector3>;
template class SplineBase<KeyHermite<Vector4>,Vector4>;

template class SplineBase<KeyBezierCubic<Real>,Real>;
template class SplineBase<KeyBezierCubic<Vector2>,Vector2>;
template class SplineBase<KeyBezierCubic<Vector3>,Vector3>;
template class SplineBase<KeyBezierCubic<Vector4>,Vector4>;
template class SplineBase<KeyBezierCubic<Quaternion>,Quaternion>;

template class SplineBase<KeyCardinal<Real>,Real>;
template class SplineBase<KeyCardinal<Vector2>,Vector2>;
template class SplineBase<KeyCardinal<Vector3>,Vector3>;
template class SplineBase<KeyCardinal<Vector4>,Vector4>;

template class SplineBase<KeyTCB<Real>,Real>;
template class SplineBase<KeyTCB<Vector2>,Vector2>;
template class SplineBase<KeyTCB<Vector3>,Vector3>;
template class SplineBase<KeyTCB<Vector4>,Vector4>;

template class SplineCardinal<Real>;
template class SplineCardinal<Vector2>;
template class SplineCardinal<Vector3>;
template class SplineCardinal<Vector4>;

template class SplineBezierCubic<Real>;
template class SplineBezierCubic<Vector2>;
template class SplineBezierCubic<Vector3>;
template class SplineBezierCubic<Vector4>;

template class SplineTCB<Real>;
template class SplineTCB<Vector2>;
template class SplineTCB<Vector3>;
template class SplineTCB<Vector4>;

const Real Three = (Real)3.0;

SplineIterator::SplineIterator()
  :t(0),seg(-1)
{
}

SplineIterator::SplineIterator(Real _t)
  :t(_t),seg(-1)
{
}


SplineIterator::SplineIterator(Real _t,int _seg)
  :t(_t),seg(_seg)
{
}

SplineTimeBase::SplineTimeBase()
:/*times(NULL), */flags(0)
{
}


void SplineTimeBase::init(int nt)
{
	cleanup();
	//numTimes = nt;
	//times = new Real [nt];
	times.resize(nt);
}

void SplineTimeBase::resize(int nt)
{
	times.resize(nt);
}

void SplineTimeBase::cleanup()
{
	//SafeArrayDelete(times);
	//numSegments = 0;
	times.clear();
	flags = 0;
}

SplineTimeBase::TimeStatus SplineTimeBase::seek(SplineIterator& it) const
{
	it.t = infinityMap(it.t);
	int numSegments = getNumSegments();
	//make sure we're always on a valid segment
	if(it.seg < 0)
		it.seg = 0;
	if(it.seg >= numSegments)
		it.seg = numSegments-1;
	while(it.t >= times[it.seg+1])
	{
		if(it.seg == numSegments-1)
			return After;
		it.seg++;
	}
	while(it.t < times[it.seg])
	{
		if(it.seg == 0)
			return Before;
		it.seg--;
	}

	assert(it.t >= times[it.seg] && it.t < times[it.seg+1]);
	return During;
}

void SplineTimeBase::timeTransform(Real scale, Real offset)
{
	for(size_t i=0; i<times.size(); i++)
		times[i] = times[i]*scale + offset;
}

Real SplineTimeBase::infinityMap(Real t) const
{
	if(isLooping()) return beginTime() + Mod(t-beginTime(), length());
	else return Min(t,endTime());
}


const SplineTimeBase& SplineTimeBase::operator = (const SplineTimeBase& rhs)
{
	copyTimeBase(rhs);
	return *this;
}

void SplineTimeBase::copyTimeBase(const SplineTimeBase& rhs)
{
	init(rhs.getNumKeys());
	flags = rhs.flags;
	for(size_t i=0; i<times.size(); i++)
		times[i] = rhs.times[i];
}

bool SplineTimeBase::Read(File& f)
{
	int nt;
	if(!ReadFile(f,nt)) return false;
	init(nt);
	if(!ReadFile(f,flags)) return false;
	//if(!ReadArrayFile(f,times,nt)) return false;
	if(!ReadArrayFile(f,&times[0],nt)) return false;
	return true;
}

bool SplineTimeBase::Write(File& f) const
{
	int nt = times.size();
	if(!WriteFile(f,nt)) return false;
	if(!WriteFile(f,flags)) return false;
	//if(!WriteArrayFile(f,times,times.size())) return false;
	if(!WriteArrayFile(f,&times[0],times.size())) return false;
	return true;
}



template <class Key,class Point>
SplineBase<Key,Point>::SplineBase()
//:points(NULL)
{
}

template <class Key,class Point>
void SplineBase<Key,Point>::init(int nt)
{
	SplineTimeBase::init(nt);
	//keys = new Point [nt];
	keys.resize(nt);
}

template <class Key,class Point>
void SplineBase<Key,Point>::cleanup()
{
	//SafeArrayDelete(keys);
	keys.clear();
	SplineTimeBase::cleanup();
}

template <class Key,class Point>
int SplineBase<Key,Point>::insertKey(Real time, int pos)
{
	if(pos < 0) {
		SplineIterator i(time);
		seek(i);
	}
	if(pos > (int)keys.size()) abort();
	times.insert(times.begin()+pos,time);
	keys.insert(keys.begin()+pos,Key());
	//numKeys++;
	return pos;
}


template <class Key,class Point>
void SplineBase<Key,Point>:: deleteKey(int key)
{
	assert(key >= 0 && key <= (int)keys.size());
	times.erase(times.begin()+key);
	keys.erase(keys.begin()+key);
	//numKeys--;
}

template <class Key,class Point>
void SplineBase<Key,Point>::evaluate(SplineIterator& it, Point& out) const
{
	seek(it);
	eval(it.seg,mapSegmentU(it.seg,it.t),out);
}

template <class Key,class Point>
void SplineBase<Key,Point>::operator = (const SplineBase<Key,Point>& rhs)
{
	if(&rhs == this) return;
	resize(rhs.getNumKeys());
	SplineTimeBase::copyTimeBase(rhs);
	for(size_t i=0; i<keys.size(); i++)
		keys[i] = rhs.keys[i];
}

template <class Key,class Point>
bool SplineBase<Key,Point>::Read(File& f)
{
	if(!SplineTimeBase::Read(f)) return false;
	//if(!ReadArrayFile(f,keys,getNumKeys())) return false;
	if(!ReadArrayFile(f,&keys[0],getNumKeys())) return false;
	return true;
}

template <class Key,class Point>
bool SplineBase<Key,Point>::Write(File& f) const
{
	if(!SplineTimeBase::Write(f)) return false;
	//if(!WriteArrayFile(f,points,getNumKeys())) return false;
	if(!WriteArrayFile(f,&keys[0],getNumKeys())) return false;
	return true;
}


template <class Point>
SplineCardinal<Point>::SplineCardinal()
{}



template <class Point>
void SplineCardinal<Point>::toHermite(SplineHermite<Point>& s) const
{
	//copy times
	s.copyTimeBase(*this);
	s.getPoint(0)=getPoint(0);
	int numSegments = getNumSegments();
	if(numSegments > 0) {
		s.getTangentOut(0)=(getPoint(1)-getPoint(0))*getTension(0);
		for(int i=1;i<numSegments;i++) {
			s.getPoint(i)=getPoint(i);
			s.getTangentIn(i)=s.getTangentOut(i)=(getPoint(i+1)-getPoint(i-1))*getTension(i);
		}
		s.getTangentIn(numSegments)=(getPoint(numSegments)-getPoint(numSegments-1))*getTension(numSegments);
	}
	s.getTangentIn(0) = Zero;
	s.getTangentOut(numSegments) = Zero;
}



template <class Point>
void SplineBezierCubic<Point>::toHermite(SplineHermite<Point>& s) const
{
	//copy times
	s.copyTimeBase(*this);
	for(size_t i=0;i<ParentT::keys.size();i++)
	{
		s.getPoint(i) = getPoint(i);
		//convert cps to tangents 
		s.getTangentIn(i)=(getPoint(i)-getCPIn(i))*Three;
		s.getTangentOut(i)=(getCPOut(i)-getPoint(i))*Three;
	}
}

template <class Point>
void SplineBezierCubic<Point>::fromHermite(const SplineHermite<Point>& s)
{
	//copy times
	SplineTimeBase::copyTimeBase(s);
	for(size_t i=0;i<ParentT::keys.size();i++)
	{
		getPoint(i) = s.getPoint(i);
		//convert cps to tangents 
		getCPIn(i) = s.getPoint(i)-Third*s.getTangentIn(i);
		getCPOut(i) = s.getPoint(i)+Third*s.getTangentOut(i);
	}
}


template <class Point>
SplineTCB<Point>::SplineTCB()
{}

template <class Point>
void SplineTCB<Point>::toHermite(SplineHermite<Point>& s) const
{
	//copy times
	s.copyTimeBase(*this);
	int numKeys = getNumKeys();
	for(int i=0;i<numKeys;i++)
	{
		int a = (i > 0 ? i-1 : 0);
		int b = (i+1 < numKeys ? i+1 : numKeys-1);

		const Point& P_1 = getPoint(a);
		const Point& P0 = getPoint(i);
		const Point& P1 = getPoint(b);

		s.getPoint(i) = P0;
		s.getTangentIn(i)=incomingTangent(i,P_1,P0,P1);
		s.getTangentOut(i)=outgoingTangent(i,P_1,P0,P1);
	}
}
