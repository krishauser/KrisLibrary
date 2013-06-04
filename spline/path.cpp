#include "path.h"
#include <assert.h>


PathTranslationCardinal::PathTranslationCardinal() { init(0); }
PathTranslationCardinal::PathTranslationCardinal(const PathTranslationCardinal& rhs) { operator=(rhs); }

void PathTranslationCardinal::setKey(int k, float time, const Vector3& pos, float t)
{
	assert(k >= 0 && k < getNumKeys());
	times[k] = time;
	getPoint(k) = pos;
	getTension(k) = t;
}

PathTranslationBezier::PathTranslationBezier() { /*init(0);*/ }
PathTranslationBezier::PathTranslationBezier(const PathTranslationBezier& rhs) { operator=(rhs); }

void PathTranslationBezier::setKey(int k, float time, const Vector3& pos)
{
	assert(k >= 0 && k < getNumKeys());
	times[k] = time;
	getPoint(k) = pos;
	getCPIn(k) = pos;
	getCPOut(k) = pos;
}

void PathTranslationBezier::smoothKey(int k, float tension, bool smooth_wrap)
{
	int numSegments = getNumSegments();
	int kin, kout;
	if(k > 0)				kin = k-1;
	else if(smooth_wrap)	kin = numSegments-1;
	else					kin = k;
	if(k < numSegments)		kout = k+1;
	else if(smooth_wrap)	kout = 1;
	else					kout = k;

	Real tin, tout;
	tin = (kin == k ? Zero : getTime(kin+1) - getTime(kin));
	tout = (kout == k ? Zero : getTime(kout) - getTime(kout-1));

	Vector3 Tang;
	Real scale = tension/(tin + tout);
	Tang = scale*(getPoint(kout)-getPoint(kin));

	if(k != 0) setKeyInTangent(k,Tang);
	else if(smooth_wrap) setKeyInTangent(numSegments,Tang);

	if(k != numSegments) setKeyOutTangent(k,Tang);
	else if(smooth_wrap) setKeyOutTangent(0,Tang);
}

void PathTranslationBezier::setKeyInTangent(int k, const Vector3& tang)
{
	const Vector3& pt = getPoint(k);
	assert(k >= 0);
	Real timeDiff = getTime(k) - getTime(k-1);
	getCPIn(k) = pt - (Third*timeDiff)*tang;
}

void PathTranslationBezier::setKeyOutTangent(int k, const Vector3& tang)
{
	const Vector3& pt = getPoint(k);
	assert(k < getNumKeys());
	Real timeDiff = getTime(k+1) - getTime(k);
	getCPOut(k) = pt + (Third*timeDiff)*tang;
}

PathTranslationTCB::PathTranslationTCB() { /*init(0);*/ }
PathTranslationTCB::PathTranslationTCB(const PathTranslationTCB& rhs) { operator=(rhs); }

void PathTranslationTCB::setKey(int k, float time, const Vector3& pos,
								float t, float c, float b)
{
	assert(k >= 0 && k < getNumKeys());
	times[k] = time;
	getPoint(k) = pos;
	getTension(k) = t;
	getContinuity(k) = c;
	getBias(k) = b;
}

PathRotation::PathRotation() { /*init(0);*/ }
PathRotation::PathRotation(const PathRotation& rhs) { operator=(rhs); }

void PathRotation::setKey(int k, float time, const Quaternion& rot)
{
	getTime(k) = time;
	getPoint(k) = rot;
	getCPIn(k) = rot;
	getCPOut(k) = rot;
}

void PathRotation::smoothKey(int k, float tension, bool smooth_wrap)
{
	int numSegments = getNumSegments();
	int kin, kout;
	if(k > 0)				kin = k-1;
	else if(smooth_wrap)	kin = numSegments-1;
	else					kin = k;
	if(k < numSegments)		kout = k+1;
	else if(smooth_wrap)	kout = 1;
	else					kout = k;

	Real tin, tout;
	tin = (kin == k ? Zero : getTime(kin+1) - getTime(kin));
	tout = (kout == k ? Zero : getTime(kout) - getTime(kout-1));

	Real scale = tension/(tin + tout);

	//calculate the "destination point"
	QuaternionRotation a,b;
	a.slerp(getPoint(k), getPoint(kin), -One);
	b.slerp(a, getPoint(kout), Half);

	//now set the control points (into a)
	//incoming
	a.slerp(getPoint(k), b, -Third*tin*scale);
	getCPIn(k) = a;
	if(smooth_wrap) getCPIn(numSegments) = a;

	//outgoing
	a.slerp(getPoint(k), b, Third*tout*scale);
	getCPOut(k) = a;
	if(smooth_wrap) getCPOut(0) = a;
}


void PathRotation::eval(int seg, float t, Quaternion& out) const
{
	QuaternionRotation a,b,c;
	const Quaternion& P0 = getPoint(seg), P1 = getPoint(seg+1),
		C0 = getCPOut(seg), C1 = getCPIn(seg+1);

	a.slerp(P0,C0,t);
	b.slerp(C0,C1,t);
	c.slerp(C1,P1,t);

	a.slerp(a, b, t);
	b.slerp(b, c, t);

	c.slerp(a, b, t);

	out = c;
}



