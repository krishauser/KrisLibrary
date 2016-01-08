#ifndef SPLINE_PATH_H
#define SPLINE_PATH_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include "spline.h"

/*************************************************************
 * PathTranslationCardinal
 *
 * A translation path with a tension parameter.
 *
 *************************************************************/
class PathTranslationCardinal : public SplineCardinal<Vector3>
{
public:
	PathTranslationCardinal();
	PathTranslationCardinal(const PathTranslationCardinal& rhs);

	void setKey(int k, float time, const Vector3& pos, float tension = 0.5);
};

/*************************************************************
 * PathTranslationBezier
 *
 * A cubic bezier translation path.
 *
 *************************************************************/
class PathTranslationBezier : public SplineBezierCubic<Vector3>
{
public:
	PathTranslationBezier();
	PathTranslationBezier(const PathTranslationBezier& rhs) ;

	void setKey(int k, float time, const Vector3& pos);
	inline void setCPIn(int k, const Vector3& pos) { getCPIn(k) = pos; }
	inline void setCPOut(int k, const Vector3& pos) { getCPOut(k) = pos; }
	void smoothKey(int k, float tension = 0.5, bool smooth_wrap = false);
	void setKeyInTangent(int k, const Vector3& tang);
	void setKeyOutTangent(int k, const Vector3& tang);
};

/*************************************************************
 * PathTranslationTCB
 *
 * A translation path with tension, continuity, bias parameters.
 *
 *************************************************************/
struct PathTranslationTCB : public SplineTCB<Vector3>
{
	PathTranslationTCB();
	PathTranslationTCB(const PathTranslationTCB& rhs);

	void setKey(int k, float time, const Vector3& pos,
		float tension=0.5, float continuity=1.0, float bias=0.0);
};

/*************************************************************
 * PathRotation
 *
 * A cubic bezier rotation path.
 *
 *************************************************************/
class PathRotation : public SplineBezierCubic<Quaternion>
{
public:
	PathRotation();
	PathRotation(const PathRotation& rhs);

	void setKey(int k, float time, const Quaternion& rot);
	void smoothKey(int k, float tension = 0.5, bool smooth_wrap = false);

protected:
	virtual void eval(int seg, float u, Quaternion& out) const;
};

/**********************************************************
 * PathCoordinate
 *
 * A template class that defines a rigid transformation
 * path in 3-space.
 * PathTrans must implement the functions in
 * SplineBase<Vector3>, and PathRot must implement the
 * functions in SplineBase<Quaternion>
 **********************************************************/
template <class PathRot,class PathTrans>
class PathCoordinate
{
public:
	PathCoordinate() { }
	PathCoordinate(const PathCoordinate& rhs) { operator=(rhs); }

	SplineTimeBase::TimeStatus eval(float time, QuaternionRotation& r, Vector3& t) const
	{
		SplineIterator ti(time);
		SplineIterator ri(time);
		return eval(ri,ti,r,t);
	}

	SplineTimeBase::TimeStatus eval(SplineIterator& ri, SplineIterator& ti, QuaternionRotation& r, Vector3& t) const
	{
		assert(ri.t==ti.t);
		rotation.evaluate(ri,r);
		translation.evaluate(ti,t);

		if(ri.t < beginTime()) return SplineTimeBase::Before;
		if(ri.t > endTime()) return SplineTimeBase::After;
		return SplineTimeBase::During;
	}

	inline float beginTime() const { return min(translation.beginTime(),rotation.beginTime()); }
	inline float endTime() const { return max(translation.endTime(),rotation.endTime()); }
	inline float length() const { return endTime() - beginTime();}

	PathRot rotation;
	PathTrans translation;
};

#endif
