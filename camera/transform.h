#ifndef CAMERA_TRANSFORM_H
#define CAMERA_TRANSFORM_H

#include <KrisLibrary/math3d/primitives.h>
using namespace Math3D;

//#warning "transform.h has been superceded by camera.h"

void SetOrbitTransform(const Vector3& rot, const Vector3& target, float dist, RigidTransform& xform);
void SetFreeTransform(const Vector3& pos, const Vector3& rot, RigidTransform& xform);
void SetTargetTransform(const Vector3& pos, const Vector3& tgt, const Vector3& up, RigidTransform& xform);

void GetOrbitTransform(const RigidTransform& xform, Vector3& rot, Vector3& target, float dist = One);
void GetFreeTransform(const RigidTransform& xform, Vector3& pos, Vector3& rot);
void GetTargetTransform(const RigidTransform& xform, Vector3& pos, Vector3& tgt, Vector3& up, float tgtDist = One);

#endif
