#include "polar.h"

namespace Math3D {

void PolarToRectangular(const Vector2& polar, Vector2& rect)
{
	Real costheta = Cos(polar.y);
	Real sintheta = Sin(polar.y);
	rect.x = polar.x * costheta;
	rect.y = polar.x * sintheta;
}

void RectangularToPolar(const Vector2& rect, Vector2& polar)
{
	polar.x = Sqrt(Sqr(rect.x) + Sqr(rect.y));
	polar.y = Atan2(rect.y, rect.x);
}

void CylindricalToRectangular(const Vector3& cyl, Vector3& rect)
{
	Real costheta = Cos(cyl.y);
	Real sintheta = Sin(cyl.y);
	rect.x = cyl.x * costheta;
	rect.y = cyl.x * sintheta;
	rect.z = cyl.z;
}

void RectangularToCylindrical(const Vector3& rect, Vector3& cyl)
{
	cyl.x = Sqrt(Sqr(rect.x) + Sqr(rect.y));
	cyl.y = Atan2(rect.y, rect.x);
	cyl.z = rect.z;
}

void SphericalToRectangular(const Vector3& sphere, Vector3& rect)
{
	Real costheta = Cos(sphere.y);
	Real sintheta = Sin(sphere.y);
	Real cosphi = Cos(sphere.z);
	Real sinphi = Sin(sphere.z);
	rect.x = sphere.x * costheta * cosphi;
	rect.y = sphere.x * sintheta * cosphi;
	rect.z = sphere.x * sinphi;
}

void RectangularToSpherical(const Vector3& rect, Vector3& sphere)
{
	sphere.x = Sqrt(Sqr(rect.x) + Sqr(rect.y) + Sqr(rect.z));
	sphere.y = Atan2(rect.y, rect.x);
	sphere.z = Atan2(rect.z, Sqrt(Sqr(rect.x) + Sqr(rect.y)));
}



void PolarDR(const Vector2& polar, Vector2& drect)
{
	drect.x = Cos(polar.y);
	drect.y = Sin(polar.y);
}

void PolarDTheta(const Vector2& polar, Vector2& drect)
{
	drect.x = -polar.x*Sin(polar.y);
	drect.y = polar.x*Cos(polar.y);
}

void CylindricalDR(const Vector3& cyl, Vector3& drect)
{
	drect.x = Cos(cyl.y);
	drect.y = Sin(cyl.y);
	drect.z = Zero;
}

void CylindricalDTheta(const Vector3& cyl, Vector3& drect)
{
	drect.x = -cyl.x*Sin(cyl.y);
	drect.y = cyl.x*Cos(cyl.y);
	drect.z = Zero;
}

void CylindricalDZ(const Vector3& cyl, Vector3& drect)
{
	drect.set(Zero,Zero,One);
}

void SphericalDR(const Vector3& sphere, Vector3& drect);
void SphericalDTheta(const Vector3& sphere, Vector3& drect);
void SphericalDPhi(const Vector3& sphere, Vector3& drect);

} //namespace Math3D
