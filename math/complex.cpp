#include "complex.h"
#include <KrisLibrary/File.h>
#include <iostream>

namespace Math {


Complex::Complex()
{}

Complex::Complex(const Complex& z)
:x(z.x), y(z.y)
{}

Complex::Complex(Real _x)
:x(_x), y(Zero)
{}

Complex::Complex(Real _x, Real _y)
:x(_x), y(_y)
{}



void Complex::setExp (const Complex& z)
{
	Real mag = Exp(z.x);

	Real ctheta = Cos(z.y);
	Real stheta = Sin(z.y);

	x = mag * ctheta;
	y = mag * stheta;
}

bool Complex::setLog (const Complex& z, int n)
{
	Real mag = z.norm();
	if(mag == 0.0)
		return false;
	x = Log(mag);
	y = z.arg() + Two*Pi*n;
	return true;
}

void Complex::setPow (const Complex& z, Real n)
{
	Real mag = z.norm();
	Real theta = z.arg();

	Real cntheta = Cos(n*theta);
	Real sntheta = Sin(n*theta);
	Real powm = Pow(mag, n);

	x = powm * cntheta;
	y = powm * sntheta;
}

void Complex::setPow (const Complex& z, const Complex& w)
{
  Real mag = z.norm();
  Real theta = z.arg();
  Real powm = Pow(mag,w.x);
  Real expt = Exp(-w.y*theta);
  Real phi = w.x*theta;
  if(w.y != 0) {
    Assert(mag != Zero);
    phi += w.y*Log(mag);
  }
  x = powm*expt*Cos(phi);
  y = powm*expt*Sin(phi);
}

bool Complex::Read(File& f)
{
	if(!ReadFile(f,x)) return false;
	if(!ReadFile(f,y)) return false;
	return true;
}

bool Complex::Write(File& f) const
{
	if(!WriteFile(f,x)) return false;
	if(!WriteFile(f,y)) return false;
	return true;
}

Quaternion::Quaternion()
{}

Quaternion::Quaternion(const Quaternion& q)
:w(q.w), x(q.x), y(q.y), z(q.z)
{}

Quaternion::Quaternion(Real W)
:w(W), x(Zero), y(Zero), z(Zero) 
{}

Quaternion::Quaternion(Real W, const Real* im)
:w(W), x(im[0]), y(im[1]), z(im[2])
{}

Quaternion::Quaternion(Real W, Real X, Real Y, Real Z)
:w(W), x(X), y(Y), z(Z)
{}


void Quaternion::mul (const Quaternion& a, const Quaternion& b)
{
	Real A, B, C, D, E, F, G, H;

	A = (a.w + a.x)* (b.w + b.x);
	B = (a.z - a.y)* (b.y - b.z);
	C = (a.x - a.w)* (b.y + b.z);
	D = (a.y + a.z)* (b.x - b.w);
	E = (a.x + a.z)* (b.x + b.y);
	F = (a.x - a.z)* (b.x - b.y);
	G = (a.w + a.y)* (b.w - b.z);
	H = (a.w - a.y)* (b.w + b.z);

	w= B + (-E - F + G + H)*Half, 
	x= A - (E + F + G + H)*Half;
	y=-C + (E - F + G - H)*Half;
	z=-D + (E - F - G + H)*Half;
}

void Quaternion::div (const Quaternion& a, const Quaternion& b)
{
	Quaternion binv;
	binv.setInverse(b);
	mul(a,binv);
}

void Quaternion::madd (const Quaternion& a, const Quaternion& b)
{
	Real A, B, C, D, E, F, G, H;

	A = (a.w + a.x)* (b.w + b.x);
	B = (a.z - a.y)* (b.y - b.z);
	C = (a.x - a.w)* (b.y + b.z);
	D = (a.y + a.z)* (b.x - b.w);
	E = (a.x + a.z)* (b.x + b.y);
	F = (a.x - a.z)* (b.x - b.y);
	G = (a.w + a.y)* (b.w - b.z);
	H = (a.w - a.y)* (b.w + b.z);

	w+= B + (-E - F + G + H)*Half, 
	x+= A - (E + F + G + H)*Half;
	y+=-C + (E - F + G - H)*Half;
	z+=-D + (E - F - G + H)*Half;
}

void Quaternion::setExp(const Quaternion& q)
{
	Real scale = Exp(q.w);
	Real immag = q.imNorm();
	Real immaginv;
	if(immag == Zero) immaginv = Zero;  //it's a real number, zero out the imaginary part
	else immaginv = One / immag;
	Real sm = Sin(immag);
	Real cm = Cos(immag);

	w = scale * cm;
	x = scale * sm * immaginv * q.x;
	y = scale * sm * immaginv * q.y;
	z = scale * sm * immaginv * q.z;
}

bool Quaternion::setLog(const Quaternion& q, int n)
{
	Real mag = q.norm();
	Real immag = q.imNorm();
	if(immag == Zero)
	{
		//there are infinitely (uncountably) many such logarithms, just choose the real case
		if(mag == Zero)
			return false;
		w = Log(mag);
		x = Zero;
		y = Zero;
		z = Zero;
	}
	else
	{
		Real immaginv = One / immag;
		Real arg = Atan2(immag, q.w) + Two*Pi*n;

		w = Log(mag);
		x = immaginv * arg * q.x;
		y = immaginv * arg * q.y;
		z = immaginv * arg * q.z;
	}
	return true;
}

void Quaternion::setPow(const Quaternion& q, Real n)
{
	Real mag = q.norm();
	Real immag = q.imNorm();
	Real immaginv;
	if(immag == Zero) immaginv = Zero;  //it's a real number, zero out the imaginary part
	else immaginv = One / immag;
	Real theta = Atan2(immag, q.w);

	Real cntheta = Cos(n*theta);
	Real sntheta = Sin(n*theta);
	Real powm = Pow(mag, n);

	w = powm * cntheta;
	x = powm * sntheta * immaginv * q.x;
	y = powm * sntheta * immaginv * q.y;
	z = powm * sntheta * immaginv * q.z;
}

bool Quaternion::Read(File& f)
{
  return f.ReadData(data,sizeof(Real)*4);
}

bool Quaternion::Write(File& f) const
{
  return f.WriteData(data,sizeof(Real)*4);
}



std::ostream& operator << (std::ostream& out, const Complex& x)
{
	out << x.x << " " << x.y;
	return out;
}

std::istream& operator >> (std::istream& in, Complex& x)
{
	in >> x.x >> x.y;
	return in;
}

std::ostream& operator << (std::ostream& out, const Quaternion& q)
{
	out << q.w << " " << q.x << " " << q.y << " " << q.z;
	return out;
}

std::istream& operator >> (std::istream& in, Quaternion& q)
{
	in >> q.w >> q.x >> q.y >> q.z;
	return in;
}

} //namespace Math
