#include "CollisionPrimitive.h"
#include <KrisLibrary/Logger.h>
#include <iostream>
using namespace Geometry;
using namespace std;
typedef ContactsQueryResult::ContactPair ContactPair;

DECLARE_LOGGER(Geometry)

Geometry3DPrimitive::Geometry3DPrimitive()
{}

Geometry3DPrimitive::Geometry3DPrimitive(const GeometricPrimitive3D& _data)
: data(_data)
{}

Geometry3DPrimitive::Geometry3DPrimitive(GeometricPrimitive3D&& _data)
: data(_data)
{}

Geometry3DPrimitive::~Geometry3DPrimitive ()
{}

bool Geometry3DPrimitive::Load(istream& in)
{
    in >> data;
    if(in) return true;
    return false;
}
bool Geometry3DPrimitive::Save(ostream& out) const
{
    out << data;
    return true;
}
AABB3D Geometry3DPrimitive::GetAABB() const
{
    return data.GetAABB();
}

bool Geometry3DPrimitive::Support(const Vector3& dir,Vector3& pt) const
{
    if(!data.SupportsClosestPoints(GeometricPrimitive3D::Point)) return false;
    Vector3 farpt = 100000*dir;
    Vector3 tempdir;
    data.ClosestPoints(farpt,pt,tempdir);
    return true;
}

bool Geometry3DPrimitive::Transform(const Matrix4& mat)
{
    return data.Transform(mat);
}

Geometry3D* Geometry3DPrimitive::ConvertTo(Type restype,Real param,Real domainExpansion) const
{
    return NULL;
}

Collider3DPrimitive::Collider3DPrimitive(shared_ptr<Geometry3DPrimitive> _data)
:data(_data)
{
  T.setIdentity();
}

Box3D Collider3DPrimitive::GetBB() const
{
    Box3D b;
    b.setTransformed(data->data.GetBB(), T);
    return b;
}      

AABB3D Collider3DPrimitive::GetAABB() const
{
    const GeometricPrimitive3D &g = data->data;
    GeometricPrimitive3D gT(g);
    gT.Transform(T);
    AABB3D res = gT.GetAABB();
    return res;
}


bool Collider3DPrimitive::Distance(const Vector3 &pt,Real& result)
{
    Vector3 ptlocal;
    T.mulInverse(pt, ptlocal);
    result = data->data.Distance(ptlocal);
    return true;
}

bool Collider3DPrimitive::Contains(const Vector3& pt,bool& result) 
{
    Vector3 ptlocal;
    T.mulInverse(pt, ptlocal);
    result = data->data.Collides(ptlocal);
    return true;
}

bool Collider3DPrimitive::Distance(const Vector3 &pt, const DistanceQuerySettings &settings,DistanceQueryResult& res)
{
    res.hasClosestPoints = true;
    res.hasElements = true;
    res.elem2 = 0;
    res.cp2 = pt;

    Vector3 ptlocal;
    T.mulInverse(pt, ptlocal);
    const GeometricPrimitive3D &g = data->data;
    if(!g.SupportsClosestPoints(GeometricPrimitive3D::Point)) return false;
    res.elem1 = 0;
    res.hasDirections = true;
    res.d = g.ClosestPoints(ptlocal, res.cp1, res.dir1);
    res.dir2.setNegative(res.dir1);
    Transform1(res, T);
    return true;
}

bool Collides(const GeometricPrimitive3D &a, const GeometricPrimitive3D &b, Real margin)
{
  if (margin == 0)
    return a.Collides(b);
  return a.Distance(b) <= margin;
}

bool Collider3DPrimitive::WithinDistance(Collider3D* geom,Real d,
              vector<int> &elements1, vector<int> &elements2, size_t maxContacts)
{
    GeometricPrimitive3D aw = data->data;
    if (aw.type == GeometricPrimitive3D::Empty) return true; //no collision
    aw.Transform(T);
    switch (geom->GetType())
    {
    case Type::Primitive:
    {
        GeometricPrimitive3D& bw = dynamic_cast<Collider3DPrimitive*>(geom)->data->data;
        bw.Transform(geom->GetTransform());
        if (::Collides(aw, bw, d))
        {
            elements1.push_back(0);
            elements2.push_back(0);
        }
        return true;
    }
    default:
        return false;
    }
}

bool Collider3DPrimitive::Distance(Collider3D* geom, const DistanceQuerySettings &settings, DistanceQueryResult& res)
{
    if(geom->GetType()==Type::Primitive) {
        const GeometricPrimitive3D& b = dynamic_cast<Collider3DPrimitive*>(geom)->data->data;
        res.hasElements = true;
        res.elem1 = 0;
        res.elem2 = 0;
        res.hasPenetration = true;
        if (data->data.SupportsClosestPoints(b.type))
        {
            res.hasClosestPoints = true;
            res.hasDirections = true;
            res.d = data->data.ClosestPoints(b, res.cp1, res.dir1);
            SetCP2(res);
        }
        else
            res.d = data->data.Distance(b);
        return true;
    }
    return false;
}

bool Collider3DPrimitive::RayCast(const Ray3D &r, Real margin, Real &distance, int &element)
{
    RigidTransform Tinv;
    Tinv.setInverse(T);
    Ray3D rlocal;
    rlocal.setTransformed(r, Tinv);
    Vector3 localpt;
    if (data->data.RayCast(rlocal, localpt))
    {
        distance = localpt.distance(rlocal.source);
        element = 0;
        //TODO: this isn't perfect if the margin is > 0 -- will miss silouettes
        distance -= margin;
    }
    return true;
}

void PrimitivePrimitiveContacts(GeometricPrimitive3D &g1, const RigidTransform &T1, Real outerMargin1, GeometricPrimitive3D &g2, const RigidTransform &T2, Real outerMargin2, vector<ContactPair> &contacts, size_t maxcontacts)
{
  contacts.resize(0);
  if (maxcontacts == 0)
    return;
  if (!g1.SupportsDistance(g2.type))
  {
    LOG4CXX_WARN(GET_LOGGER(Geometry), "TODO: primitive collisions of type " << g1.TypeName() << " to " << g2.TypeName());
    return;
  }
  if ((g1.type != GeometricPrimitive3D::Point && g1.type != GeometricPrimitive3D::Sphere) && (g2.type == GeometricPrimitive3D::Point || g2.type == GeometricPrimitive3D::Sphere))
  {
    //do this the other way around
    PrimitivePrimitiveContacts(g2, T2, outerMargin2, g1, T1, outerMargin1, contacts, maxcontacts);
    for (auto &c : contacts)
      ReverseContact(c);
    return;
  }
  GeometricPrimitive3D tg1 = g1, tg2 = g2;
  tg1.Transform(T1);
  tg2.Transform(T2);
  if (g1.type != GeometricPrimitive3D::Point && g1.type != GeometricPrimitive3D::Sphere)
  {
    //TODO: try copying into ODE data structures?
    LOG4CXX_WARN(GET_LOGGER(Geometry), "Contact computations between primitives " << g1.TypeName() << " and " << g2.TypeName() << " not yet supported");
    return;
  }
  else
  {
    Sphere3D s;
    if (g1.type == GeometricPrimitive3D::Point)
    {
      s.center = *AnyCast<Point3D>(&tg1.data);
      s.radius = 0;
    }
    else
    {
      s = *AnyCast<Sphere3D>(&tg1.data);
    }
    if (tg2.Distance(s.center) > s.radius + outerMargin1 + outerMargin2)
      return;
    vector<double> params = tg2.ClosestPointParameters(s.center);
    Vector3 p2 = tg2.ParametersToPoint(params);
    //normal out from sphere to g2
    Vector3 n = p2 - s.center;
    Real d = n.norm();
    if (FuzzyZero(d))
    {
      //penetrating all the way to center?
      n = tg2.ParametersToNormal(params);
    }
    else
      n /= d;
    Vector3 p1 = s.center + n * s.radius;
    p2 -= outerMargin2 * n;
    p1 += outerMargin1 * n;
    contacts.resize(1);
    contacts[0].depth = p1.distance(p2);
    contacts[0].p1 = p1;
    contacts[0].p2 = p2;
    contacts[0].n = n;
    contacts[0].elem1 = contacts[0].elem2 = 0;
    contacts[0].unreliable = false;
  }
}

bool Collider3DPrimitive::Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) 
{
    if(other->GetType() == Type::Primitive) {
        auto* prim = dynamic_cast<Collider3DPrimitive*>(other);
        PrimitivePrimitiveContacts(data->data, T, settings.padding1,
                                prim->data->data, prim->T, settings.padding2, res.contacts, settings.maxcontacts);
        return true;
    }
    return false;
}