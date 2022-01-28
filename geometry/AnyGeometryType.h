#ifndef ANY_GEOMETRY_TYPE_H
#define ANY_GEOMETRY_TYPE_H

#include <KrisLibrary/math3d/geometry3d.h>
#include <memory>
#include <limits.h>

namespace Geometry {

using namespace std;
using namespace Math;
using namespace Math3D;
class DistanceQuerySettings;
class DistanceQueryResult;
class ContactsQuerySettings;
class ContactsQueryResult;

//foward declaration
#ifndef EXTRACT_ROI_FLAG_DECL
#define EXTRACT_ROI_FLAG_DECL
enum {
    ExtractROIFlagIntersection=0x01,
    ExtractROIFlagTouching=0x02,
    ExtractROIFlagWithin=0x04,
    ExtractROIFlagInvert=0x08
};
#endif //EXTRACT_ROI_FLAG_DECL



/** @brief Base class for any 3D-geometry type. 
 * 
 * All known types will derive from this type. 
 * 
 * An implementer should try to implement as many of the methods as
 * possible. Return true if the operation is successful.
 * 
 * All struct pointers will be allocated via new. The caller is responsible
 * for deleting them or assigning to a smart pointer type.
 */
class Geometry3D
{
public:
    /** 
     * List of geometry types
     */
    enum class Type { Primitive, ConvexHull, TriangleMesh, PointCloud, ImplicitSurface, OccupancyGrid, Group };

    static Geometry3D* Make(Type type);
    static Geometry3D* Make(const char* typestr);
    static const char* TypeName(Type type);

    virtual ~Geometry3D() {}
    virtual Type GetType() const=0;
    virtual const char* FileExtension() const { return NULL; }
    virtual vector<string> FileExtensions() const;
    virtual bool Load(const char* fn);
    virtual bool Save(const char* fn) const;
    virtual bool Load(istream& in)=0;
    virtual bool Save(ostream& out) const=0;
    virtual bool Empty() const { return NumElements()==0; }
    virtual size_t NumElements() const=0;
    virtual shared_ptr<Geometry3D> GetElement(int elem) const=0;
    virtual AABB3D GetAABB() const=0;
    virtual bool Transform(const RigidTransform& T);
    virtual bool Transform(const Matrix4& mat)=0;
    virtual Geometry3D* Copy() const=0;
    ///Converts to another geometry type, allocating a new object and returning it
    ///if the conversion is available. 
    ///
    ///If restype==type, returns a copy of this.
    ///
    ///param is interpreted in a type-dependent manner.
    ///
    ///domainExpansion specifies a domain buffer for certain types, such as ImplicitSurface.
    virtual Geometry3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0) const { return NULL; }
    ///Converts from another geometry type, returning true if the conversion is available. 
    ///
    ///param is interpreted in a type-dependent manner.
    ///
    ///domainExpansion specifies a domain buffer for certain types, such as ImplicitSurface.
    virtual bool ConvertFrom(const Geometry3D* geom,Real param=0,Real domainExpansion=0) { return NULL; }
    ///Re-meshes the geometry at the desired resolution, allocating a new object and returning it
    ///
    ///Resolution is interpreted  in a type-dependent manner
    virtual Geometry3D* Remesh(Real resolution,bool refine=true,bool coarsen=true) const { return NULL; }
    ///Merges a set of other geometries. Should be of this type.
    virtual bool Merge(const vector<Geometry3D*>& geoms) { return false; }
    ///Extracts a slice from the geometry at a given plane.  The plane is specified
    ///as the local X-Y plane of the given world coordinates T.  The resulting values are
    ///given in T's local coordinates.
    ///
    ///For point clouds, tol must be > 0.
    virtual Geometry3D* Slice(const RigidTransform& T,Real tol=0) const { return NULL; }
    ///Extracts a region of interest (bounding box) from the geometry.  The region of interest
    ///may be specified in the flag as all geometry intersecting the box, within the box, 
    ///or touching the box.  It is also possible to invert the selection.  See the
    ///ExtractROIFlagX enum for more details.
    virtual Geometry3D* ExtractROI(const AABB3D& bb,int flag=1) const { return NULL; }
    virtual Geometry3D* ExtractROI(const Box3D& bb,int flag=1) const { return NULL; }
};

/** @brief Stores collision geometry for a certain geometry type.
 * 
 * Collision data must be created upon initialization and if Reset()
 * is called.  If the geometry data is changed, Reset() should be called.
 * 
 * Collision checking convention: each type should implement
 * Collides, WithinDistance, and Distance for all types <= my type when
 * possible.
 * 
 * Structures passed in as arguments to receive results should all be empty.
 * 
 * All struct pointers will be allocated via new. The caller is responsible
 * for deleting them or assigning to a smart pointer type.
 */
class Collider3D
{
public:
    typedef Geometry3D::Type Type;

    static Collider3D* Make(shared_ptr<Geometry3D> geom);

    virtual ~Collider3D() {}
    virtual Type GetType() const { return GetData()->GetType(); }
    virtual shared_ptr<Geometry3D> GetData() const=0;
    virtual void Reset() {}
    ///Performs a type conversion, also copying the active transform.
    ///May be a bit faster than Geometry3D.Convert for some conversions
    ///(TriangleMesh->ImplicitSurface, specifically)
    virtual Collider3D* Convert(Type restype,Real param=0,Real domainExpansion=0) { return NULL; }
    ///Returns an axis-aligned bounding box in the world coordinate frame
    ///containing the transformed geometry.  
    virtual AABB3D GetAABB() const;
    ///Returns an oriented bounding box in the world coordinate frame
    ///containing the transformed geometry.  
    virtual Box3D GetBB() const;
    ///Returns a tight bounding box around the data.  Slower than GetAABB 
    ///but tighter.
    virtual AABB3D GetAABBTight() const { return GetAABB(); }
    ///Gets the active transform
    virtual RigidTransform GetTransform() const=0;
    ///Sets the *active* transform without modifying the underlying geometry. 
    virtual void SetTransform(const RigidTransform& T)=0;
    virtual bool Collides(Collider3D* geom,bool& result);
    virtual bool Collides(Collider3D* geom,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) { return WithinDistance(geom,0,elements1,elements2,maxcollisions); }
    virtual bool Contains(const Vector3& pt,bool& result) { return false; }
    virtual bool Distance(const Vector3& pt,Real& result);
    virtual bool Distance(const Vector3& pt,const DistanceQuerySettings& settings,DistanceQueryResult& res);
    virtual bool Distance(Collider3D* geom,const DistanceQuerySettings& settings,DistanceQueryResult& res) { return false; }
    virtual bool WithinDistance(Collider3D* geom,Real d,bool& result);
    virtual bool WithinDistance(Collider3D* geom,Real d,vector<int>& elements1,vector<int>& elements2,size_t maxcollisions=INT_MAX) { return false; }
    virtual bool Contacts(Collider3D* other,const ContactsQuerySettings& settings,ContactsQueryResult& res) { return false; }
    virtual bool RayCast(const Ray3D& r,Real margin,Real& distance,int& element) { return false; }
    /////Computes the furthest point on the geometry in the direction dir
    virtual bool Support(const Vector3& dir,Vector3& pt) { return false; }
    virtual Collider3D* Copy() const;
    virtual Collider3D* ConvertTo(Type restype,Real param=0,Real domainExpansion=0);
    virtual bool ConvertFrom(Collider3D* geom,Real param=0,Real domainExpansion=0);
    virtual bool Merge(const vector<Collider3D*>& geoms);
    ///Extracts a slice from the geometry at a given plane.  The plane is specified
    ///as the local X-Y plane of the given world coordinates T.  The resulting values are
    ///given with data in T's local coordinates, with active transform T.
    ///
    ///For point clouds, tol must be > 0.
    virtual Collider3D* Slice(const RigidTransform& T,Real tol=0) const;
    ///Extracts a region of interest (bounding box) from the geometry.  The region of interest
    ///may be specified in the flag as all geometry intersecting the box, within the box, 
    ///or touching the box.  It is also possible to invert the selection.  See the
    ///ExtractROIFlagX enum for more details.
    virtual Collider3D* ExtractROI(const AABB3D& bb,int flag=1) const;
    virtual Collider3D* ExtractROI(const Box3D& bb,int flag=1) const;
};

class DistanceQuerySettings
{
public:
  DistanceQuerySettings();
  ///Allowable relative and absolute errors
  Real relErr,absErr;
  ///An upper bound on the distance, and if the two objects are farther than this distance the computation may break
  Real upperBound;
};

class DistanceQueryResult
{
public:
  DistanceQueryResult();
  ///flags indicating which elements are filled out
  bool hasPenetration,hasElements,hasClosestPoints,hasDirections;
  ///The distance, with negative values indicating penetration if hasPenetration=true. Otherwise, 0 indicates penetration.
  Real d;
  ///The elements defining the closest points on the geometries
  int elem1,elem2;
  ///The closest points on the two geometries, in world coordinates
  Vector3 cp1,cp2;
  ///The direction from geometry 1 to geometry 2, and the distance from geometry 2 to geometry 1, in world coordinates
  ///These are typically proportional to cp2-cp1 and cp1-cp2, respectively, EXCEPT when the points are exactly
  ///coincident.
  Vector3 dir1,dir2;
  ///If the item is a group, this vector will recursively define the sub-elements
  vector<int> group_elem1,group_elem2;
};

class ContactsQuerySettings
{
public:
  ContactsQuerySettings();
  ///Extra padding on the geometries, padding1 for this object and padding2 for the other other
  Real padding1,padding2;
  ///Maximum number of contacts queried
  size_t maxcontacts;
  ///True if you'd like to cluster the contacts into at most maxcontacts results
  bool cluster;
};

class ContactsQueryResult
{
public:
  struct ContactPair
  {
    ///the depth of the contact, padding included
    Real depth;
    ///the contact points on the padded geometries of object1 and object2, in world coordinates
    Vector3 p1,p2;
    ///the outward contact normal from object 1 pointing into object 2, in world coordinates
    Vector3 n;
    ///the item defining the element to which this point belongs
    int elem1,elem2;
    ///if true, the contact normal can't be estimated accurately
    bool unreliable;
  };

  ContactsQueryResult();
  ContactsQueryResult(const ContactsQueryResult & rhs) = default;
  ContactsQueryResult(ContactsQueryResult&& other) = default;
  ContactsQueryResult& operator = (const ContactsQueryResult& rhs) = default;
  ContactsQueryResult& operator = (ContactsQueryResult&& rhs) = default;
  ///The list of computed contact points
  vector<ContactPair> contacts;
  ///True if clustering was performed
  bool clustered;
};


} //namespace Geometry

#endif //ANY_GEOMETRY_TYPE_H