#include <KrisLibrary/Logger.h>
#include "Geodesic.h"
using namespace Meshing;

//assumes a is at the origin, b is on the x axis
Real TriangulateDistance(Real bx,Real da,Real db,const Vector2& c)
{
  Real dc;
  Real bx2=Sqr(bx);
  Real A=Two*Sqr(da)*bx2-Sqr(bx2) + Two*Sqr(db)*bx2;
  Real B=Sqr(Sqr(da)-Sqr(db));
  if(A < B) {
    /*
    LOG4CXX_ERROR(KrisLibrary::logger(),"TriangulateDistance: circles do not intersect");
    LOG4CXX_ERROR(KrisLibrary::logger(),"bx="<<bx<<", A="<<A<<", B="<<B);
    LOG4CXX_INFO(KrisLibrary::logger(),"da="<<da<<", db="<<db);
    LOG4CXX_INFO(KrisLibrary::logger(),"a="<<a<<", b="<<b);
    */
    Real x;
    if(da+db <= bx) x=Half*(da+bx-db);
    else if(db+bx <= da) x=Half*(da+bx+db);
    else if(da+bx <= db) x=Half*(-da+bx-db);
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TriangulateDistance: circles don't intersect, and numerical error?");
      Real s11 = bx+db-da;
      Real s00 = bx-db+da;
      Real s10 = bx+db-da;
      Real s01 = bx-db+da;
      Real min = Min(Min(s11,s00),Min(s10,s01));
      if(min==s11) x=Half*(da+bx+db);
      else if(min==s00) x=Half*(-da+bx-db);
      else if(min==s10) x=Half*(da+bx-db);
      else x=Half*(-da+bx+db);
    }
    dc=Sqrt(Sqr(c.x-x)+Sqr(c.y));
    return dc;
  }
  Real x=Half*(bx2+Sqr(da)-Sqr(db))/bx;
  Real y=-Half*Sqrt(A-B)/bx;  //choose the negative version
  dc=Sqrt(Sqr(c.x-x)+Sqr(c.y-y));
  return dc;
}

Real TriangulateDistance(const Vector2& a,const Vector2& b,Real da,Real db,const Vector2& c)
{
  Vector2 ba=b-a,ca=c-a;
  Real bx2=ba.normSquared();
  Real bx=Sqrt(bx2);
  Real cx=dot(ca,ba)/bx;
  Real cy=ca.distance(cx/bx*ba);
  return TriangulateDistance(bx,da,db,Vector2(cx,cy));
}

Real TriangulateDistance(const Vector3& a,const Vector3& b,Real da,Real db,const Vector3& c)
{
  Real dc;
  Vector3 ba=b-a,ca=c-a;
  Real bx2=ba.normSquared();
  Real bx=Sqrt(bx2);
  Real cx=dot(ca,ba)/bx;
  Real cy=ca.distance(cx/bx*ba);
  Real A=Two*Sqr(da)*bx2-Sqr(bx2) + Two*Sqr(db)*bx2;
  Real B=Sqr(Sqr(da)-Sqr(db));
  if(A < B) {
    /*
    LOG4CXX_ERROR(KrisLibrary::logger(),"TriangulateDistance: circles do not intersect");
    LOG4CXX_ERROR(KrisLibrary::logger(),"bx="<<bx<<", A="<<A<<", B="<<B);
    LOG4CXX_INFO(KrisLibrary::logger(),"da="<<da<<", db="<<db);
    LOG4CXX_INFO(KrisLibrary::logger(),"a="<<a<<", b="<<b);
    */
    Real x;
    if(da+db <= bx) x=Half*(da+bx-db);
    else if(db+bx <= da) x=Half*(da+bx+db);
    else if(da+bx <= db) x=Half*(-da+bx-db);
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"TriangulateDistance: circles don't intersect, and numerical error?");
      Real s11 = bx+db-da;
      Real s00 = bx-db+da;
      Real s10 = bx+db-da;
      Real s01 = bx-db+da;
      Real min = Min(Min(s11,s00),Min(s10,s01));
      if(min==s11) x=Half*(da+bx+db);
      else if(min==s00) x=Half*(-da+bx-db);
      else if(min==s10) x=Half*(da+bx-db);
      else x=Half*(-da+bx+db);
    }
    dc=Sqrt(Sqr(cx-x)+Sqr(cy));
    return dc;
  }
  Real x=Half*(bx2+Sqr(da)-Sqr(db))/bx;
  Real y=-Half*Sqrt(A-B)/bx;  //choose the negative version
  dc=Sqrt(Sqr(cx-x)+Sqr(cy-y));
  return dc;
}

Real WeightedTriangulateDistance(const Vector3& a,const Vector3& b,Real da,Real db,const Vector3& c,Real weight)
{
  Real dc;
  Vector3 ba=b-a,ca=c-a;
  Real bx2=ba.normSquared();
  Real bx=Sqrt(bx2);
  Real cx=dot(ca,ba)/bx;
  Real cy=ca.distance(cx/bx*ba);
  Real A=Two*Sqr(da)*bx2-Sqr(bx2) + Two*Sqr(db)*bx2;
  Real B=Sqr(Sqr(da)-Sqr(db));
  if(A < B) {
    Real x;
    if(da+db <= bx) x=Half*(da+bx-db);
    else if(db+bx <= da) x=Half*(da+bx+db);
    else if(da+bx <= db) x=Half*(-da+bx-db);
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"WeightedTriangulateDistance: circles don't intersect, and numerical error?");
      Real s11 = bx+db-da;
      Real s00 = bx-db+da;
      Real s10 = bx+db-da;
      Real s01 = bx-db+da;
      Real min = Min(Min(s11,s00),Min(s10,s01));
      if(min==s11) x=Half*(da+bx+db);
      else if(min==s00) x=Half*(-da+bx-db);
      else if(min==s10) x=Half*(da+bx-db);
      else x=Half*(-da+bx+db);
    }
    dc=Sqrt(Sqr(cx-x)+Sqr(cy))*weight;
    Assert(dc >= da && dc >= db);
    return dc;
  }
  Real x=Half*(bx2+Sqr(da)-Sqr(db))/bx;
  Real y=-Half*Sqrt(A-B)/bx;  //choose the negative version
  Real u=x+(cx-x)/(cy-y);
  dc=weight*Sqrt(Sqr(cx-u)+Sqr(cy));
  dc+=Sqrt(Sqr(x-u)+Sqr(y));
  return dc;
}

Vector2 UnfoldTriangle(const TriMesh& mesh,int tri,int edge,const Vector2& e1,const Vector2& e2)
{
  const Vector3& v = mesh.verts[mesh.tris[tri][edge]];
  const Vector3& e10 = mesh.verts[mesh.tris[tri][(edge+1)%3]];
  const Vector3& e20 = mesh.verts[mesh.tris[tri][(edge+2)%3]];
  Vector3 n=mesh.TriangleNormal(tri);
  Vector3 p0 = e20-e10;
  p0.inplaceNormalize();
  Vector3 q0; q0.setCross(n,p0);
  Vector2 p=e2-e1;
  p.inplaceNormalize();
  Vector2 q; q.setPerpendicular(p);
  Real horiz = dot(p0,v-e10);
  Real vert = dot(q0,v-e10);
  return e1 + horiz*p + vert*q;
}

void GetTriangleBasis(const TriMesh& mesh,int tri,Vector3& xb,Vector3& yb)
{
  Vector3 n=mesh.TriangleNormal(tri);
  int a,b;
  a=mesh.tris[tri].a;
  b=mesh.tris[tri].b;
  xb=mesh.verts[b]-mesh.verts[a];
  xb.inplaceNormalize();
  yb.setCross(n,xb);
  Assert(FuzzyEquals(yb.norm(),One));
}

int ObtuseVertex(const Vector3& a,const Vector3& b,const Vector3& c)
{
  Vector3 ba=b-a;
  Vector3 cb=c-b;
  Vector3 ac=a-c;
  if(ba.dot(ac) > 0)  //obtuse at a
    return 0;
  else if(ba.dot(cb) > 0) //obtuse at b
    return 1;
  else if(cb.dot(ac) > 0) //obtuse at c
    return 2;
  return -1;
}

ApproximateGeodesic::ApproximateGeodesic(const TriMeshWithTopology& _mesh)
  :mesh(_mesh)
{
  ComputeVirtualEdges();
}

void ApproximateGeodesic::ComputeVirtualEdges()
{
  virtualEdges.resize(mesh.tris.size());
  incomingVirtualEdges.resize(mesh.verts.size());
  /*
  for(size_t i=0;i<mesh.tris.size();i++) {
    virtualEdges[i].vertex1=-1;
    virtualEdges[i].vertex2=-1;
  }
  return;
  */
  for(size_t i=0;i<mesh.tris.size();i++) {
    virtualEdges[i].vertex1=-1;
    virtualEdges[i].vertex2=-1;
    const Vector3& a=mesh.verts[mesh.tris[i].a];
    const Vector3& b=mesh.verts[mesh.tris[i].b];
    const Vector3& c=mesh.verts[mesh.tris[i].c];
    int obtuse = ObtuseVertex(a,b,c);
    if(obtuse < 0) continue;

    int o=mesh.tris[i][obtuse];
    //LOG4CXX_INFO(KrisLibrary::logger(),"Obtuse vertex "<<o<<" on tri "<<i);
    int a1,a2;
    mesh.tris[i].getCompliment(obtuse,a1,a2);
    Vector3 bound1,bound2;   //boundk^T*x >= boundk^T*o
    bound1 = mesh.verts[a1]-mesh.verts[o];
    bound2 = mesh.verts[a2]-mesh.verts[o];
    //unfold vertices onto the basis of triangle i
    Vector3 xb,yb;  //get the standard basis of triangle
    GetTriangleBasis(mesh,i,xb,yb);
    //express bound in terms of standard basis
    Vector2 b1p(xb.dot(bound1),yb.dot(bound1));
    Vector2 b2p(xb.dot(bound2),yb.dot(bound2));
    Vector2 op(xb.dot(mesh.verts[o]),yb.dot(mesh.verts[o]));
    //edge in 2d space
    Vector2 a1p(xb.dot(mesh.verts[a1]),yb.dot(mesh.verts[a1]));
    Vector2 a2p(xb.dot(mesh.verts[a2]),yb.dot(mesh.verts[a2]));
    Vector2 supp;
    //LOG4CXX_INFO(KrisLibrary::logger(),"Vertex folded out to "<<op);
    //rotate the "spoke" a2 about a1 until a supporting vertex is found
    int spoke=a2;
    Vector2 spokePos=a2p;
    int triangle = i;
    int opposite=o;
    int numHops=1;
    while(spoke != o) {
      int oindex=mesh.tris[triangle].getIndex(opposite);
      Assert(oindex!=-1);
      int neighbor=mesh.triNeighbors[triangle][oindex];
      if(neighbor==-1) break;
      int nedge=mesh.triNeighbors[neighbor].getIndex(triangle);
      Assert(nedge!=-1);
      int support=mesh.tris[neighbor][nedge];
      Vector2 supp = UnfoldTriangle(mesh,neighbor,nedge,spokePos,a1p);  //a1-spoke needs to be reversed
      if(supp.dot(b1p) > op.dot(b1p)) {
	//move spoke to support?
	//LOG4CXX_INFO(KrisLibrary::logger(),"Linking to "<<o<<" with vertex "<<support<<", "<<numHops<<" hops");
	//LOG4CXX_INFO(KrisLibrary::logger(),"Folded out to "<<supp<<" with basis "<<xb<<", "<<yb);
	incomingVirtualEdges[support].push_back(i);
	virtualEdges[i].vertex1=support;
	virtualEdges[i].planePos1=supp;
	break;
      }
      else {
	opposite = spoke;
	spoke = support;
	spokePos = supp;
	triangle = neighbor;
	numHops++;
      }
    }

    //rotate about a2 until a supporting vertex is found
    spoke=a1;
    spokePos=a1p;
    triangle = i;
    opposite=o;
    numHops=1;
    while(spoke != o) {
      int oindex=mesh.tris[triangle].getIndex(opposite);
      Assert(oindex!=-1);
      int neighbor=mesh.triNeighbors[triangle][oindex];
      if(neighbor==-1) break;
      int nedge=mesh.triNeighbors[neighbor].getIndex(triangle);
      Assert(nedge!=-1);
      int support=mesh.tris[neighbor][nedge];
      Vector2 supp = UnfoldTriangle(mesh,neighbor,nedge,a2p,spokePos);  //a1-spoke needs to be reversed
      if(supp.dot(b2p) > op.dot(b2p)) {
	//move spoke to support?
	//LOG4CXX_INFO(KrisLibrary::logger(),"Linking to "<<o<<" with vertex "<<support<<", "<<numHops<<" hops");
	//LOG4CXX_INFO(KrisLibrary::logger(),"Folded out to "<<supp<<" with basis "<<xb<<", "<<yb);
	incomingVirtualEdges[support].push_back(i);
	virtualEdges[i].vertex2=support;
	virtualEdges[i].planePos2=supp;
	break;
      }
      else {
	opposite = spoke;
	spoke = support;
	spokePos = supp;
	triangle = neighbor;
	numHops++;
      }
    }
  }
}

void ApproximateGeodesic::ExpandVert(int v)
{
  for(size_t i=0;i<mesh.incidentTris[v].size();i++) {
    int t=mesh.incidentTris[v][i];
    int vi=mesh.tris[t].getIndex(v);
    Assert(vi >= 0);
    int v2,v3;  //opposite vertices
    mesh.tris[t].getCompliment(vi,v2,v3);
    //make v2 the visited one
    if(vertColor[v2] < vertColor[v3]) swap(v2,v3);
    if(vertColor[v2]==2 && vertColor[v3]!=2) {  //propagate to v3
      Real d;
      if(triangleWeights.empty())
	d=TriangulateDistance(mesh.verts[v],mesh.verts[v2],vertCosts[v],vertCosts[v2],mesh.verts[v3]);
      else
	d=WeightedTriangulateDistance(mesh.verts[v],mesh.verts[v2],vertCosts[v],vertCosts[v2],mesh.verts[v3],triangleWeights[t]);
      UpdateDistance(v3,d);
    }
    else if(vertColor[v2]<2) {  //triangle is obtuse!
      //UpdateDistance(v2,vertCosts[v]+mesh.verts[v].distance(mesh.verts[v2]));
      //UpdateDistance(v3,vertCosts[v]+mesh.verts[v].distance(mesh.verts[v3]));
      //must be obtuse at v2 or v3...
      int obtuse=ObtuseVertex(mesh.verts[v],mesh.verts[v2],mesh.verts[v3]);
      if(obtuse < 0) continue;
      if(mesh.tris[t][obtuse]!=v2 && mesh.tris[t][obtuse]!=v3) continue;
      if(mesh.tris[t][obtuse]==v3) swap(v2,v3);
      //2 options for support
      int vs=virtualEdges[t].vertex1,vs2=virtualEdges[t].vertex2;
      Vector2 ps=virtualEdges[t].planePos1,ps2=virtualEdges[t].planePos2;
      if(vs == -1 && vs2 == -1) continue; //no support
      if(vs == -1) { vs=vs2; ps=ps2; }
      if(vertColor[vs] < 2) { vs=vs2; ps=ps2; }
      //LOG4CXX_INFO(KrisLibrary::logger(),"Updating distance "<<v2<<" out of link "<<vs<<", adj "<<v);
      if(vertColor[vs]==2) {
	Vector3 xb,yb;
	GetTriangleBasis(mesh,t,xb,yb);
	Vector2 p0(dot(xb,mesh.verts[v]),dot(yb,mesh.verts[v]));
	Vector2 p2(dot(xb,mesh.verts[v2]),dot(yb,mesh.verts[v2]));
	Real d;
	if(triangleWeights.empty())
	  d=TriangulateDistance(p0,ps,vertCosts[v],vertCosts[vs],p2);
	else
	  FatalError("Weight not done yet");
	//d=WeightedTriangulateDistance(pv,vertCosts[v],vertCosts[vs],mesh.verts[v2],p2);
	UpdateDistance(v2,d);
      }
    }
  }
  //propagate to incoming virtual edges
  for(size_t i=0;i<incomingVirtualEdges[v].size();i++) {
    int t=incomingVirtualEdges[v][i];
    Assert(virtualEdges[t].vertex1 == v || virtualEdges[t].vertex2 == v);
    const IntTriple& tri=mesh.tris[t];
    int obtuse=ObtuseVertex(mesh.verts[tri.a],mesh.verts[tri.b],mesh.verts[tri.c]);
    Assert(obtuse != -1);
    int o=tri[obtuse];
    if(vertColor[o] == 2) continue;  //already computed
    //propagate distances to obtuse vertex
    int vacute1,vacute2;
    tri.getCompliment(obtuse,vacute1,vacute2);
    if(vertColor[vacute1] < vertColor[vacute2]) swap(vacute1,vacute2);
    if(vertColor[vacute1]==2) { //can propagate
      Vector3 xb,yb;
      GetTriangleBasis(mesh,t,xb,yb);
      Vector2 pv=(virtualEdges[t].vertex1==v?virtualEdges[t].planePos1:virtualEdges[t].planePos2);
      Vector2 po(dot(xb,mesh.verts[o]),dot(yb,mesh.verts[o]));
      Vector2 p1(dot(xb,mesh.verts[vacute1]),dot(yb,mesh.verts[vacute1]));
      Real d;
      if(triangleWeights.empty())
	d=TriangulateDistance(p1,pv,vertCosts[vacute1],vertCosts[v],po);
      else
	FatalError("Weight not done yet");
      //LOG4CXX_INFO(KrisLibrary::logger(),"Updating distance "<<o<<" from virtual link "<<v<<", adj "<<vacute1);
      UpdateDistance(o,d);
    }
  }
}

void ApproximateGeodesic::UpdateDistance(int v,Real d)
{
  if(vertColor[v]==0 || (vertColor[v]==1 && d<vertCosts[v])) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Updating cost["<<v<<"] to "<<d);
    bool existedBefore=(vertColor[v]==1);
    vertColor[v]=1;
    vertCosts[v]=d;
    if(existedBefore) h.adjust(v,-d);
    else h.push(v,-d);
  } 
}

void ApproximateGeodesic::SolveFromVertex(int v)
{
  if(!triangleWeights.empty()) Assert(triangleWeights.size()==mesh.tris.size());
  Assert(v < (int)mesh.verts.size());

  vertCosts.resize(mesh.verts.size(),Inf);
  vertColor.resize(mesh.verts.size());
  fill(vertColor.begin(),vertColor.end(),0);

  h.init(mesh.verts.size());
  vertCosts[v]=0;
  vertColor[v]=2;  //closed
  if(triangleWeights.empty()) {
    for(size_t i=0;i<mesh.vertexNeighbors[v].size();i++) {
      int adj=mesh.vertexNeighbors[v][i];
      Real d=mesh.verts[v].distance(mesh.verts[adj]);
      vertCosts[adj]=d;
      vertColor[adj]=1;  //fringe
      h.push(adj,-d);
    }
  }
  else {  //consider triangle weights
    for(size_t i=0;i<mesh.incidentTris[v].size();i++) {
      int t=mesh.incidentTris[v][i];
      int vi=mesh.tris[t].getIndex(v);
      int v2,v3;  //opposite vertices
      mesh.tris[t].getCompliment(vi,v2,v3);
      Real d=mesh.verts[v].distance(mesh.verts[v2])*triangleWeights[t];
      UpdateDistance(v2,d);
      d=mesh.verts[v].distance(mesh.verts[v3])*triangleWeights[t];
      UpdateDistance(v3,d);
    }
  }

  while(!h.empty()) {
    int v=h.top(); h.pop();
    vertColor[v]=2;  //closed
    //if v connects to any closed vertices, propagate that edge to fringe vertices
    ExpandVert(v);
  }
  for(size_t i=0;i<vertColor.size();i++) {
    if(vertColor[i] != 2)
      LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't propagate to vertex"<<i);
  }
}

void ApproximateGeodesic::SolveFromTri(int tri,const Vector3& pt)
{
  if(!triangleWeights.empty()) Assert(triangleWeights.size()==mesh.tris.size());
  Assert(tri < (int)mesh.tris.size());

  vertCosts.resize(mesh.verts.size());
  vertColor.resize(mesh.verts.size());
  fill(vertColor.begin(),vertColor.end(),0);

  h.init(mesh.verts.size());
  for(int k=0;k<3;k++) {
    int v=mesh.tris[tri][k];
    vertCosts[v]=pt.distance(mesh.verts[v]);
    if(!triangleWeights.empty())
      vertCosts[v]*=triangleWeights[tri];
    vertColor[v]=2;  //closed
  }
  for(int k=0;k<3;k++) {
    int v=mesh.tris[tri][k];
    ExpandVert(v);
  }

  while(!h.empty()) {
    int v=h.top(); h.pop();
    vertColor[v]=2;  //closed
    //if v connects to any closed vertices, propagate that edge to fringe vertices
    ExpandVert(v);
  }
}

Real ApproximateGeodesic::Distance(int tri,const Vector3& pt) const
{
  int a=mesh.tris[tri].a;
  int b=mesh.tris[tri].b;
  int c=mesh.tris[tri].c;
  Assert(a>=0&&a<(int)vertCosts.size());
  Assert(b>=0&&b<(int)vertCosts.size());
  Assert(c>=0&&c<(int)vertCosts.size());
  /*
  if(vertColor[a]==0 || vertColor[b]==0 || vertColor[c]==0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Mesh was not connected to triangle "<<tri);
    return Inf;
  }
  */
  if(triangleWeights.empty()) {
    return TriangulateDistance(mesh.verts[a],mesh.verts[b],
			       vertCosts[a],vertCosts[b],
			       pt);
  }
  else {
    return WeightedTriangulateDistance(mesh.verts[a],mesh.verts[b],
				       vertCosts[a],vertCosts[b],
				       pt,triangleWeights[tri]);
  }
}
