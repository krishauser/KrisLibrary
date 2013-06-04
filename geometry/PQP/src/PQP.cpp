/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Gottschalk, E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#include <stdio.h>
#include <string.h>
#include "PQP.h"
#include "BVTQ.h"
#include "Build.h"
#include "MatVec.h"
#include "TriDist.h"
#include <assert.h>

enum BUILD_STATE
{ 
  PQP_BUILD_STATE_EMPTY,     // empty state, immediately after constructor
  PQP_BUILD_STATE_BEGUN,     // after BeginModel(), state for adding triangles
  PQP_BUILD_STATE_PROCESSED  // after tree has been built, ready to use
};

PQP_Model::PQP_Model()
{
  // no bounding volume tree yet

  b = 0;  
  num_bvs_alloced = 0;
  num_bvs = 0;

  // no tri list yet

  tris = 0;
  num_tris = 0;
  num_tris_alloced = 0;

  build_state = PQP_BUILD_STATE_EMPTY;
}

PQP_Model::~PQP_Model()
{
  if (b != NULL)
    delete [] b;
  if (tris != NULL)
    delete [] tris;
}

int
PQP_Model::BeginModel(int n)
{
  // reset to initial state if necessary

  if (build_state != PQP_BUILD_STATE_EMPTY) 
  {
    delete [] b;
    delete [] tris;
  
    num_tris = num_bvs = num_tris_alloced = num_bvs_alloced = 0;
  }

  // prepare model for addition of triangles

  if (n <= 0) n = 8;
  num_tris_alloced = n;
  tris = new Tri[n];
  if (!tris) 
  {
    fprintf(stderr, "PQP Error!  Out of memory for tri array on "
                    "BeginModel() call!\n");
    return PQP_ERR_MODEL_OUT_OF_MEMORY;  
  }

  // give a warning if called out of sequence

  if (build_state != PQP_BUILD_STATE_EMPTY)
  {
    fprintf(stderr,
            "PQP Warning! Called BeginModel() on a PQP_Model that \n"
            "was not empty. This model was cleared and previous\n"
            "triangle additions were lost.\n");
    build_state = PQP_BUILD_STATE_BEGUN;
    return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  build_state = PQP_BUILD_STATE_BEGUN;
  return PQP_OK;
}

int
PQP_Model::AddTri(const PQP_REAL *p1, 
                  const PQP_REAL *p2, 
                  const PQP_REAL *p3, 
                  int id)
{
  if (build_state == PQP_BUILD_STATE_EMPTY)
  {
    BeginModel();
  }
  else if (build_state == PQP_BUILD_STATE_PROCESSED)
  {
    fprintf(stderr,"PQP Warning! Called AddTri() on PQP_Model \n"
                   "object that was already ended. AddTri() was\n"
                   "ignored.  Must do a BeginModel() to clear the\n"
                   "model for addition of new triangles\n");
    return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
  }
        
  // allocate for new triangles

  if (num_tris >= num_tris_alloced)
  {
    Tri *temp;
    temp = new Tri[num_tris_alloced*2];
    if (!temp)
    {
      fprintf(stderr, "PQP Error!  Out of memory for tri array on"
	              " AddTri() call!\n");
      return PQP_ERR_MODEL_OUT_OF_MEMORY;  
    }
    memcpy(temp, tris, sizeof(Tri)*num_tris);
    delete [] tris;
    tris = temp;
    num_tris_alloced = num_tris_alloced*2;
  }
  
  // initialize the new triangle

  tris[num_tris].p1[0] = p1[0];
  tris[num_tris].p1[1] = p1[1];
  tris[num_tris].p1[2] = p1[2];

  tris[num_tris].p2[0] = p2[0];
  tris[num_tris].p2[1] = p2[1];
  tris[num_tris].p2[2] = p2[2];

  tris[num_tris].p3[0] = p3[0];
  tris[num_tris].p3[1] = p3[1];
  tris[num_tris].p3[2] = p3[2];

  tris[num_tris].id = id;

  num_tris += 1;

  return PQP_OK;
}

int
PQP_Model::EndModel()
{
  if (build_state == PQP_BUILD_STATE_PROCESSED)
  {
    fprintf(stderr,"PQP Warning! Called EndModel() on PQP_Model \n"
                   "object that was already ended. EndModel() was\n"
                   "ignored.  Must do a BeginModel() to clear the\n"
                   "model for addition of new triangles\n");
    return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  // report error is no tris

  if (num_tris == 0)
  {
    fprintf(stderr,"PQP Error! EndModel() called on model with"
                   " no triangles\n");
    return PQP_ERR_BUILD_EMPTY_MODEL;
  }

  // shrink fit tris array 

  if (num_tris_alloced > num_tris)
  {
    Tri *new_tris = new Tri[num_tris];
    if (!new_tris) 
    {
      fprintf(stderr, "PQP Error!  Out of memory for tri array "
                      "in EndModel() call!\n");
      return PQP_ERR_MODEL_OUT_OF_MEMORY;  
    }
    memcpy(new_tris, tris, sizeof(Tri)*num_tris);
    delete [] tris;
    tris = new_tris;
    num_tris_alloced = num_tris;
  }

  // create an array of BVs for the model

  b = new BV[2*num_tris - 1];
  if (!b)
  {
    fprintf(stderr,"PQP Error! out of memory for BV array "
                   "in EndModel()\n");
    return PQP_ERR_MODEL_OUT_OF_MEMORY;
  }
  num_bvs_alloced = 2*num_tris - 1;
  num_bvs = 0;

  // we should build the model now.

  build_model(this);
  build_state = PQP_BUILD_STATE_PROCESSED;

  return PQP_OK;
}

int
PQP_Model::MemUsage(int msg) const
{
  int mem_bv_list = sizeof(BV)*num_bvs;
  int mem_tri_list = sizeof(Tri)*num_tris;

  int total_mem = mem_bv_list + mem_tri_list + sizeof(PQP_Model);

  if (msg) 
  {
    fprintf(stderr,"Total for model %x: %d bytes\n", this, total_mem);
    fprintf(stderr,"BVs: %d alloced, take %d bytes each\n", 
            num_bvs, sizeof(BV));
    fprintf(stderr,"Tris: %d alloced, take %d bytes each\n", 
            num_tris, sizeof(Tri));
  }
  
  return total_mem;
}

//  COLLIDE STUFF
//
//--------------------------------------------------------------------------

PQP_CollideResult::PQP_CollideResult()
{
  pairs = 0;
  num_pairs = num_pairs_alloced = 0;
  num_bv_tests = 0;
  num_tri_tests = 0;
}

PQP_CollideResult::~PQP_CollideResult()
{
  delete [] pairs;
}

void
PQP_CollideResult::FreePairsList()
{
  num_pairs = num_pairs_alloced = 0;
  delete [] pairs;
  pairs = 0;
}

// may increase OR reduce mem usage
void
PQP_CollideResult::SizeTo(int n)
{
  CollisionPair *temp;

  if (n < num_pairs) 
  {
    fprintf(stderr, "PQP Error: Internal error in "
                    "'PQP_CollideResult::SizeTo(int n)'\n");
    fprintf(stderr, "       n = %d, but num_pairs = %d\n", n, num_pairs);
    return;
  }
  
  temp = new CollisionPair[n];
  memcpy(temp, pairs, num_pairs*sizeof(CollisionPair));
  delete [] pairs;
  pairs = temp;
  num_pairs_alloced = n;
  return;
}

void
PQP_CollideResult::Add(int a, int b)
{
  if (num_pairs >= num_pairs_alloced) 
  {
    // allocate more

    SizeTo(num_pairs_alloced*2+8);
  }

  // now proceed as usual

  pairs[num_pairs].id1 = a;
  pairs[num_pairs].id2 = b;
  num_pairs++;
}

// TRIANGLE OVERLAP TEST
       
inline
PQP_REAL
max(PQP_REAL a, PQP_REAL b, PQP_REAL c)
{
  PQP_REAL t = a;
  if (b > t) t = b;
  if (c > t) t = c;
  return t;
}

inline
PQP_REAL
min(PQP_REAL a, PQP_REAL b, PQP_REAL c)
{
  PQP_REAL t = a;
  if (b < t) t = b;
  if (c < t) t = c;
  return t;
}

int
project6(PQP_REAL *ax, 
         PQP_REAL *p1, PQP_REAL *p2, PQP_REAL *p3, 
         PQP_REAL *q1, PQP_REAL *q2, PQP_REAL *q3)
{
  PQP_REAL P1 = VdotV(ax, p1);
  PQP_REAL P2 = VdotV(ax, p2);
  PQP_REAL P3 = VdotV(ax, p3);
  PQP_REAL Q1 = VdotV(ax, q1);
  PQP_REAL Q2 = VdotV(ax, q2);
  PQP_REAL Q3 = VdotV(ax, q3);
  
  PQP_REAL mx1 = max(P1, P2, P3);
  PQP_REAL mn1 = min(P1, P2, P3);
  PQP_REAL mx2 = max(Q1, Q2, Q3);
  PQP_REAL mn2 = min(Q1, Q2, Q3);

  if (mn1 > mx2) return 0;
  if (mn2 > mx1) return 0;
  return 1;
}

// very robust triangle intersection test
// uses no divisions
// works on coplanar triangles
int 
TriContact(const PQP_REAL *P1, const PQP_REAL *P2, const PQP_REAL *P3,
           const PQP_REAL *Q1, const PQP_REAL *Q2, const PQP_REAL *Q3) 
{

  // One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
  // Edges are (e1,e2,e3) and (f1,f2,f3).
  // Normals are n1 and m1
  // Outwards are (g1,g2,g3) and (h1,h2,h3).
  //  
  // We assume that the triangle vertices are in the same coordinate system.
  //
  // First thing we do is establish a new c.s. so that p1 is at (0,0,0).

  PQP_REAL p1[3], p2[3], p3[3];
  PQP_REAL q1[3], q2[3], q3[3];
  PQP_REAL e1[3], e2[3], e3[3];
  PQP_REAL f1[3], f2[3], f3[3];
  PQP_REAL g1[3], g2[3], g3[3];
  PQP_REAL h1[3], h2[3], h3[3];
  PQP_REAL n1[3], m1[3];

  PQP_REAL ef11[3], ef12[3], ef13[3];
  PQP_REAL ef21[3], ef22[3], ef23[3];
  PQP_REAL ef31[3], ef32[3], ef33[3];
  
  p1[0] = P1[0] - P1[0];  p1[1] = P1[1] - P1[1];  p1[2] = P1[2] - P1[2];
  p2[0] = P2[0] - P1[0];  p2[1] = P2[1] - P1[1];  p2[2] = P2[2] - P1[2];
  p3[0] = P3[0] - P1[0];  p3[1] = P3[1] - P1[1];  p3[2] = P3[2] - P1[2];
  
  q1[0] = Q1[0] - P1[0];  q1[1] = Q1[1] - P1[1];  q1[2] = Q1[2] - P1[2];
  q2[0] = Q2[0] - P1[0];  q2[1] = Q2[1] - P1[1];  q2[2] = Q2[2] - P1[2];
  q3[0] = Q3[0] - P1[0];  q3[1] = Q3[1] - P1[1];  q3[2] = Q3[2] - P1[2];
  
  e1[0] = p2[0] - p1[0];  e1[1] = p2[1] - p1[1];  e1[2] = p2[2] - p1[2];
  e2[0] = p3[0] - p2[0];  e2[1] = p3[1] - p2[1];  e2[2] = p3[2] - p2[2];
  e3[0] = p1[0] - p3[0];  e3[1] = p1[1] - p3[1];  e3[2] = p1[2] - p3[2];

  f1[0] = q2[0] - q1[0];  f1[1] = q2[1] - q1[1];  f1[2] = q2[2] - q1[2];
  f2[0] = q3[0] - q2[0];  f2[1] = q3[1] - q2[1];  f2[2] = q3[2] - q2[2];
  f3[0] = q1[0] - q3[0];  f3[1] = q1[1] - q3[1];  f3[2] = q1[2] - q3[2];
  
  VcrossV(n1, e1, e2);
  VcrossV(m1, f1, f2);

  VcrossV(g1, e1, n1);
  VcrossV(g2, e2, n1);
  VcrossV(g3, e3, n1);
  VcrossV(h1, f1, m1);
  VcrossV(h2, f2, m1);
  VcrossV(h3, f3, m1);

  VcrossV(ef11, e1, f1);
  VcrossV(ef12, e1, f2);
  VcrossV(ef13, e1, f3);
  VcrossV(ef21, e2, f1);
  VcrossV(ef22, e2, f2);
  VcrossV(ef23, e2, f3);
  VcrossV(ef31, e3, f1);
  VcrossV(ef32, e3, f2);
  VcrossV(ef33, e3, f3);
  
  // now begin the series of tests

  if (!project6(n1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(m1, p1, p2, p3, q1, q2, q3)) return 0;
  
  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return 0;

  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return 0;

  return 1;
}

inline
PQP_REAL
TriDistance(PQP_REAL R[3][3], PQP_REAL T[3], const Tri *t1, const Tri *t2,
            PQP_REAL p[3], PQP_REAL q[3])
{
  // transform tri 2 into same space as tri 1

  PQP_REAL tri1[3][3], tri2[3][3];

  VcV(tri1[0], t1->p1);
  VcV(tri1[1], t1->p2);
  VcV(tri1[2], t1->p3);
  MxVpV(tri2[0], R, t2->p1, T);
  MxVpV(tri2[1], R, t2->p2, T);
  MxVpV(tri2[2], R, t2->p3, T);
                                
  return TriDist(p,q,tri1,tri2);
}


void
CollideRecurse(PQP_CollideResult *res,
               const PQP_Model *o1, int b1, 
               const PQP_Model *o2, int b2, int flag)
{
  // first thing, see if we're overlapping

  res->num_bv_tests++;

  //original PQP had this here
  //if (!BV_Overlap2(res->R, res->T, o1->child(b1), o2->child(b2))) return;

  // if we are, see if we test triangles next

  int l1 = o1->child(b1)->Leaf();
  int l2 = o2->child(b2)->Leaf();

  if (l1 && l2) 
  {
    res->num_tri_tests++;

    // transform the points in b2 into space of b1, then compare

    Tri *t1 = &o1->tris[-o1->child(b1)->first_child - 1];
    Tri *t2 = &o2->tris[-o2->child(b2)->first_child - 1];
    PQP_REAL q1[3], q2[3], q3[3];
    PQP_REAL *p1 = t1->p1;
    PQP_REAL *p2 = t1->p2;
    PQP_REAL *p3 = t1->p3;    
    MxVpV(q1, res->R, t2->p1, res->T);
    MxVpV(q2, res->R, t2->p2, res->T);
    MxVpV(q3, res->R, t2->p3, res->T);
    if (TriContact(p1, p2, p3, q1, q2, q3)) 
    {
      // add this to result

      res->Add(t1->id, t2->id);
    }
    return;
  }

  //moving this out if both bbs are leafs
  if (!BV_Overlap2(res->R, res->T, o1->child(b1), o2->child(b2))) return;

  // we dont, so decide whose children to visit next

  PQP_REAL sz1 = o1->child(b1)->GetSize();
  PQP_REAL sz2 = o2->child(b2)->GetSize();

  PQP_REAL Rc[3][3],Tc[3],Ttemp[3];
    
  if (l2 || (!l1 && (sz1 > sz2)))
  {
    int c1 = o1->child(b1)->first_child;
    int c2 = c1 + 1;

    CollideRecurse(res,o1,c1,o2,b2,flag);

    if ((flag == PQP_FIRST_CONTACT) && (res->num_pairs > 0)) return;

    CollideRecurse(res,o1,c2,o2,b2,flag);
  }
  else 
  {
    int c1 = o2->child(b2)->first_child;
    int c2 = c1 + 1;

    CollideRecurse(res,o1,b1,o2,c1,flag);

    if ((flag == PQP_FIRST_CONTACT) && (res->num_pairs > 0)) return;

    CollideRecurse(res,o1,b1,o2,c2,flag);
  }
}

int 
PQP_Collide(PQP_CollideResult *res,
            PQP_REAL R1[3][3], PQP_REAL T1[3], const PQP_Model *o1,
            PQP_REAL R2[3][3], PQP_REAL T2[3], const PQP_Model *o2,
            int flag)
{
  // make sure that the models are built

  if (o1->build_state != PQP_BUILD_STATE_PROCESSED) 
    return PQP_ERR_UNPROCESSED_MODEL;
  if (o2->build_state != PQP_BUILD_STATE_PROCESSED) 
    return PQP_ERR_UNPROCESSED_MODEL;

  // clear the stats

  res->num_bv_tests = 0;
  res->num_tri_tests = 0;
  
  // don't release the memory, but reset the num_pairs counter

  res->num_pairs = 0;
  
  // Okay, compute what transform [R,T] that takes us from cs1 to cs2.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  // First compute the rotation part, then translation part

  MTxM(res->R,R1,R2);
  PQP_REAL Ttemp[3];
  VmV(Ttemp, T2, T1);  
  MTxV(res->T, R1, Ttemp);
  
  // now start with both top level BVs  

  CollideRecurse(res,o1,0,o2,0,flag);
  
  return PQP_OK; 
}

#if PQP_BV_TYPE & RSS_TYPE // distance/tolerance only available with RSS
                           // unless an OBB distance test is supplied in 
                           // BV.cpp

// DISTANCE STUFF
//
//--------------------------------------------------------------------------

void
DistanceRecurse(PQP_DistanceResult *res,
                const PQP_Model *o1, int b1,
                const PQP_Model *o2, int b2)
{
  PQP_REAL sz1 = o1->child(b1)->GetSize();
  PQP_REAL sz2 = o2->child(b2)->GetSize();
  int l1 = o1->child(b1)->Leaf();
  int l2 = o2->child(b2)->Leaf();

  if (l1 && l2)
  {
    // both leaves.  Test the triangles beneath them.

    res->num_tri_tests++;

    PQP_REAL p[3], q[3];

    Tri *t1 = &o1->tris[-o1->child(b1)->first_child - 1];
    Tri *t2 = &o2->tris[-o2->child(b2)->first_child - 1];

    PQP_REAL d = TriDistance(res->R,res->T,t1,t2,p,q);
  
    if (d < res->distance) 
    {
      res->distance = d;

      VcV(res->p1, p);         // p already in c.s. 1
      VcV(res->p2, q);         // q must be transformed 
                               // into c.s. 2 later
      res->t1 = -o1->child(b1)->first_child - 1;
      res->t2 = -o2->child(b2)->first_child - 1;
    }

    return;
  }

  // First, perform distance tests on the children. Then traverse 
  // them recursively, but test the closer pair first, the further 
  // pair second.

  int a1,a2,c1,c2;  // new bv tests 'a' and 'c'
  PQP_REAL R1[3][3], T1[3], R2[3][3], T2[3], Ttemp[3];

  if (l2 || (!l1 && (sz1 > sz2)))
  {
    // visit the children of b1

    a1 = o1->child(b1)->first_child;
    a2 = b2;
    c1 = o1->child(b1)->first_child+1;
    c2 = b2;
  }
  else 
  {
    // visit the children of b2

    a1 = b1;
    a2 = o2->child(b2)->first_child;
    c1 = b1;
    c2 = o2->child(b2)->first_child+1;

  }

  res->num_bv_tests += 2;

  PQP_REAL d1 = BV_Distance2(res->R, res->T, o1->child(a1), o2->child(a2));
  PQP_REAL d2 = BV_Distance2(res->R, res->T, o1->child(c1), o2->child(c2));

  if (d2 < d1)
  {
    if ((d2 < (res->distance - res->abs_err)) || 
        (d2*(1 + res->rel_err) < res->distance)) 
    {      
      DistanceRecurse(res, o1, c1, o2, c2);      
    }

    if ((d1 < (res->distance - res->abs_err)) || 
        (d1*(1 + res->rel_err) < res->distance)) 
    {      
      DistanceRecurse(res, o1, a1, o2, a2);
    }
  }
  else 
  {
    if ((d1 < (res->distance - res->abs_err)) || 
        (d1*(1 + res->rel_err) < res->distance)) 
    {      
      DistanceRecurse(res, o1, a1, o2, a2);
    }

    if ((d2 < (res->distance - res->abs_err)) || 
        (d2*(1 + res->rel_err) < res->distance)) 
    {      
      DistanceRecurse(res, o1, c1, o2, c2);      
    }
  }
}

void
DistanceQueueRecurse(PQP_DistanceResult *res, 
                     const PQP_Model *o1, int b1,
                     const PQP_Model *o2, int b2)
{
  BVTQ bvtq(res->qsize);

  BVT min_test;
  min_test.b1 = b1;
  min_test.b2 = b2;

  while(1) 
  {  
    int l1 = o1->child(min_test.b1)->Leaf();
    int l2 = o2->child(min_test.b2)->Leaf();
    
    if (l1 && l2) 
    {  
      // both leaves.  Test the triangles beneath them.

      res->num_tri_tests++;

      PQP_REAL p[3], q[3];

      Tri *t1 = &o1->tris[-o1->child(min_test.b1)->first_child - 1];
      Tri *t2 = &o2->tris[-o2->child(min_test.b2)->first_child - 1];

      PQP_REAL d = TriDistance(res->R,res->T,t1,t2,p,q);
  
      if (d < res->distance)
      {
        res->distance = d;

        VcV(res->p1, p);         // p already in c.s. 1
        VcV(res->p2, q);         // q must be transformed 
                                 // into c.s. 2 later
	res->t1 = -o1->child(b1)->first_child - 1;
	res->t2 = -o2->child(b2)->first_child - 1;
      }
    }		 
    else if (bvtq.GetNumTests() == bvtq.GetSize() - 1) 
    {  
      // queue can't get two more tests, recur
      
      DistanceQueueRecurse(res,o1,min_test.b1,o2,min_test.b2);
    }
    else 
    {  
      // decide how to descend to children
      
      PQP_REAL sz1 = o1->child(min_test.b1)->GetSize();
      PQP_REAL sz2 = o2->child(min_test.b2)->GetSize();

      res->num_bv_tests += 2;
 
      BVT bvt1,bvt2;
      PQP_REAL Ttemp[3];

      if (l2 || (!l1 && (sz1 > sz2)))	
      {  
        // put new tests on queue consisting of min_test.b2 
        // with children of min_test.b1 
      
        int c1 = o1->child(min_test.b1)->first_child;
        int c2 = c1 + 1;

        // init bv test 1

        bvt1.b1 = c1;
        bvt1.b2 = min_test.b2;
        bvt1.d = BV_Distance2(res->R,res->T,
                            o1->child(bvt1.b1),o2->child(bvt1.b2));

        // init bv test 2

        bvt2.b1 = c2;
        bvt2.b2 = min_test.b2;
        bvt2.d = BV_Distance2(res->R,res->T,
                            o1->child(bvt2.b1),o2->child(bvt2.b2));
      }
      else 
      {
        // put new tests on queue consisting of min_test.b1 
        // with children of min_test.b2
      
        int c1 = o2->child(min_test.b2)->first_child;
        int c2 = c1 + 1;

        // init bv test 1

        bvt1.b1 = min_test.b1;
        bvt1.b2 = c1;
        bvt1.d = BV_Distance2(res->R,res->T,
                            o1->child(bvt1.b1),o2->child(bvt1.b2));

        // init bv test 2

        bvt2.b1 = min_test.b1;
        bvt2.b2 = c2;
        bvt2.d = BV_Distance2(res->R,res->T,
                            o1->child(bvt2.b1),o2->child(bvt2.b2));
      }

      bvtq.AddTest(bvt1);	
      bvtq.AddTest(bvt2);
    }

    if (bvtq.Empty())
    {
      break;
    }
    else
    {
      min_test = bvtq.ExtractMinTest();

      if ((min_test.d + res->abs_err >= res->distance) && 
         ((min_test.d * (1 + res->rel_err)) >= res->distance)) 
      {
        break;
      }
    }
  }  
}	

int 
PQP_Distance(PQP_DistanceResult *res,
             PQP_REAL R1[3][3], PQP_REAL T1[3], const PQP_Model *o1,
             PQP_REAL R2[3][3], PQP_REAL T2[3], const PQP_Model *o2,
             PQP_REAL rel_err, PQP_REAL abs_err,
             int qsize,PQP_REAL init_bound)
{
  
  
  // make sure that the models are built

  if (o1->build_state != PQP_BUILD_STATE_PROCESSED) 
    return PQP_ERR_UNPROCESSED_MODEL;
  if (o2->build_state != PQP_BUILD_STATE_PROCESSED) 
    return PQP_ERR_UNPROCESSED_MODEL;

  // Okay, compute what transform [R,T] that takes us from cs2 to cs1.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  // First compute the rotation part, then translation part

  MTxM(res->R,R1,R2);
  PQP_REAL Ttemp[3];
  VmV(Ttemp, T2, T1);  
  MTxV(res->T, R1, Ttemp);
  
  // establish initial upper bound using last triangles which 
  // provided the minimum distance

  if(init_bound < 0) {
    if(res->t1 < 0) res->t1 = 0;
    if(res->t2 < 0) res->t2 = 0;
    if(res->t1 >= o1->num_tris) res->t1 = 0;
    if(res->t2 >= o2->num_tris) res->t2 = 0;
    PQP_REAL p[3],q[3];
    res->distance = TriDistance(res->R,res->T,&o1->tris[res->t1],&o2->tris[res->t2],p,q);
    VcV(res->p1,p);
    VcV(res->p2,q);
    /*
    assert(o1->num_tris < 100000);
    assert(o2->num_tris < 100000);
    assert(o1->num_tris > res->t1);
    assert(o2->num_tris > res->t2);
    assert(res->t1 >= 0);
    assert(res->t2 >= 0);
    if(res->distance == 0) {
      printf("Init distance %g\n",res->distance);
      printf("P = %g %g %g\n",p[0],p[1],p[2]);
      printf("Q = %g %g %g\n",q[0],q[1],q[2]);
      printf("T1.a = %g %g %g\n",o1->tris[res->t1].p1[0],o1->tris[res->t1].p1[1],o1->tris[res->t1].p1[2]);
      printf("T1.b = %g %g %g\n",o1->tris[res->t1].p2[0],o1->tris[res->t1].p2[1],o1->tris[res->t1].p2[2]);
      printf("T1.c = %g %g %g\n",o1->tris[res->t1].p3[0],o1->tris[res->t1].p3[1],o1->tris[res->t1].p3[2]);
      printf("T2.a = %g %g %g\n",o2->tris[res->t2].p1[0],o2->tris[res->t2].p1[1],o2->tris[res->t2].p1[2]);
      printf("T2.b = %g %g %g\n",o2->tris[res->t2].p2[0],o2->tris[res->t2].p2[1],o2->tris[res->t2].p2[2]);
      printf("T2.c = %g %g %g\n",o2->tris[res->t2].p3[0],o2->tris[res->t2].p3[1],o2->tris[res->t2].p3[2]);
      printf("Translation %g %g %g\n",res->T[0],res->T[1],res->T[2]);
    }
    */
  }
  else if(init_bound == 0.0) {
    res->distance = 0;
    return PQP_OK;
  }
  else {
    res->distance = init_bound;
  }

  // initialize error bounds

  res->abs_err = abs_err;
  res->rel_err = rel_err;
  
  // clear the stats

  res->num_bv_tests = 0;
  res->num_tri_tests = 0;
  
    // choose routine according to queue size
  
  if (qsize <= 2)
  {
    DistanceRecurse(res,o1,0,o2,0);    
  }
  else 
  { 
    res->qsize = qsize;

    DistanceQueueRecurse(res,o1,0,o2,0);
  }

  if(res->distance != init_bound) 
  {
  // res->p2 is in cs 1 ; transform it to cs 2
    PQP_REAL u[3];
    VmV(u, res->p2, res->T);
    MTxV(res->p2, res->R, u);
  }
  return PQP_OK;
}

// Tolerance Stuff
//
//---------------------------------------------------------------------------
void 
ToleranceRecurse(PQP_ToleranceResult *res, 
                 const PQP_Model *o1, int b1, const PQP_Model *o2, int b2)
{
  PQP_REAL sz1 = o1->child(b1)->GetSize();
  PQP_REAL sz2 = o2->child(b2)->GetSize();
  int l1 = o1->child(b1)->Leaf();
  int l2 = o2->child(b2)->Leaf();

  if (l1 && l2) 
  {
    // both leaves - find if tri pair within tolerance
    
    res->num_tri_tests++;

    PQP_REAL p[3], q[3];

    Tri *t1 = &o1->tris[-o1->child(b1)->first_child - 1];
    Tri *t2 = &o2->tris[-o2->child(b2)->first_child - 1];

    PQP_REAL d = TriDistance(res->R,res->T,t1,t2,p,q);
    
    if (d <= res->tolerance)  
    {  
      // triangle pair distance less than tolerance

      res->closer_than_tolerance = 1;
      res->distance = d;
      VcV(res->p1, p);         // p already in c.s. 1
      VcV(res->p2, q);         // q must be transformed 
                               // into c.s. 2 later

      res->t1 = -o1->child(b1)->first_child - 1;
      res->t2 = -o2->child(b2)->first_child - 1;
    }

    return;
  }

  int a1,a2,c1,c2;  // new bv tests 'a' and 'c'

  if (l2 || (!l1 && (sz1 > sz2)))
  {
    // visit the children of b1

    a1 = o1->child(b1)->first_child;
    a2 = b2;
    c1 = o1->child(b1)->first_child+1;
    c2 = b2;
  }
  else 
  {
    // visit the children of b2

    a1 = b1;
    a2 = o2->child(b2)->first_child;
    c1 = b1;
    c2 = o2->child(b2)->first_child+1;
  }

  res->num_bv_tests += 2;

  PQP_REAL d1 = BV_Distance2(res->R, res->T, o1->child(a1), o2->child(a2));
  PQP_REAL d2 = BV_Distance2(res->R, res->T, o1->child(c1), o2->child(c2));

  if (d2 < d1) 
  {
    if (d2 <= res->tolerance) ToleranceRecurse(res, o1, c1, o2, c2);
    if (res->closer_than_tolerance) return;
    if (d1 <= res->tolerance) ToleranceRecurse(res, o1, a1, o2, a2);
  }
  else 
  {
    if (d1 <= res->tolerance) ToleranceRecurse(res, o1, a1, o2, a2);
    if (res->closer_than_tolerance) return;
    if (d2 <= res->tolerance) ToleranceRecurse(res, o1, c1, o2, c2);
  }
}

void
ToleranceQueueRecurse(PQP_ToleranceResult *res,
                      const PQP_Model *o1, int b1,
                      const PQP_Model *o2, int b2)
{
  BVTQ bvtq(res->qsize);
  BVT min_test;
  min_test.b1 = b1;
  min_test.b2 = b2;

  while(1)
  {  
    int l1 = o1->child(min_test.b1)->Leaf();
    int l2 = o2->child(min_test.b2)->Leaf();
    
    if (l1 && l2) 
    {  
      // both leaves - find if tri pair within tolerance
    
      res->num_tri_tests++;

      PQP_REAL p[3], q[3];

      Tri *t1 = &o1->tris[-o1->child(min_test.b1)->first_child - 1];
      Tri *t2 = &o2->tris[-o2->child(min_test.b2)->first_child - 1];

      PQP_REAL d = TriDistance(res->R,res->T,t1,t2,p,q);
    
      if (d <= res->tolerance)  
      {  
        // triangle pair distance less than tolerance

        res->closer_than_tolerance = 1;
        res->distance = d;
        VcV(res->p1, p);         // p already in c.s. 1
        VcV(res->p2, q);         // q must be transformed 
                                 // into c.s. 2 later
	res->t1 = -o1->child(b1)->first_child - 1;
	res->t2 = -o2->child(b2)->first_child - 1;
        return;
      }
    }
    else if (bvtq.GetNumTests() == bvtq.GetSize() - 1)
    {  
      // queue can't get two more tests, recur
      
      ToleranceQueueRecurse(res,
                            o1,min_test.b1,o2,min_test.b2);
      if (res->closer_than_tolerance == 1) return;
    }
    else 
    {  
      // decide how to descend to children
      
      PQP_REAL sz1 = o1->child(min_test.b1)->GetSize();
      PQP_REAL sz2 = o2->child(min_test.b2)->GetSize();

      res->num_bv_tests += 2;
      
      BVT bvt1,bvt2;
      PQP_REAL Ttemp[3];

      if (l2 || (!l1 && (sz1 > sz2)))	
      {
	      // add two new tests to queue, consisting of min_test.b2
        // with the children of min_test.b1

        int c1 = o1->child(min_test.b1)->first_child;
        int c2 = c1 + 1;

        // init bv test 1

        bvt1.b1 = c1;
        bvt1.b2 = min_test.b2;
        bvt1.d = BV_Distance2(res->R,res->T,
                            o1->child(bvt1.b1),o2->child(bvt1.b2));

	      // init bv test 2

	bvt2.b1 = c2;
	bvt2.b2 = min_test.b2;
        bvt2.d = BV_Distance2(res->R,res->T,
                            o1->child(bvt2.b1),o2->child(bvt2.b2));
      }
      else 
      {
        // add two new tests to queue, consisting of min_test.b1
        // with the children of min_test.b2

        int c1 = o2->child(min_test.b2)->first_child;
        int c2 = c1 + 1;

        // init bv test 1

        bvt1.b1 = min_test.b1;
        bvt1.b2 = c1;
        bvt1.d = BV_Distance2(res->R,res->T,
                            o1->child(bvt1.b1),o2->child(bvt1.b2));

        // init bv test 2

        bvt2.b1 = min_test.b1;
        bvt2.b2 = c2;
        bvt2.d = BV_Distance2(res->R,res->T,
                            o1->child(bvt2.b1),o2->child(bvt2.b2));
      }

      // put children tests in queue

      if (bvt1.d <= res->tolerance) bvtq.AddTest(bvt1);
      if (bvt2.d <= res->tolerance) bvtq.AddTest(bvt2);
    }

    if (bvtq.Empty() || (bvtq.MinTest() > res->tolerance)) 
    {
      res->closer_than_tolerance = 0;
      return;
    }
    else 
    {
      min_test = bvtq.ExtractMinTest();
    }
  }  
}	

int
PQP_Tolerance(PQP_ToleranceResult *res,
              PQP_REAL R1[3][3], PQP_REAL T1[3], const PQP_Model *o1,
              PQP_REAL R2[3][3], PQP_REAL T2[3], const PQP_Model *o2,
              PQP_REAL tolerance,
              int qsize)
{
  // make sure that the models are built

  if (o1->build_state != PQP_BUILD_STATE_PROCESSED) 
    return PQP_ERR_UNPROCESSED_MODEL;
  if (o2->build_state != PQP_BUILD_STATE_PROCESSED) 
    return PQP_ERR_UNPROCESSED_MODEL;
  
  // Compute the transform [R,T] that takes us from cs2 to cs1.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]

  MTxM(res->R,R1,R2);
  PQP_REAL Ttemp[3];
  VmV(Ttemp, T2, T1);
  MTxV(res->T, R1, Ttemp);

  // set tolerance, used to prune the search

  if (tolerance < 0.0) tolerance = 0.0;
  res->tolerance = tolerance;
  
  // clear the stats

  res->num_bv_tests = 0;
  res->num_tri_tests = 0;

  // initially assume not closer than tolerance

  res->closer_than_tolerance = 0;
  
  // compute the transform from o1->child(0) to o2->child(0)

  // find a distance lower bound for trivial reject

  PQP_REAL d = BV_Distance2(res->R, res->T, o1->child(0), o2->child(0));
  
  if (d <= res->tolerance)
  {
    // more work needed - choose routine according to queue size

    if (qsize <= 2) 
    {
      ToleranceRecurse(res, o1, 0, o2, 0);
    }
    else 
    {
      res->qsize = qsize;
      ToleranceQueueRecurse(res, o1, 0, o2, 0);
    }
  }

  // res->p2 is in cs 1 ; transform it to cs 2

  PQP_REAL u[3];
  VmV(u, res->p2, res->T);
  MTxV(res->p2, res->R, u);

  return PQP_OK;
}




void 
ToleranceRecurse2(PQP_ToleranceResult *res, 
		  const PQP_Model *o1, int b1, const PQP_Model *o2, int b2, PQP_ToleranceAllResult& allRes)
{
  int l1 = o1->child(b1)->Leaf();
  int l2 = o2->child(b2)->Leaf();

  if (l1 && l2) 
  {
    // both leaves - find if tri pair within tolerance
    
    res->num_tri_tests++;

    PQP_REAL p[3], q[3];

    int ti1=-o1->child(b1)->first_child - 1;
    int ti2=-o2->child(b2)->first_child - 1;
    //printf("Testing leaves %d %d\n",ti1,ti2);
    Tri *t1 = &o1->tris[ti1];
    Tri *t2 = &o2->tris[ti2];

    PQP_REAL d = TriDistance(res->R,res->T,t1,t2,p,q);
    
    if (d <= res->tolerance)  
    {  
      // triangle pair distance less than tolerance
      //printf("Triangle pair %d %d within tolerance, distance %g\n",ti1,ti2,d);
      res->closer_than_tolerance = 1;
      if(d < res->distance) {
	res->distance = d;
	VcV(res->p1, p);         // p already in c.s. 1
	VcV(res->p2, q);         // q must be transformed 
                                 // into c.s. 2 later
      }
      //mark it in the list
      if(allRes.triDist1.count(t1->id)==0 || d < allRes.triDist1[t1->id]) {
	allRes.triDist1[t1->id] = d;
	allRes.triPartner1[t1->id]=t2->id;
	VcV(allRes.triCp1[t1->id].p1,p);
	VcV(allRes.triCp1[t1->id].p2,q);
      }
      if(allRes.triDist2.count(t2->id)==0 || d < allRes.triDist2[t2->id]) {
	allRes.triDist2[t2->id] = d;
	allRes.triPartner2[t2->id]=t1->id;
	VcV(allRes.triCp2[t2->id].p1,p);
	VcV(allRes.triCp2[t2->id].p2,q);
      }
    }
    return;
  }

  int a1,a2,c1,c2;  // new bv tests 'a' and 'c'
  PQP_REAL sz1 = o1->child(b1)->GetSize();
  PQP_REAL sz2 = o2->child(b2)->GetSize();

  PQP_REAL lim1=res->tolerance,lim2=res->tolerance;
  if(l2) {  //clip points of interest by existing per-triangle tolerance
    int ti2=-o2->child(b2)->first_child - 1;
    if(allRes.triDist2.count(o1->tris[ti2].id)!=0)
      lim1=allRes.triDist2[o1->tris[ti2].id];
  }
  if(l1) {  //clip points of interest by existing per-triangle tolerance
    int ti1=-o1->child(b1)->first_child - 1;
    if(allRes.triDist1.count(o1->tris[ti1].id)!=0)
      lim2=allRes.triDist1[o1->tris[ti1].id];
  }
  if (l2 || (!l1 && (sz1 > sz2)))
  {
    // visit the children of b1 
    a1 = o1->child(b1)->first_child;
    a2 = b2;
    c1 = o1->child(b1)->first_child+1;
    c2 = b2;
   }
  else 
  {
    // visit the children of b2
    a1 = b1;
    a2 = o2->child(b2)->first_child;
    c1 = b1;
    c2 = o2->child(b2)->first_child+1;
  }

  res->num_bv_tests += 2;

  PQP_REAL d1 = BV_Distance2(res->R,res->T, o1->child(a1), o2->child(a2));
  PQP_REAL d2 = BV_Distance2(res->R,res->T, o1->child(c1), o2->child(c2));

  if (d2 < d1) 
  {
    if (d2 <= lim2) ToleranceRecurse2(res, o1, c1, o2, c2, allRes);
    if (d1 <= lim1) ToleranceRecurse2(res, o1, a1, o2, a2, allRes);
  }
  else 
  {
    if (d1 <= lim1) ToleranceRecurse2(res, o1, a1, o2, a2, allRes);
    if (d2 <= lim2) ToleranceRecurse2(res, o1, c1, o2, c2, allRes);
  }
}


int
PQP_ToleranceAll(PQP_ToleranceResult* res,
	       PQP_REAL R1[3][3], PQP_REAL T1[3], const PQP_Model *o1,
	       PQP_REAL R2[3][3], PQP_REAL T2[3], const PQP_Model *o2,
	       PQP_REAL tolerance,PQP_ToleranceAllResult& allRes)
{
  // make sure that the models are built
  if (o1->build_state != 2) 
    return PQP_ERR_UNPROCESSED_MODEL;
  if (o2->build_state != 2) 
    return PQP_ERR_UNPROCESSED_MODEL;

  allRes.triDist1.clear();
  allRes.triPartner1.clear();
  allRes.triCp1.clear();
  allRes.triDist2.clear();
  allRes.triPartner2.clear();
  allRes.triCp2.clear();
    /*
    allRes.triDist1.resize(o1->num_tris);
    allRes.triPartner1.resize(o1->num_tris);
    allRes.triCp1.resize(o1->num_tris);
    allRes.triDist2.resize(o2->num_tris);
    allRes.triPartner2.resize(o2->num_tris);
    allRes.triCp2.resize(o2->num_tris);
    fill(allRes.triDist1.begin(),allRes.triDist1.end(),tolerance);
    fill(allRes.triDist2.begin(),allRes.triDist2.end(),tolerance);
    fill(allRes.triPartner1.begin(),allRes.triPartner1.end(),-1);
    fill(allRes.triPartner2.begin(),allRes.triPartner2.end(),-1);
    */

  // Compute the transform [R,T] that takes us from cs2 to cs1.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]

  MTxM(res->R,R1,R2);
  PQP_REAL Ttemp[3];
  VmV(Ttemp, T2, T1);
  MTxV(res->T, R1, Ttemp);

  // set tolerance, used to prune the search

  if (tolerance < 0.0) tolerance = 0.0;
  res->tolerance = tolerance;
  res->distance = tolerance;
  
  // clear the stats

  res->num_bv_tests = 0;
  res->num_tri_tests = 0;

  // initially assume not closer than tolerance

  res->closer_than_tolerance = 0;
  
  // find a distance lower bound for trivial reject

  res->num_bv_tests += 2;
  PQP_REAL d = BV_Distance2(res->R, res->T, o1->child(0), o2->child(0));
  
  if (d <= res->tolerance)
  {
    ToleranceRecurse2(res, o1, 0, o2, 0, allRes);

    for(std::map<int,PQP_ClosestPoints>::iterator i=allRes.triCp1.begin();i!=allRes.triCp1.end();i++) {
      // res->p2 is in cs 1 ; transform it to cs 2 
      PQP_REAL u[3];
      VmV(u, i->second.p2, res->T);
      MTxV(i->second.p2, res->R, u);
    }
    for(std::map<int,PQP_ClosestPoints >::iterator i=allRes.triCp2.begin();i!=allRes.triCp2.end();i++) {
      // res->p2 is in cs 1 ; transform it to cs 2 
      PQP_REAL u[3];
      VmV(u, i->second.p2, res->T);
      MTxV(i->second.p2, res->R, u);
    }
    /*
    for(size_t i=0;i<allRes.triCp1.size();i++) {
      if(allRes.triPartner1[i] >= 0) {
	// res->p2 is in cs 1 ; transform it to cs 2 
	PQP_REAL u[3];
	VmV(u, allRes.triCp1[i].second, res->T);
	MTxV(allRes.triCp1[i].second, res->R, u);
      }
    }
    for(size_t i=0;i<allRes.triCp2.size();i++) {
      if(allRes.triPartner2[i] >= 0) {
	// res->p2 is in cs 1 ; transform it to cs 2 
	PQP_REAL u[3];
	VmV(u, allRes.triCp2[i].second, res->T);
	MTxV(allRes.triCp2[i].second, res->R, u);
      }
    }
    */
  }

  // res->p2 is in cs 1 ; transform it to cs 2

  PQP_REAL u[3];
  VmV(u, res->p2, res->T);
  MTxV(res->p2, res->R, u);

  //printf("Res num bv tests %d, num tri tests %d\n",res->num_bv_tests,res->num_tri_tests);

  return PQP_OK;
}


#endif
