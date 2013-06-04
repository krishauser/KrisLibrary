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
#include <stdlib.h>
#include <string.h>
#include "PQP.h"
#include "MatVec.h"



PQP_REAL max(PQP_REAL a, PQP_REAL b, PQP_REAL c, PQP_REAL d)
{
  PQP_REAL t = a;
  if (b > t) t = b;
  if (c > t) t = c;
  if (d > t) t = d;
  return t;
}

PQP_REAL min(PQP_REAL a, PQP_REAL b, PQP_REAL c, PQP_REAL d)
{
  PQP_REAL t = a;
  if (b < t) t = b;
  if (c < t) t = c;
  if (d < t) t = d;
  return t;
}

void
get_centroid_triverts(PQP_REAL c[3], Tri *tris, int num_tris)
{
  int i;

  c[0] = c[1] = c[2] = 0.0;

  // get center of mass
  for(i=0; i<num_tris; i++)
  {
    PQP_REAL *p1 = tris[i].p1;
    PQP_REAL *p2 = tris[i].p2;
    PQP_REAL *p3 = tris[i].p3;

    c[0] += p1[0] + p2[0] + p3[0];
    c[1] += p1[1] + p2[1] + p3[1];
    c[2] += p1[2] + p2[2] + p3[2];      
  }

  PQP_REAL n = (PQP_REAL)(3 * num_tris);

  c[0] /= n;
  c[1] /= n;
  c[2] /= n;
}

void
get_covariance_triverts(PQP_REAL M[3][3], Tri *tris, int num_tris)
{
  int i;
  PQP_REAL S1[3];
  PQP_REAL S2[3][3];

  S1[0] = S1[1] = S1[2] = 0.0;
  S2[0][0] = S2[1][0] = S2[2][0] = 0.0;
  S2[0][1] = S2[1][1] = S2[2][1] = 0.0;
  S2[0][2] = S2[1][2] = S2[2][2] = 0.0;

  // get center of mass
  for(i=0; i<num_tris; i++)
  {
    PQP_REAL *p1 = tris[i].p1;
    PQP_REAL *p2 = tris[i].p2;
    PQP_REAL *p3 = tris[i].p3;

    S1[0] += p1[0] + p2[0] + p3[0];
    S1[1] += p1[1] + p2[1] + p3[1];
    S1[2] += p1[2] + p2[2] + p3[2];

    S2[0][0] += (p1[0] * p1[0] +  
                 p2[0] * p2[0] +  
                 p3[0] * p3[0]);
    S2[1][1] += (p1[1] * p1[1] +  
                 p2[1] * p2[1] +  
                 p3[1] * p3[1]);
    S2[2][2] += (p1[2] * p1[2] +  
                 p2[2] * p2[2] +  
                 p3[2] * p3[2]);
    S2[0][1] += (p1[0] * p1[1] +  
                 p2[0] * p2[1] +  
                 p3[0] * p3[1]);
    S2[0][2] += (p1[0] * p1[2] +  
                 p2[0] * p2[2] +  
                 p3[0] * p3[2]);
    S2[1][2] += (p1[1] * p1[2] +  
                 p2[1] * p2[2] +  
                 p3[1] * p3[2]);
  }

  PQP_REAL n = (PQP_REAL)(3 * num_tris);

  // now get covariances

  M[0][0] = S2[0][0] - S1[0]*S1[0] / n;
  M[1][1] = S2[1][1] - S1[1]*S1[1] / n;
  M[2][2] = S2[2][2] - S1[2]*S1[2] / n;
  M[0][1] = S2[0][1] - S1[0]*S1[1] / n;
  M[1][2] = S2[1][2] - S1[1]*S1[2] / n;
  M[0][2] = S2[0][2] - S1[0]*S1[2] / n;
  M[1][0] = M[0][1];
  M[2][0] = M[0][2];
  M[2][1] = M[1][2];
}


// given a list of triangles, a splitting axis, and a coordinate on
// that axis, partition the triangles into two groups according to
// where their centroids fall on the axis (under axial projection).
// Returns the number of tris in the first half

int 
split_tris(Tri *tris, int num_tris, PQP_REAL a[3], PQP_REAL c)
{
  int i;
  int c1 = 0;
  PQP_REAL p[3];
  PQP_REAL x;
  Tri temp;

  for(i = 0; i < num_tris; i++)
  {
    // loop invariant: up to (but not including) index c1 in group 1,
    // then up to (but not including) index i in group 2
    //
    //  [1] [1] [1] [1] [2] [2] [2] [x] [x] ... [x]
    //                   c1          i
    //
    VcV(p, tris[i].p1);
    VpV(p, p, tris[i].p2);
    VpV(p, p, tris[i].p3);      
    x = VdotV(p, a);
    x /= 3.0;
    if (x <= c)
    {
	    // group 1
	    temp = tris[i];
	    tris[i] = tris[c1];
	    tris[c1] = temp;
	    c1++;
    }
    else
    {
	    // group 2 -- do nothing
    }
  }

  // split arbitrarily if one group empty

  if ((c1 == 0) || (c1 == num_tris)) c1 = num_tris/2;

  return c1;
}

// Fits m->child(bn) to the num_tris triangles starting at first_tri
// Then, if num_tris is greater than one, partitions the tris into two
// sets, and recursively builds two children of m->child(bn)

int
build_recurse(PQP_Model *m, int bn, int first_tri, int num_tris)
{
  BV *b = m->child(bn);

  // compute a rotation matrix

  PQP_REAL C[3][3], E[3][3], R[3][3], s[3], axis[3], mean[3], coord;

  get_covariance_triverts(C,&m->tris[first_tri],num_tris);

  Meigen(E, s, C);

  // place axes of E in order of increasing s

  int min, mid, max;
  if (s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if (s[2] < s[min]) { mid = min; min = 2; }
  else if (s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }
  McolcMcol(R,0,E,max);
  McolcMcol(R,1,E,mid);
  R[0][2] = E[1][max]*E[2][mid] - E[1][mid]*E[2][max];
  R[1][2] = E[0][mid]*E[2][max] - E[0][max]*E[2][mid];
  R[2][2] = E[0][max]*E[1][mid] - E[0][mid]*E[1][max];

  // fit the BV

  b->FitToTris(R, &m->tris[first_tri], num_tris);

  if (num_tris == 1)
  {
    // BV is a leaf BV - first_child will index a triangle

    b->first_child = -(first_tri + 1);
  }
  else if (num_tris > 1)
  {
    // BV not a leaf - first_child will index a BV

    b->first_child = m->num_bvs;
    m->num_bvs+=2;

    // choose splitting axis and splitting coord

    McolcV(axis,R,0);

    get_centroid_triverts(mean,&m->tris[first_tri],num_tris);
    coord = VdotV(axis, mean);

    // now split

    int num_first_half = split_tris(&m->tris[first_tri], num_tris, 
                                    axis, coord);

    // recursively build the children

    build_recurse(m, m->child(bn)->first_child, first_tri, num_first_half); 
    build_recurse(m, m->child(bn)->first_child + 1,
                  first_tri + num_first_half, num_tris - num_first_half); 
  }
  return PQP_OK;
}


int
build_model(PQP_Model *m)
{
  // set num_bvs to 1, the first index for a child bv

  m->num_bvs = 1;

  // build recursively

  build_recurse(m, 0, 0, m->num_tris);

  return PQP_OK;
}
