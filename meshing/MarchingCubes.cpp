/*
 * Based on the algorithm & look-up tables by Paul Bourke        [http://astronomy.swin.edu.au/~pbourke/modelling/polygonise/]
 */

#include <KrisLibrary/Logger.h>
#include "MarchingCubes.h"
#include <math3d/interpolate.h>
#include <math/function.h>

namespace Meshing {

#define DEBUG 0

extern const int MCEdgeTable[256] = {
	0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
	0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
	0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
	0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
	0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
	0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
	0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
	0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
	0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
	0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
	0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
	0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
	0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
	0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
	0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
	0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
	0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
	0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
	0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
	0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
	0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
	0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
	0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
	0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
	0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
	0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
	0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
	0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
	0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
	0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
	0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
	0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
};

extern const int MCTriTable[256][16] = {
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
	{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
	{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
	{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
	{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
	{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
	{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
	{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
	{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
	{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
	{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
	{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
	{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
	{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
	{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
	{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
	{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
	{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
	{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
	{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
	{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
	{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
	{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
	{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
	{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
	{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
	{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
	{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
	{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
	{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
	{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
	{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
	{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
	{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
	{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
	{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
	{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
	{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
	{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
	{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
	{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
	{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
	{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
	{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
	{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
	{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
	{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
	{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
	{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
	{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
	{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
	{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
	{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
	{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
	{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
	{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
	{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
	{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
	{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
	{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
	{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
	{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
	{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
	{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
	{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
	{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
	{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
	{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
	{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
	{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
	{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
	{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
	{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
	{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
	{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
	{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
	{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
	{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
	{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
	{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
	{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
	{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
	{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
	{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
	{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
	{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
	{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
	{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
	{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
	{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
	{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
	{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
	{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
	{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
	{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
	{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
	{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
	{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
	{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
	{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
	{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
	{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
	{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
	{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
	{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};


const static int cube[8][3] = {
  {0,0,0},
  {1,0,0},
  {1,0,1},
  {0,0,1},
  {0,1,0},
  {1,1,0},
  {1,1,1},
  {0,1,1},
};

//for cube vertices ordered 000,001,010, etc as xyz components, returns
//the corresponding marching cube index
const static int regularCubeIndexToMCIndex[8] = {0,3,4,7,1,2,5,6};

inline Vector3 MulCubeOffset(const Vector3& dx,int v)
{
  return Vector3((cube[v][0]?dx.x:Zero),
		 (cube[v][1]?dx.y:Zero),
		 (cube[v][2]?dx.z:Zero));
}

void EvaluateCube(ScalarFieldFunction& f,const Vector3& x,const Vector3& dx,Real vals[8])
{
  Vector v(3);
  v.copy(x + MulCubeOffset(dx,0));  vals[0] = f(v);
  v.copy(x + MulCubeOffset(dx,1));  vals[1] = f(v);
  v.copy(x + MulCubeOffset(dx,2));  vals[2] = f(v);
  v.copy(x + MulCubeOffset(dx,3));  vals[3] = f(v);
  v.copy(x + MulCubeOffset(dx,4));  vals[4] = f(v);
  v.copy(x + MulCubeOffset(dx,5));  vals[5] = f(v);
  v.copy(x + MulCubeOffset(dx,6));  vals[6] = f(v);
  v.copy(x + MulCubeOffset(dx,7));  vals[7] = f(v);
}

void EvaluateCube(Real (*f)(Real,Real,Real),const Vector3& x,const Vector3& dx,Real vals[8])
{
  Vector3 v;
  v = x + MulCubeOffset(dx,0);  vals[0] = f(v.x,v.y,v.z);
  v = x + MulCubeOffset(dx,1);  vals[1] = f(v.x,v.y,v.z);
  v = x + MulCubeOffset(dx,2);  vals[2] = f(v.x,v.y,v.z);
  v = x + MulCubeOffset(dx,3);  vals[3] = f(v.x,v.y,v.z);
  v = x + MulCubeOffset(dx,4);  vals[4] = f(v.x,v.y,v.z);
  v = x + MulCubeOffset(dx,5);  vals[5] = f(v.x,v.y,v.z);
  v = x + MulCubeOffset(dx,6);  vals[6] = f(v.x,v.y,v.z);
  v = x + MulCubeOffset(dx,7);  vals[7] = f(v.x,v.y,v.z);
}

template <class T>
void EvaluateCube(const Array3D<T>& a,int i,int j,int k,T vals[8])
{
  vals[0] = a(i  ,j  ,k  );
  vals[1] = a(i+1,j  ,k  );
  vals[2] = a(i+1,j  ,k+1);
  vals[3] = a(i  ,j  ,k+1);
  vals[4] = a(i  ,j+1,k  );
  vals[5] = a(i+1,j+1,k  );
  vals[6] = a(i+1,j+1,k+1);
  vals[7] = a(i  ,j+1,k+1);
}

Vector3 EvalCubeEdge(const Vector3& x0,const Vector3& dx,Real u,int v1,int v2)
{
  Vector3 d;
  interpolate(MulCubeOffset(dx,v1),MulCubeOffset(dx,v2),u,d);
  return x0+d;
}

void MarchingCubes(ScalarFieldFunction& input,Real isoLevel,const AABB3D& bb,const int dims[3],TriMesh& m)
{
  m.verts.resize(0);
  m.tris.resize(0);
  m.verts.reserve(dims[0]*dims[1]);
  m.tris.reserve(dims[0]*dims[1]);
  Real vals[8];
  Vector3 verts[12];
  int vertMap[12];
  Vector3 x;
  Vector3 dh(bb.bmax-bb.bmin);
  dh.x /= Real(dims[0]-1);
  dh.y /= Real(dims[1]-1);
  dh.z /= Real(dims[2]-1);

  IntTriple tri;
  x = bb.bmin;
  for (int i=0;i<dims[0]-1;i++,x.x += dh.x) {
    x.y = bb.bmin.y;
    for (int j=0;j<dims[1]-1;j++,x.y += dh.y) {
      x.z = bb.bmin.z;
      for (int k=0;k<dims[2]-1;k++,x.z += dh.z) {
	EvaluateCube(input,x,dh,vals);
	
	int cubeIndex = 0;
	if (vals[0] < isoLevel) cubeIndex |= 1;
	if (vals[1] < isoLevel) cubeIndex |= 2;
	if (vals[2] < isoLevel) cubeIndex |= 4;
	if (vals[3] < isoLevel) cubeIndex |= 8;
	if (vals[4] < isoLevel) cubeIndex |= 16;
	if (vals[5] < isoLevel) cubeIndex |= 32;
	if (vals[6] < isoLevel) cubeIndex |= 64;
	if (vals[7] < isoLevel) cubeIndex |= 128;

	if (MCEdgeTable[cubeIndex] == 0) {
	  continue;
	}

	int vertoffset = (int)m.verts.size();
	if (MCEdgeTable[cubeIndex] & 1) {
	  Real u=SegmentCrossing(vals[0], vals[1], isoLevel);
	  verts[0] = EvalCubeEdge(x,dh,u,0,1);
	}
	if (MCEdgeTable[cubeIndex] & 2) {
	  Real u=SegmentCrossing(vals[1], vals[2], isoLevel);
	  verts[1] = EvalCubeEdge(x,dh,u,1,2);
	}
	if (MCEdgeTable[cubeIndex] & 4) {
	  Real u=SegmentCrossing(vals[2], vals[3], isoLevel);
	  verts[2] = EvalCubeEdge(x,dh,u,2,3);
	}
	if (MCEdgeTable[cubeIndex] & 8) {
	  Real u=SegmentCrossing(vals[3], vals[0], isoLevel);
	  verts[3] = EvalCubeEdge(x,dh,u,3,0);
	}
	if (MCEdgeTable[cubeIndex] & 16) {
	  Real u=SegmentCrossing(vals[4], vals[5], isoLevel);
	  verts[4] = EvalCubeEdge(x,dh,u,4,5);
	}
	if (MCEdgeTable[cubeIndex] & 32) {
	  Real u=SegmentCrossing(vals[5], vals[6], isoLevel);
	  verts[5] = EvalCubeEdge(x,dh,u,5,6);
	}
	if (MCEdgeTable[cubeIndex] & 64) {
	  Real u=SegmentCrossing(vals[6], vals[7], isoLevel);
	  verts[6] = EvalCubeEdge(x,dh,u,6,7);
	}
	if (MCEdgeTable[cubeIndex] & 128) {
	  Real u=SegmentCrossing(vals[7], vals[4], isoLevel);
	  verts[7] = EvalCubeEdge(x,dh,u,7,4);
	}
	if (MCEdgeTable[cubeIndex] & 256) {
	  Real u=SegmentCrossing(vals[0], vals[4], isoLevel);
	  verts[8] = EvalCubeEdge(x,dh,u,0,4);
	}
	if (MCEdgeTable[cubeIndex] & 512) {
	  Real u=SegmentCrossing(vals[1], vals[5], isoLevel);
	  verts[9] = EvalCubeEdge(x,dh,u,1,5);
	}
	if (MCEdgeTable[cubeIndex] & 1024) {
	  Real u=SegmentCrossing(vals[2], vals[6], isoLevel);
	  verts[10] = EvalCubeEdge(x,dh,u,2,6);
	}
	if (MCEdgeTable[cubeIndex] & 2048) {
	  Real u=SegmentCrossing(vals[3], vals[7], isoLevel);
	  verts[11] = EvalCubeEdge(x,dh,u,3,7);
	}

	//get the mapping from tri vertices to mesh vertices
	for(const int* t=MCTriTable[cubeIndex];*t!=-1; t++) {
	  vertMap[*t] = (int)m.verts.size();
	  m.verts.push_back(verts[*t]);
	}
	for(const int* t=MCTriTable[cubeIndex];*t!=-1; t+=3) {
	  tri.a = vertMap[*t];
	  tri.b = vertMap[*(t+1)];
	  tri.c = vertMap[*(t+2)];
	  if(tri.a < vertoffset || tri.a >= (int)m.verts.size() ||
	     tri.b < vertoffset || tri.b >= (int)m.verts.size() ||
	     tri.c < vertoffset || tri.c >= (int)m.verts.size())
	    {
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"Internal Marching cubes error!\n");
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"Triangle "<<tri.a<<" "<<tri.b<<" "<<tri.c);
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"Starting vertex size: "<<vertoffset<<" ending vertex size "<<m.verts.size());
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"cube index "<<cubeIndex);
	      abort();
	    }
	  m.tris.push_back(tri);
	}
      }
    }
  }
  assert(m.IsValid());

  //TODO: merge vertices
}

void MarchingCubes(Real (*input)(Real,Real,Real),Real isoLevel,const AABB3D& bb,const int dims[3],TriMesh& m)
{
  m.verts.resize(0);
  m.tris.resize(0);
  m.verts.reserve(dims[0]*dims[1]);
  m.tris.reserve(dims[0]*dims[1]);
  Real vals[8];
  Vector3 verts[12];
  int vertMap[12];
  Vector3 x;
  Vector3 dh(bb.bmax-bb.bmin);
  dh.x /= Real(dims[0]-1);
  dh.y /= Real(dims[1]-1);
  dh.z /= Real(dims[2]-1);

  IntTriple tri;
  x = bb.bmin;
  for (int i=0;i<dims[0]-1;i++,x.x += dh.x) {
    x.y = bb.bmin.y;
    for (int j=0;j<dims[1]-1;j++,x.y += dh.y) {
      x.z = bb.bmin.z;
      for (int k=0;k<dims[2]-1;k++,x.z += dh.z) {
	EvaluateCube(input,x,dh,vals);
	
	int cubeIndex = 0;
	if (vals[0] < isoLevel) cubeIndex |= 1;
	if (vals[1] < isoLevel) cubeIndex |= 2;
	if (vals[2] < isoLevel) cubeIndex |= 4;
	if (vals[3] < isoLevel) cubeIndex |= 8;
	if (vals[4] < isoLevel) cubeIndex |= 16;
	if (vals[5] < isoLevel) cubeIndex |= 32;
	if (vals[6] < isoLevel) cubeIndex |= 64;
	if (vals[7] < isoLevel) cubeIndex |= 128;

	if (MCEdgeTable[cubeIndex] == 0) {
	  continue;
	}

	int vertoffset = (int)m.verts.size();
	if (MCEdgeTable[cubeIndex] & 1) {
	  Real u=SegmentCrossing(vals[0], vals[1], isoLevel);
	  verts[0] = EvalCubeEdge(x,dh,u,0,1);
	}
	if (MCEdgeTable[cubeIndex] & 2) {
	  Real u=SegmentCrossing(vals[1], vals[2], isoLevel);
	  verts[1] = EvalCubeEdge(x,dh,u,1,2);
	}
	if (MCEdgeTable[cubeIndex] & 4) {
	  Real u=SegmentCrossing(vals[2], vals[3], isoLevel);
	  verts[2] = EvalCubeEdge(x,dh,u,2,3);
	}
	if (MCEdgeTable[cubeIndex] & 8) {
	  Real u=SegmentCrossing(vals[3], vals[0], isoLevel);
	  verts[3] = EvalCubeEdge(x,dh,u,3,0);
	}
	if (MCEdgeTable[cubeIndex] & 16) {
	  Real u=SegmentCrossing(vals[4], vals[5], isoLevel);
	  verts[4] = EvalCubeEdge(x,dh,u,4,5);
	}
	if (MCEdgeTable[cubeIndex] & 32) {
	  Real u=SegmentCrossing(vals[5], vals[6], isoLevel);
	  verts[5] = EvalCubeEdge(x,dh,u,5,6);
	}
	if (MCEdgeTable[cubeIndex] & 64) {
	  Real u=SegmentCrossing(vals[6], vals[7], isoLevel);
	  verts[6] = EvalCubeEdge(x,dh,u,6,7);
	}
	if (MCEdgeTable[cubeIndex] & 128) {
	  Real u=SegmentCrossing(vals[7], vals[4], isoLevel);
	  verts[7] = EvalCubeEdge(x,dh,u,7,4);
	}
	if (MCEdgeTable[cubeIndex] & 256) {
	  Real u=SegmentCrossing(vals[0], vals[4], isoLevel);
	  verts[8] = EvalCubeEdge(x,dh,u,0,4);
	}
	if (MCEdgeTable[cubeIndex] & 512) {
	  Real u=SegmentCrossing(vals[1], vals[5], isoLevel);
	  verts[9] = EvalCubeEdge(x,dh,u,1,5);
	}
	if (MCEdgeTable[cubeIndex] & 1024) {
	  Real u=SegmentCrossing(vals[2], vals[6], isoLevel);
	  verts[10] = EvalCubeEdge(x,dh,u,2,6);
	}
	if (MCEdgeTable[cubeIndex] & 2048) {
	  Real u=SegmentCrossing(vals[3], vals[7], isoLevel);
	  verts[11] = EvalCubeEdge(x,dh,u,3,7);
	}

	//get the mapping from tri vertices to mesh vertices
	for(const int* t=MCTriTable[cubeIndex];*t!=-1; t++) {
	  vertMap[*t] = (int)m.verts.size();
	  m.verts.push_back(verts[*t]);
	}
	for(const int* t=MCTriTable[cubeIndex];*t!=-1; t+=3) {
	  tri.a = vertMap[*t];
	  tri.b = vertMap[*(t+1)];
	  tri.c = vertMap[*(t+2)];
	  if(tri.a < vertoffset || tri.a >= (int)m.verts.size() ||
	     tri.b < vertoffset || tri.b >= (int)m.verts.size() ||
	     tri.c < vertoffset || tri.c >= (int)m.verts.size())
	    {
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"Internal Marching cubes error!\n");
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"Triangle "<<tri.a<<" "<<tri.b<<" "<<tri.c);
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"Starting vertex size: "<<vertoffset<<" ending vertex size "<<m.verts.size());
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"cube index "<<cubeIndex);
	      abort();
	    }
	  m.tris.push_back(tri);
	}
      }
    }
  }
  assert(m.IsValid());

  //TODO: merge vertices
}

template <class T>
void MarchingCubes(const Array3D<T>& input,T isoLevel,const AABB3D& bb,TriMesh& m)
{
  m.verts.resize(0);
  m.tris.resize(0);
  m.verts.reserve(input.m*input.n);
  m.tris.reserve(input.m*input.n);
  T vals[8];
  Vector3 verts[12];
  int vertMap[12];
  Vector3 x;
  Vector3 dh(bb.bmax-bb.bmin);
  dh.x /= Real(input.m-1);
  dh.y /= Real(input.n-1);
  dh.z /= Real(input.p-1);

  IntTriple tri;
  x = bb.bmin;
  for (int i=0;i<input.m-1;i++,x.x += dh.x) {
    x.y = bb.bmin.y;
    for (int j=0;j<input.n-1;j++,x.y += dh.y) {
      x.z = bb.bmin.z;
      for (int k=0;k<input.p-1;k++,x.z += dh.z) {
	EvaluateCube(input,i,j,k,vals);
	
	int cubeIndex = 0;
	if (vals[0] < isoLevel) cubeIndex |= 1;
	if (vals[1] < isoLevel) cubeIndex |= 2;
	if (vals[2] < isoLevel) cubeIndex |= 4;
	if (vals[3] < isoLevel) cubeIndex |= 8;
	if (vals[4] < isoLevel) cubeIndex |= 16;
	if (vals[5] < isoLevel) cubeIndex |= 32;
	if (vals[6] < isoLevel) cubeIndex |= 64;
	if (vals[7] < isoLevel) cubeIndex |= 128;

	if (MCEdgeTable[cubeIndex] == 0) {
	  continue;
	}

	if (MCEdgeTable[cubeIndex] & 1) {
	  Real u=SegmentCrossing(vals[0], vals[1], isoLevel);
	  verts[0] = EvalCubeEdge(x,dh,u,0,1);
	}
	if (MCEdgeTable[cubeIndex] & 2) {
	  Real u=SegmentCrossing(vals[1], vals[2], isoLevel);
	  verts[1] = EvalCubeEdge(x,dh,u,1,2);
	}
	if (MCEdgeTable[cubeIndex] & 4) {
	  Real u=SegmentCrossing(vals[2], vals[3], isoLevel);
	  verts[2] = EvalCubeEdge(x,dh,u,2,3);
	}
	if (MCEdgeTable[cubeIndex] & 8) {
	  Real u=SegmentCrossing(vals[3], vals[0], isoLevel);
	  verts[3] = EvalCubeEdge(x,dh,u,3,0);
	}
	if (MCEdgeTable[cubeIndex] & 16) {
	  Real u=SegmentCrossing(vals[4], vals[5], isoLevel);
	  verts[4] = EvalCubeEdge(x,dh,u,4,5);
	}
	if (MCEdgeTable[cubeIndex] & 32) {
	  Real u=SegmentCrossing(vals[5], vals[6], isoLevel);
	  verts[5] = EvalCubeEdge(x,dh,u,5,6);
	}
	if (MCEdgeTable[cubeIndex] & 64) {
	  Real u=SegmentCrossing(vals[6], vals[7], isoLevel);
	  verts[6] = EvalCubeEdge(x,dh,u,6,7);
	}
	if (MCEdgeTable[cubeIndex] & 128) {
	  Real u=SegmentCrossing(vals[7], vals[4], isoLevel);
	  verts[7] = EvalCubeEdge(x,dh,u,7,4);
	}
	if (MCEdgeTable[cubeIndex] & 256) {
	  Real u=SegmentCrossing(vals[0], vals[4], isoLevel);
	  verts[8] = EvalCubeEdge(x,dh,u,0,4);
	}
	if (MCEdgeTable[cubeIndex] & 512) {
	  Real u=SegmentCrossing(vals[1], vals[5], isoLevel);
	  verts[9] = EvalCubeEdge(x,dh,u,1,5);
	}
	if (MCEdgeTable[cubeIndex] & 1024) {
	  Real u=SegmentCrossing(vals[2], vals[6], isoLevel);
	  verts[10] = EvalCubeEdge(x,dh,u,2,6);
	}
	if (MCEdgeTable[cubeIndex] & 2048) {
	  Real u=SegmentCrossing(vals[3], vals[7], isoLevel);
	  verts[11] = EvalCubeEdge(x,dh,u,3,7);
	}

	//get the mapping from tri vertices to mesh vertices
	for(const int* t=MCTriTable[cubeIndex];*t!=-1; t++) {
	  vertMap[*t] = (int)m.verts.size();
	  m.verts.push_back(verts[*t]);
	}
	for(const int* t=MCTriTable[cubeIndex];*t!=-1; t+=3) {
	  tri.a = vertMap[*t];
	  tri.b = vertMap[*(t+1)];
	  tri.c = vertMap[*(t+2)];
	  m.tris.push_back(tri);
	}
      }
    }
  }

  //TODO: merge vertices
}

template <class T>
void TSDFMarchingCubes(const Array3D<T>& input,T isoLevel,T truncationDistance,const AABB3D& bb,TriMesh& m)
{
  m.verts.resize(0);
  m.tris.resize(0);
  m.verts.reserve(input.m*input.n);
  m.tris.reserve(input.m*input.n);
  T vals[8];
  Vector3 verts[12];
  int vertMap[12];
  Vector3 x;
  Vector3 dh(bb.bmax-bb.bmin);
  dh.x /= Real(input.m-1);
  dh.y /= Real(input.n-1);
  dh.z /= Real(input.p-1);

  int numInternal=0,numEmpty=0,numOccupied=0;
  IntTriple tri;
  x = bb.bmin;
  for (int i=0;i<input.m-1;i++,x.x += dh.x) {
    x.y = bb.bmin.y;
    for (int j=0;j<input.n-1;j++,x.y += dh.y) {
      x.z = bb.bmin.z;
      for (int k=0;k<input.p-1;k++,x.z += dh.z) {
        EvaluateCube(input,i,j,k,vals);
        
        int cubeIndex = 0;
        if (vals[0] < isoLevel) cubeIndex |= 1;
        if (vals[1] < isoLevel) cubeIndex |= 2;
        if (vals[2] < isoLevel) cubeIndex |= 4;
        if (vals[3] < isoLevel) cubeIndex |= 8;
        if (vals[4] < isoLevel) cubeIndex |= 16;
        if (vals[5] < isoLevel) cubeIndex |= 32;
        if (vals[6] < isoLevel) cubeIndex |= 64;
        if (vals[7] < isoLevel) cubeIndex |= 128;

        if (MCEdgeTable[cubeIndex] == 0) {
          numEmpty ++;
          continue;
        }

        //CHANGE TO STANDARD MC
        //if there's a negative value next to the truncation distance, this is an internal edge
        bool internal = false;
        for(int c=0;c<8;c++)
          if(vals[c] >= truncationDistance) {
            internal=true;
            break;
          }
        if(internal) {
          numInternal ++;
          continue;
        }
        numOccupied ++;

        if (MCEdgeTable[cubeIndex] & 1) {
          Real u=SegmentCrossing(vals[0], vals[1], isoLevel);
          verts[0] = EvalCubeEdge(x,dh,u,0,1);
        }
        if (MCEdgeTable[cubeIndex] & 2) {
          Real u=SegmentCrossing(vals[1], vals[2], isoLevel);
          verts[1] = EvalCubeEdge(x,dh,u,1,2);
        }
        if (MCEdgeTable[cubeIndex] & 4) {
          Real u=SegmentCrossing(vals[2], vals[3], isoLevel);
          verts[2] = EvalCubeEdge(x,dh,u,2,3);
        }
        if (MCEdgeTable[cubeIndex] & 8) {
          Real u=SegmentCrossing(vals[3], vals[0], isoLevel);
          verts[3] = EvalCubeEdge(x,dh,u,3,0);
        }
        if (MCEdgeTable[cubeIndex] & 16) {
          Real u=SegmentCrossing(vals[4], vals[5], isoLevel);
          verts[4] = EvalCubeEdge(x,dh,u,4,5);
        }
        if (MCEdgeTable[cubeIndex] & 32) {
          Real u=SegmentCrossing(vals[5], vals[6], isoLevel);
          verts[5] = EvalCubeEdge(x,dh,u,5,6);
        }
        if (MCEdgeTable[cubeIndex] & 64) {
          Real u=SegmentCrossing(vals[6], vals[7], isoLevel);
          verts[6] = EvalCubeEdge(x,dh,u,6,7);
        }
        if (MCEdgeTable[cubeIndex] & 128) {
          Real u=SegmentCrossing(vals[7], vals[4], isoLevel);
          verts[7] = EvalCubeEdge(x,dh,u,7,4);
        }
        if (MCEdgeTable[cubeIndex] & 256) {
          Real u=SegmentCrossing(vals[0], vals[4], isoLevel);
          verts[8] = EvalCubeEdge(x,dh,u,0,4);
        }
        if (MCEdgeTable[cubeIndex] & 512) {
          Real u=SegmentCrossing(vals[1], vals[5], isoLevel);
          verts[9] = EvalCubeEdge(x,dh,u,1,5);
        }
        if (MCEdgeTable[cubeIndex] & 1024) {
          Real u=SegmentCrossing(vals[2], vals[6], isoLevel);
          verts[10] = EvalCubeEdge(x,dh,u,2,6);
        }
        if (MCEdgeTable[cubeIndex] & 2048) {
          Real u=SegmentCrossing(vals[3], vals[7], isoLevel);
          verts[11] = EvalCubeEdge(x,dh,u,3,7);
        }

        //get the mapping from tri vertices to mesh vertices
        for(const int* t=MCTriTable[cubeIndex];*t!=-1; t++) {
          vertMap[*t] = (int)m.verts.size();
          m.verts.push_back(verts[*t]);
        }
        for(const int* t=MCTriTable[cubeIndex];*t!=-1; t+=3) {
          tri.a = vertMap[*t];
          tri.b = vertMap[*(t+1)];
          tri.c = vertMap[*(t+2)];
          m.tris.push_back(tri);
        }
      }
    }
  }
  if(DEBUG) {
    printf("%d empty, %d internal, %d occupied\n",numEmpty,numInternal,numOccupied);
    if(numOccupied==0) {
      printf("Truncation distance? %f\n",float(truncationDistance));
      T vmin=Inf,vmax=-Inf;
      for(auto i=input.begin();i!=input.end();++i) {
          vmin = Min(vmin,*i);
          vmax = Max(vmax,*i);
      }
      printf("Range %f %f\n",float(vmin),float(vmax));
    }
  }
  //TODO: merge vertices
}

void CubeToMesh(const Real origvals[8],Real isoLevel,const AABB3D& bb,TriMesh& m)
{
  m.verts.resize(0);
  m.tris.resize(0);
  Real vals[8];
  for(int i=0;i<8;i++)
    vals[regularCubeIndexToMCIndex[i]] = origvals[i];
  Vector3 verts[12];
  int vertMap[12];
  const Vector3& x=bb.bmin;
  Vector3 dh=bb.bmax-bb.bmin;

  IntTriple tri;
  int cubeIndex = 0;
  if (vals[0] < isoLevel) cubeIndex |= 1;
  if (vals[1] < isoLevel) cubeIndex |= 2;
  if (vals[2] < isoLevel) cubeIndex |= 4;
  if (vals[3] < isoLevel) cubeIndex |= 8;
  if (vals[4] < isoLevel) cubeIndex |= 16;
  if (vals[5] < isoLevel) cubeIndex |= 32;
  if (vals[6] < isoLevel) cubeIndex |= 64;
  if (vals[7] < isoLevel) cubeIndex |= 128;
  
  if (MCEdgeTable[cubeIndex] == 0) {
    return;
  }
  
  if (MCEdgeTable[cubeIndex] & 1) {
    Real u=SegmentCrossing(vals[0], vals[1], isoLevel);
    verts[0] = EvalCubeEdge(x,dh,u,0,1);
  }
  if (MCEdgeTable[cubeIndex] & 2) {
    Real u=SegmentCrossing(vals[1], vals[2], isoLevel);
    verts[1] = EvalCubeEdge(x,dh,u,1,2);
  }
  if (MCEdgeTable[cubeIndex] & 4) {
    Real u=SegmentCrossing(vals[2], vals[3], isoLevel);
    verts[2] = EvalCubeEdge(x,dh,u,2,3);
  }
  if (MCEdgeTable[cubeIndex] & 8) {
    Real u=SegmentCrossing(vals[3], vals[0], isoLevel);
    verts[3] = EvalCubeEdge(x,dh,u,3,0);
  }
  if (MCEdgeTable[cubeIndex] & 16) {
    Real u=SegmentCrossing(vals[4], vals[5], isoLevel);
    verts[4] = EvalCubeEdge(x,dh,u,4,5);
  }
  if (MCEdgeTable[cubeIndex] & 32) {
    Real u=SegmentCrossing(vals[5], vals[6], isoLevel);
    verts[5] = EvalCubeEdge(x,dh,u,5,6);
  }
  if (MCEdgeTable[cubeIndex] & 64) {
    Real u=SegmentCrossing(vals[6], vals[7], isoLevel);
    verts[6] = EvalCubeEdge(x,dh,u,6,7);
  }
  if (MCEdgeTable[cubeIndex] & 128) {
    Real u=SegmentCrossing(vals[7], vals[4], isoLevel);
    verts[7] = EvalCubeEdge(x,dh,u,7,4);
  }
  if (MCEdgeTable[cubeIndex] & 256) {
    Real u=SegmentCrossing(vals[0], vals[4], isoLevel);
    verts[8] = EvalCubeEdge(x,dh,u,0,4);
  }
  if (MCEdgeTable[cubeIndex] & 512) {
    Real u=SegmentCrossing(vals[1], vals[5], isoLevel);
    verts[9] = EvalCubeEdge(x,dh,u,1,5);
  }
  if (MCEdgeTable[cubeIndex] & 1024) {
    Real u=SegmentCrossing(vals[2], vals[6], isoLevel);
    verts[10] = EvalCubeEdge(x,dh,u,2,6);
  }
  if (MCEdgeTable[cubeIndex] & 2048) {
    Real u=SegmentCrossing(vals[3], vals[7], isoLevel);
    verts[11] = EvalCubeEdge(x,dh,u,3,7);
  }

  //get the mapping from tri vertices to mesh vertices
  for(const int* t=MCTriTable[cubeIndex];*t!=-1; t++) {
    vertMap[*t] = (int)m.verts.size();
    m.verts.push_back(verts[*t]);
  }
  for(const int* t=MCTriTable[cubeIndex];*t!=-1; t+=3) {
    tri.a = vertMap[*t];
    tri.b = vertMap[*(t+1)];
    tri.c = vertMap[*(t+2)];
    m.tris.push_back(tri);
  }
}

//forward evaluations
template void MarchingCubes<char>(const Array3D<char>& input,char isoLevel,const AABB3D& bb,TriMesh& m);
template void MarchingCubes<int>(const Array3D<int>& input,int isoLevel,const AABB3D& bb,TriMesh& m);
template void MarchingCubes<float>(const Array3D<float>& input,float isoLevel,const AABB3D& bb,TriMesh& m);
template void MarchingCubes<double>(const Array3D<double>& input,double isoLevel,const AABB3D& bb,TriMesh& m);

template void TSDFMarchingCubes<char>(const Array3D<char>& input,char isoLevel,char truncationValue,const AABB3D& bb,TriMesh& m);
template void TSDFMarchingCubes<int>(const Array3D<int>& input,int isoLevel,int truncationValue,const AABB3D& bb,TriMesh& m);
template void TSDFMarchingCubes<float>(const Array3D<float>& input,float isoLevel,float truncationValue,const AABB3D& bb,TriMesh& m);
template void TSDFMarchingCubes<double>(const Array3D<double>& input,double isoLevel,double truncationValue,const AABB3D& bb,TriMesh& m);


} //namespace Meshing
