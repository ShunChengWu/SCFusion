// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"

static const _CPU_AND_GPU_CONSTANT_ int edgeTable[256] = { 0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
	0x190, 0x99, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90, 0x230, 0x339, 0x33, 0x13a, 
	0x636, 0x73f, 0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30, 0x3a0, 0x2a9, 0x1a3, 0xaa, 0x7a6, 0x6af, 0x5a5, 0x4ac, 
	0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0, 0x460, 0x569, 0x663, 0x76a, 0x66, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 
	0x86a, 0x963, 0xa69, 0xb60, 0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff, 0x3f5, 0x2fc, 0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
	0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55, 0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950, 0x7c0, 0x6c9, 0x5c3, 0x4ca, 
	0x3c6, 0x2cf, 0x1c5, 0xcc, 0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0, 0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc, 
	0xcc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0, 0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x55, 0x35f, 0x256,
	0x55a, 0x453, 0x759, 0x650, 0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5, 0xff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
	0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f, 0x66, 0x76a, 0x663, 0x569, 0x460, 0xca0, 0xda9, 0xea3, 0xfaa, 
	0x8a6, 0x9af, 0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa, 0x1a3, 0x2a9, 0x3a0, 0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 
	0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33, 0x339, 0x230, 0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c, 0x69c, 0x795, 0x49f, 0x596, 
	0x29a, 0x393, 0x99, 0x190, 0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c, 0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0 };

static const _CPU_AND_GPU_CONSTANT_ int triangleTable[256][16] = { { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1 }, { 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1 }, { 3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1 }, { 3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
{ 9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1 }, { 9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
{ 2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1 }, { 8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1 }, { 9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
{ 4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1 }, { 3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1 }, { 4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
{ 4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1 }, { 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 }, { 5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
{ 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1 }, { 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 }, { 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
{ 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1 }, { 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1 },
{ 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1 }, { 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
{ 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1 }, { 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1 }, { 0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1 },
{ 10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1 }, { 8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1 },
{ 2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1 }, { 7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1 },
{ 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1 }, { 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1 },
{ 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1 }, { 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1 },
{ 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1 }, { 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1 },
{ 11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 }, { 1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1 }, { 9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
{ 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1 }, { 2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 }, { 0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
{ 5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1 }, { 6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1 }, { 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1 },
{ 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1 }, { 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1 }, { 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
{ 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1 }, { 6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1 }, { 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1 },
{ 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1 }, { 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
{ 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1 }, { 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1 },
{ 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1 }, { 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
{ 5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1 }, { 0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1 },
{ 6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1 }, { 10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1 }, { 10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
{ 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1 }, { 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1 }, { 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1 }, { 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1 }, { 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
{ 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1 }, { 9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1 },
{ 8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1 }, { 3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
{ 6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1 }, { 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1 },
{ 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
{ 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1 }, { 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
{ 7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
{ 2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1 }, { 1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1 },
{ 11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1 }, { 8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1 },
{ 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1 },
{ 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 }, { 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 }, { 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
{ 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1 }, { 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1 }, { 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1 }, { 10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
{ 10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1 }, { 0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1 },
{ 7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1 }, { 6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1 }, { 8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1 },
{ 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1 }, { 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1 },
{ 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1 }, { 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1 },
{ 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1 }, { 8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1 },
{ 1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1 }, { 8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1 },
{ 10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1 }, { 4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1 },
{ 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 }, { 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
{ 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1 }, { 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
{ 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1 }, { 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1 },
{ 3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1 }, { 7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1 },
{ 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1 }, { 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1 },
{ 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1 }, { 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1 },
{ 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1 }, { 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1 },
{ 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1 }, { 6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1 }, { 0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1 },
{ 6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1 },
{ 0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1 }, { 11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1 },
{ 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1 }, { 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1 },
{ 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1 }, { 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1 },
{ 1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1 },
{ 10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1 }, { 0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1 }, { 5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1 },
{ 10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1 }, { 11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1 }, { 9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1 },
{ 7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1 }, { 2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
{ 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1 }, { 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1 },
{ 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1 }, { 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1 }, { 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
{ 9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
{ 5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1 }, { 0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1 },
{ 10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1 }, { 2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1 },
{ 0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1 }, { 0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1 },
{ 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1 },
{ 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1 }, { 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1 },
{ 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1 }, { 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1 },
{ 9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1 }, { 1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1 },
{ 3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1 }, { 4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1 },
{ 9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1 }, { 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
{ 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1 }, { 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1 },
{ 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1 }, { 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1 },
{ 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
{ 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1 }, { 4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1 }, { 0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1 }, { 0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
{ 9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1 },
{ 1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } };

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline bool findPointNeighbors(THREADPTR(Vector3f) *p, THREADPTR(float) *sdf, Vector3i blockLocation, const CONSTPTR(TVoxel) *localVBA, 
	const CONSTPTR(ITMHashEntry) *hashTable, bool checkVoxel)
{
	int vmIndex; Vector3i localBlockLocation;
    TVoxel voxel;

	localBlockLocation = blockLocation + Vector3i(0, 0, 0); p[0] = localBlockLocation.toFloat();
	voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[0] = TVoxel::valueToFloat(voxel.sdf);
	if (!vmIndex) return false;
	if(checkVoxel) if (!checkVoxelState(voxel)) return false;

	localBlockLocation = blockLocation + Vector3i(1, 0, 0); p[1] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[1] = TVoxel::valueToFloat(voxel.sdf);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState(voxel)) return false;

	localBlockLocation = blockLocation + Vector3i(1, 1, 0); p[2] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[2] = TVoxel::valueToFloat(voxel.sdf);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState(voxel)) return false;

	localBlockLocation = blockLocation + Vector3i(0, 1, 0); p[3] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[3] = TVoxel::valueToFloat(voxel.sdf);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState(voxel)) return false;

	localBlockLocation = blockLocation + Vector3i(0, 0, 1); p[4] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[4] = TVoxel::valueToFloat(voxel.sdf);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState(voxel)) return false;

	localBlockLocation = blockLocation + Vector3i(1, 0, 1); p[5] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[5] = TVoxel::valueToFloat(voxel.sdf);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState(voxel)) return false;

	localBlockLocation = blockLocation + Vector3i(1, 1, 1); p[6] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[6] = TVoxel::valueToFloat(voxel.sdf);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState(voxel)) return false;

	localBlockLocation = blockLocation + Vector3i(0, 1, 1); p[7] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[7] = TVoxel::valueToFloat(voxel.sdf);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState(voxel)) return false;

	return true;
}
#if 0 // label
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline bool findPointNeighborsLabel(THREADPTR(Vector3f) *p, THREADPTR(float) *sdf, THREADPTR(uint) *label, Vector3i blockLocation, const CONSTPTR(TVoxel) *localVBA,
                                                  const CONSTPTR(ITMHashEntry) *hashTable, bool checkVoxel)
{
    int vmIndex; Vector3i localBlockLocation;
    TVoxel voxel;

    localBlockLocation = blockLocation + Vector3i(0, 0, 0); p[0] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[0] = TVoxel::valueToFloat(voxel.sdf);
    label[0] = accessLabel<TVoxel>::readLabel(voxel);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState<TVoxel::integrateType>(sdf[0])) return false;

    localBlockLocation = blockLocation + Vector3i(1, 0, 0); p[1] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[1] = TVoxel::valueToFloat(voxel.sdf);
    label[1] = accessLabel<TVoxel>::readLabel(voxel);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState<TVoxel::integrateType>(sdf[1])) return false;

    localBlockLocation = blockLocation + Vector3i(1, 1, 0); p[2] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[2] = TVoxel::valueToFloat(voxel.sdf);
    label[2] = accessLabel<TVoxel>::readLabel(voxel);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState<TVoxel::integrateType>(sdf[2])) return false;

    localBlockLocation = blockLocation + Vector3i(0, 1, 0); p[3] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[3] = TVoxel::valueToFloat(voxel.sdf);
    label[3] = accessLabel<TVoxel>::readLabel(voxel);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState<TVoxel::integrateType>(sdf[3])) return false;

    localBlockLocation = blockLocation + Vector3i(0, 0, 1); p[4] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[4] = TVoxel::valueToFloat(voxel.sdf);
    label[4] = accessLabel<TVoxel>::readLabel(voxel);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState<TVoxel::integrateType>(sdf[4])) return false;

    localBlockLocation = blockLocation + Vector3i(1, 0, 1); p[5] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[5] = TVoxel::valueToFloat(voxel.sdf);
    label[5] = accessLabel<TVoxel>::readLabel(voxel);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState<TVoxel::integrateType>(sdf[5])) return false;

    localBlockLocation = blockLocation + Vector3i(1, 1, 1); p[6] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[6] = TVoxel::valueToFloat(voxel.sdf);
    label[6] = accessLabel<TVoxel>::readLabel(voxel);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState<TVoxel::integrateType>(sdf[6])) return false;

    localBlockLocation = blockLocation + Vector3i(0, 1, 1); p[7] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex);
    sdf[7] = TVoxel::valueToFloat(voxel.sdf);
    label[7] = accessLabel<TVoxel>::readLabel(voxel);
    if (!vmIndex) return false;
    if(checkVoxel) if (!checkVoxelState<TVoxel::integrateType>(sdf[7])) return false;

    return true;
}
#endif
#if 0
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline bool findPointNeighbors(THREADPTR(Vector3f) *p, THREADPTR(float) *sdf, Vector3i blockLocation, const CONSTPTR(TVoxel) *localVBA,
                                                  const CONSTPTR(SCFUSION::SCVoxelArrayInfo) *arrayInfo)
{
    int vmIndex; Vector3i localBlockLocation;
    TVoxel voxel;

    localBlockLocation = blockLocation + Vector3i(0, 0, 0); p[0] = localBlockLocation.toFloat();
    sdf[0] = TVoxel::valueToFloat(readVoxel(localVBA, arrayInfo, localBlockLocation, vmIndex).sdf);
    if (!vmIndex || !checkVoxelState<TVoxel::integrateType>(sdf[0])) return false;

    localBlockLocation = blockLocation + Vector3i(1, 0, 0); p[1] = localBlockLocation.toFloat();
    sdf[1] = TVoxel::valueToFloat(readVoxel(localVBA, arrayInfo, localBlockLocation, vmIndex).sdf);
    if (!vmIndex || !checkVoxelState<TVoxel::integrateType>(sdf[1])) return false;

    localBlockLocation = blockLocation + Vector3i(1, 1, 0); p[2] = localBlockLocation.toFloat();
    sdf[2] = TVoxel::valueToFloat(readVoxel(localVBA, arrayInfo, localBlockLocation, vmIndex).sdf);
    if (!vmIndex || !checkVoxelState<TVoxel::integrateType>(sdf[2])) return false;

    localBlockLocation = blockLocation + Vector3i(0, 1, 0); p[3] = localBlockLocation.toFloat();
    sdf[3] = TVoxel::valueToFloat(readVoxel(localVBA, arrayInfo, localBlockLocation, vmIndex).sdf);
    if (!vmIndex || !checkVoxelState<TVoxel::integrateType>(sdf[3])) return false;

    localBlockLocation = blockLocation + Vector3i(0, 0, 1); p[4] = localBlockLocation.toFloat();
    sdf[4] = TVoxel::valueToFloat(readVoxel(localVBA, arrayInfo, localBlockLocation, vmIndex).sdf);
    if (!vmIndex || !checkVoxelState<TVoxel::integrateType>(sdf[4])) return false;

    localBlockLocation = blockLocation + Vector3i(1, 0, 1); p[5] = localBlockLocation.toFloat();
    sdf[5] = TVoxel::valueToFloat(readVoxel(localVBA, arrayInfo, localBlockLocation, vmIndex).sdf);
    if (!vmIndex || !checkVoxelState<TVoxel::integrateType>(sdf[5])) return false;

    localBlockLocation = blockLocation + Vector3i(1, 1, 1); p[6] = localBlockLocation.toFloat();
    sdf[6] = TVoxel::valueToFloat(readVoxel(localVBA, arrayInfo, localBlockLocation, vmIndex).sdf);
    if (!vmIndex || !checkVoxelState<TVoxel::integrateType>(sdf[6])) return false;

    localBlockLocation = blockLocation + Vector3i(0, 1, 1); p[7] = localBlockLocation.toFloat();
    sdf[7] = TVoxel::valueToFloat(readVoxel(localVBA, arrayInfo, localBlockLocation, vmIndex).sdf);
    if (!vmIndex || !checkVoxelState<TVoxel::integrateType>(sdf[7])) return false;

    return true;
}
#endif
template<SCFUSION::IntegrateType T>
_CPU_AND_GPU_CODE_ inline bool findPointNeighbors(THREADPTR(Vector3f) *p, THREADPTR(float) *sdf, Vector3i blockLocation, const CONSTPTR(float) *localVBA,
                                                  const CONSTPTR(Vector3s) &dims)
{
    int vmIndex; Vector3i localBlockLocation;
    float voxel;

    localBlockLocation = blockLocation + Vector3i(0, 0, 0); p[0] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, dims, localBlockLocation, vmIndex);
    sdf[0] = voxel;
  if (!vmIndex) return false;

    localBlockLocation = blockLocation + Vector3i(1, 0, 0); p[1] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, dims, localBlockLocation, vmIndex);
    sdf[1] = voxel;
//    if (!vmIndex || !checkVoxelState<T>(sdf[1])) return false;
    if (!vmIndex) return false;

    localBlockLocation = blockLocation + Vector3i(1, 1, 0); p[2] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, dims, localBlockLocation, vmIndex);
    sdf[2] = voxel;
//    if (!vmIndex || !checkVoxelState<T>(sdf[2])) return false;
    if (!vmIndex) return false;

    localBlockLocation = blockLocation + Vector3i(0, 1, 0); p[3] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, dims, localBlockLocation, vmIndex);
    sdf[3] = voxel;
//    if (!vmIndex || !checkVoxelState<T>(sdf[3])) return false;
    if (!vmIndex) return false;

    localBlockLocation = blockLocation + Vector3i(0, 0, 1); p[4] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, dims, localBlockLocation, vmIndex);
    sdf[4] = voxel;
//    if (!vmIndex || !checkVoxelState<T>(sdf[4])) return false;
    if (!vmIndex) return false;

    localBlockLocation = blockLocation + Vector3i(1, 0, 1); p[5] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, dims, localBlockLocation, vmIndex);
    sdf[5] = voxel;
//    if (!vmIndex || !checkVoxelState<T>(sdf[5])) return false;
    if (!vmIndex) return false;

    localBlockLocation = blockLocation + Vector3i(1, 1, 1); p[6] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, dims, localBlockLocation, vmIndex);
    sdf[6] = voxel;
//    if (!vmIndex || !checkVoxelState<T>(sdf[6])) return false;
    if (!vmIndex) return false;

    localBlockLocation = blockLocation + Vector3i(0, 1, 1); p[7] = localBlockLocation.toFloat();
    voxel = readVoxel(localVBA, dims, localBlockLocation, vmIndex);
    sdf[7] = voxel;
//    if (!vmIndex || !checkVoxelState<T>(sdf[7])) return false;
    if (!vmIndex) return false;

    return true;
}

_CPU_AND_GPU_CODE_ inline Vector3f sdfInterp(const THREADPTR(Vector3f) &p1, const THREADPTR(Vector3f) &p2, float valp1, float valp2, float isoValue)
{
	if (fabs(isoValue - valp1) < 0.00001f) return p1;
	if (fabs(isoValue - valp2) < 0.00001f) return p2;
	if (fabs(valp1 - valp2) < 0.00001f) return p1;

	return p1 + ((isoValue - valp1) / (valp2 - valp1)) * (p2 - p1);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline int calCubeIndex(THREADPTR(Vector3f) *points, THREADPTR(float) *sdfVals, Vector3i globalPos, Vector3i localPos,
        const CONSTPTR(TVoxel) *localVBA, const CONSTPTR(ITMHashEntry) *hashTable, float isoValue, bool checkVoxelState) {
    if (!findPointNeighbors<TVoxel>(points, sdfVals, globalPos + localPos, localVBA, hashTable, checkVoxelState)) return -1;

    int cubeIndex = 0;
    if (sdfVals[0] < isoValue) cubeIndex |= 1;
    if (sdfVals[1] < isoValue) cubeIndex |= 2;
    if (sdfVals[2] < isoValue) cubeIndex |= 4;
    if (sdfVals[3] < isoValue) cubeIndex |= 8;
    if (sdfVals[4] < isoValue) cubeIndex |= 16;
    if (sdfVals[5] < isoValue) cubeIndex |= 32;
    if (sdfVals[6] < isoValue) cubeIndex |= 64;
    if (sdfVals[7] < isoValue) cubeIndex |= 128;
    return cubeIndex;
}
#if 1 //label
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline int calCubeIndexLabels(Vector3f &point, float &sdfVal, uint &label, Vector3i globalPos, Vector3i localPos,
                                           const CONSTPTR(TVoxel) *localVBA, const CONSTPTR(ITMHashEntry) *hashTable, float isoValue, bool checkState) {
//    Vector3f points[4];
//    float sdfVals[4];
    int vmIndex;

//    bool state = false;
    int cubeIndex = -1;
//    {
//        TVoxel voxeltmp;
//        points[0] = TO_FLOAT3(globalPos + localPos + Vector3i(0,0,0));
//        voxeltmp = readVoxel(localVBA, hashTable, points[0].toInt(), vmIndex);
//        sdfVals[0] = TVoxel::valueToFloat(voxeltmp.sdf);
//
//        points[1] = TO_FLOAT3(globalPos + localPos + Vector3i(1,0,0));
//        voxeltmp = readVoxel(localVBA, hashTable, points[1].toInt(), vmIndex);
//        sdfVals[1] = TVoxel::valueToFloat(voxeltmp.sdf);
//
//        points[2] = TO_FLOAT3(globalPos + localPos + Vector3i(0,1,0));
//        voxeltmp = readVoxel(localVBA, hashTable, points[2].toInt(), vmIndex);
//        sdfVals[2] = TVoxel::valueToFloat(voxeltmp.sdf);
//
//        points[3] = TO_FLOAT3(globalPos + localPos + Vector3i(0,0,1));
//        voxeltmp = readVoxel(localVBA, hashTable, points[3].toInt(), vmIndex);
//        sdfVals[3] = TVoxel::valueToFloat(voxeltmp.sdf);
//    }
//    if(sdfVals[0] > isoValue) cubeIndex |= 1;
//    if(sdfVals[1] > isoValue) cubeIndex |= 2;
//    if(sdfVals[2] > isoValue) cubeIndex |= 4;
//    if(sdfVals[3] > isoValue) cubeIndex |= 8;


    point = TO_FLOAT3(globalPos + localPos);
    TVoxel voxel = readVoxel(localVBA, hashTable, globalPos + localPos, vmIndex);
    sdfVal = TVoxel::valueToFloat(voxel.sdf);
    label = accessLabel<TVoxel>::readLabel(voxel);


//    if(checkState) state = checkVoxelState<TVoxel::integrateType>(sdfVal);
//    if (label == 0 || sdfVal < isoValue) return cubeIndex;
    if (sdfVal < isoValue) return cubeIndex;
    cubeIndex = 12;
    return cubeIndex;
}
#endif

#if 0 // SCVoxelArrayInfo
template<SCFUSION::IntegrateType T, class TVoxel>
_CPU_AND_GPU_CODE_ inline int calCubeIndex(THREADPTR(Vector3f) *points, THREADPTR(float) *sdfVals, Vector3i globalPos,
                                      const CONSTPTR(TVoxel) *localVBA, const CONSTPTR(SCFUSION::SCVoxelArrayInfo) *arrayInfo, float isoValue) {
    if (!findPointNeighbors<T, TVoxel>(points, sdfVals, globalPos, localVBA, arrayInfo)) return -1;

    int cubeIndex = 0;
    if (sdfVals[0] < isoValue) cubeIndex |= 1;
    if (sdfVals[1] < isoValue) cubeIndex |= 2;
    if (sdfVals[2] < isoValue) cubeIndex |= 4;
    if (sdfVals[3] < isoValue) cubeIndex |= 8;
    if (sdfVals[4] < isoValue) cubeIndex |= 16;
    if (sdfVals[5] < isoValue) cubeIndex |= 32;
    if (sdfVals[6] < isoValue) cubeIndex |= 64;
    if (sdfVals[7] < isoValue) cubeIndex |= 128;
    return cubeIndex;
}
#endif

template<SCFUSION::IntegrateType T>
_CPU_AND_GPU_CODE_ inline int calCubeIndex(THREADPTR(Vector3f) *points, THREADPTR(float) *sdfVals, Vector3i globalPos,
                                           const CONSTPTR(float) *localVBA, const CONSTPTR(Vector3s) &dims, float isoValue) {
    if (!findPointNeighbors<T>(points, sdfVals, globalPos, localVBA, dims)) {
        return -1;
    }

    int cubeIndex = 0;
    if (sdfVals[0] < isoValue) cubeIndex |= 1;
    if (sdfVals[1] < isoValue) cubeIndex |= 2;
    if (sdfVals[2] < isoValue) cubeIndex |= 4;
    if (sdfVals[3] < isoValue) cubeIndex |= 8;
    if (sdfVals[4] < isoValue) cubeIndex |= 16;
    if (sdfVals[5] < isoValue) cubeIndex |= 32;
    if (sdfVals[6] < isoValue) cubeIndex |= 64;
    if (sdfVals[7] < isoValue) cubeIndex |= 128;
    return cubeIndex;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline int buildVertList(THREADPTR(Vector3f) *vertList, Vector3i globalPos, Vector3i localPos,
        const CONSTPTR(TVoxel) *localVBA, const CONSTPTR(ITMHashEntry) *hashTable, float isoValue, bool checkVoxelState)
{
	Vector3f points[8]; float sdfVals[8];
	int cubeIndex = calCubeIndex<TVoxel>(points, sdfVals,globalPos, localPos, localVBA, hashTable, isoValue, checkVoxelState);
    if( cubeIndex < 0) return -1;
	if (edgeTable[cubeIndex] == 0) return -1;

	if (edgeTable[cubeIndex] & 1) vertList[0] = sdfInterp(points[0], points[1], sdfVals[0], sdfVals[1], isoValue);
	if (edgeTable[cubeIndex] & 2) vertList[1] = sdfInterp(points[1], points[2], sdfVals[1], sdfVals[2], isoValue);
	if (edgeTable[cubeIndex] & 4) vertList[2] = sdfInterp(points[2], points[3], sdfVals[2], sdfVals[3], isoValue);
	if (edgeTable[cubeIndex] & 8) vertList[3] = sdfInterp(points[3], points[0], sdfVals[3], sdfVals[0], isoValue);
	if (edgeTable[cubeIndex] & 16) vertList[4] = sdfInterp(points[4], points[5], sdfVals[4], sdfVals[5], isoValue);
	if (edgeTable[cubeIndex] & 32) vertList[5] = sdfInterp(points[5], points[6], sdfVals[5], sdfVals[6], isoValue);
	if (edgeTable[cubeIndex] & 64) vertList[6] = sdfInterp(points[6], points[7], sdfVals[6], sdfVals[7], isoValue);
	if (edgeTable[cubeIndex] & 128) vertList[7] = sdfInterp(points[7], points[4], sdfVals[7], sdfVals[4], isoValue);
	if (edgeTable[cubeIndex] & 256) vertList[8] = sdfInterp(points[0], points[4], sdfVals[0], sdfVals[4], isoValue);
	if (edgeTable[cubeIndex] & 512) vertList[9] = sdfInterp(points[1], points[5], sdfVals[1], sdfVals[5], isoValue);
	if (edgeTable[cubeIndex] & 1024) vertList[10] = sdfInterp(points[2], points[6], sdfVals[2], sdfVals[6], isoValue);
	if (edgeTable[cubeIndex] & 2048) vertList[11] = sdfInterp(points[3], points[7], sdfVals[3], sdfVals[7], isoValue);

	return cubeIndex;
}
#if 1 // label
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline int buildVertListLabel(THREADPTR(Vector3f) *points, Vector3i globalPos, Vector3i localPos,
                                            const CONSTPTR(TVoxel) *localVBA, const CONSTPTR(ITMHashEntry) *hashTable, float isoValue, bool checkVoxelState)
{
    Vector3f point; float sdfVal; uint label;
    int cubeIndex = calCubeIndexLabels<TVoxel>(point, sdfVal, label,globalPos, localPos, localVBA, hashTable, isoValue, checkVoxelState);
    if( cubeIndex < 0) return -1;
    points[0] = point + Vector3f(0,0,0);
    points[1] = point + Vector3f(0,0,1);
    points[2] = point + Vector3f(0,1,0);
    points[3] = point + Vector3f(0,1,1);
    points[4] = point + Vector3f(1,0,0);
    points[5] = point + Vector3f(1,0,1);
    points[6] = point + Vector3f(1,1,0);
    points[7] = point + Vector3f(1,1,1);
    return cubeIndex;
}
#endif
#if 0
template<SCFUSION::IntegrateType T, class TVoxel>
_CPU_AND_GPU_CODE_ inline int buildVertList(THREADPTR(Vector3f) *vertList, Vector3i Pos,
        const CONSTPTR(TVoxel) *localVBA, const CONSTPTR(SCFUSION::SCVoxelArrayInfo) *arrayInfo, float isoValue)
{
    Vector3f points[8]; float sdfVals[8];
    int cubeIndex = calCubeIndex<T,TVoxel>(points, sdfVals, Pos, localVBA, arrayInfo, isoValue);
    if( cubeIndex < 0) return -1;
    if (edgeTable[cubeIndex] == 0) return -1;

    if (edgeTable[cubeIndex] & 1) vertList[0] = sdfInterp(points[0], points[1], sdfVals[0], sdfVals[1], isoValue);
    if (edgeTable[cubeIndex] & 2) vertList[1] = sdfInterp(points[1], points[2], sdfVals[1], sdfVals[2], isoValue);
    if (edgeTable[cubeIndex] & 4) vertList[2] = sdfInterp(points[2], points[3], sdfVals[2], sdfVals[3], isoValue);
    if (edgeTable[cubeIndex] & 8) vertList[3] = sdfInterp(points[3], points[0], sdfVals[3], sdfVals[0], isoValue);
    if (edgeTable[cubeIndex] & 16) vertList[4] = sdfInterp(points[4], points[5], sdfVals[4], sdfVals[5], isoValue);
    if (edgeTable[cubeIndex] & 32) vertList[5] = sdfInterp(points[5], points[6], sdfVals[5], sdfVals[6], isoValue);
    if (edgeTable[cubeIndex] & 64) vertList[6] = sdfInterp(points[6], points[7], sdfVals[6], sdfVals[7], isoValue);
    if (edgeTable[cubeIndex] & 128) vertList[7] = sdfInterp(points[7], points[4], sdfVals[7], sdfVals[4], isoValue);
    if (edgeTable[cubeIndex] & 256) vertList[8] = sdfInterp(points[0], points[4], sdfVals[0], sdfVals[4], isoValue);
    if (edgeTable[cubeIndex] & 512) vertList[9] = sdfInterp(points[1], points[5], sdfVals[1], sdfVals[5], isoValue);
    if (edgeTable[cubeIndex] & 1024) vertList[10] = sdfInterp(points[2], points[6], sdfVals[2], sdfVals[6], isoValue);
    if (edgeTable[cubeIndex] & 2048) vertList[11] = sdfInterp(points[3], points[7], sdfVals[3], sdfVals[7], isoValue);

    return cubeIndex;
}
#endif

template<SCFUSION::IntegrateType T>
_CPU_AND_GPU_CODE_ inline int buildVertList(THREADPTR(Vector3f) *vertList, Vector3i Pos,
                                            const CONSTPTR(float) *localVBA, const CONSTPTR(Vector3s) &dims, float isoValue)
{
    Vector3f points[8]; float sdfVals[8];
    int cubeIndex = calCubeIndex<T>(points, sdfVals, Pos, localVBA, dims, isoValue);
    if( cubeIndex < 0) return -1;
    if (edgeTable[cubeIndex] == 0) return -1;

    if (edgeTable[cubeIndex] & 1) vertList[0] = sdfInterp(points[0], points[1], sdfVals[0], sdfVals[1], isoValue);
    if (edgeTable[cubeIndex] & 2) vertList[1] = sdfInterp(points[1], points[2], sdfVals[1], sdfVals[2], isoValue);
    if (edgeTable[cubeIndex] & 4) vertList[2] = sdfInterp(points[2], points[3], sdfVals[2], sdfVals[3], isoValue);
    if (edgeTable[cubeIndex] & 8) vertList[3] = sdfInterp(points[3], points[0], sdfVals[3], sdfVals[0], isoValue);
    if (edgeTable[cubeIndex] & 16) vertList[4] = sdfInterp(points[4], points[5], sdfVals[4], sdfVals[5], isoValue);
    if (edgeTable[cubeIndex] & 32) vertList[5] = sdfInterp(points[5], points[6], sdfVals[5], sdfVals[6], isoValue);
    if (edgeTable[cubeIndex] & 64) vertList[6] = sdfInterp(points[6], points[7], sdfVals[6], sdfVals[7], isoValue);
    if (edgeTable[cubeIndex] & 128) vertList[7] = sdfInterp(points[7], points[4], sdfVals[7], sdfVals[4], isoValue);
    if (edgeTable[cubeIndex] & 256) vertList[8] = sdfInterp(points[0], points[4], sdfVals[0], sdfVals[4], isoValue);
    if (edgeTable[cubeIndex] & 512) vertList[9] = sdfInterp(points[1], points[5], sdfVals[1], sdfVals[5], isoValue);
    if (edgeTable[cubeIndex] & 1024) vertList[10] = sdfInterp(points[2], points[6], sdfVals[2], sdfVals[6], isoValue);
    if (edgeTable[cubeIndex] & 2048) vertList[11] = sdfInterp(points[3], points[7], sdfVals[3], sdfVals[7], isoValue);

    return cubeIndex;
}

_CPU_AND_GPU_CODE_ inline Vector3f calcNormal(Vector3f *v0, Vector3f *v1, Vector3f *v2) {
    Vector3f edge0 = *v1 - *v0;
    Vector3f edge1 = *v2 - *v0;
    return cross(edge0, edge1);
}

_CPU_AND_GPU_CODE_ inline Vector4f calcNormal(const Vector4f &v0, const Vector4f &v1, const Vector4f &v2) {
    return cross(v1 - v0, v2 - v0);
}


template<class TVoxel>
_CPU_AND_GPU_CODE_  inline void classifyVoxel_shared(uint i, float isoValue, uint *voxelVerts, uint *voxelOccupied,
 const Vector4s *visibleBlockPos, const TVoxel *localVBA, const ITMHashEntry *hashTable, bool checkState)
{
    const Vector4s Pos_4s = visibleBlockPos[i];

    if (Pos_4s.w == 0) return;

    Vector3f points[8]; float sdfVals[8];
    int cubeIndex = 0;
    cubeIndex = calCubeIndex<TVoxel>(points, sdfVals, Vector3i(Pos_4s.x,Pos_4s.y,Pos_4s.z), Vector3i(0,0,0),localVBA, hashTable, isoValue, checkState);

    if (cubeIndex < 0 || edgeTable[cubeIndex] == 0) {
        voxelVerts[i] = 0;
        voxelOccupied[i] = 0;
        return;
    }

    uint numTriangle = 0;
    for (int j = 0; triangleTable[cubeIndex][j] != -1; j += 3)
        numTriangle++;
    voxelVerts[i] = numTriangle;
    voxelOccupied[i] = (numTriangle > 0);
}
template<class TVoxel>
_CPU_AND_GPU_CODE_  inline void classifyVoxelLabel_shared(uint i, float isoValue, uint *voxelVerts, uint *voxelOccupied,
                                                     const Vector4s *visibleBlockPos, const TVoxel *localVBA, const ITMHashEntry *hashTable, bool checkState)
{
    const Vector4s Pos_4s = visibleBlockPos[i];

    if (Pos_4s.w == 0) return;

    Vector3f point;float sdf; uint label;
    int cubeIndex = calCubeIndexLabels(point,sdf,label,Vector3i(Pos_4s.x,Pos_4s.y,Pos_4s.z), Vector3i(0,0,0), localVBA,hashTable,isoValue, checkState);
    if(cubeIndex<0) {
        voxelVerts[i] = 0;
        voxelOccupied[i] = 0;
        return;
    }
    voxelVerts[i] = cubeIndex;
    voxelOccupied[i] = 1;
}

#if 0
template<class TVoxel>
_CPU_AND_GPU_CODE_  inline void classifyVoxel_shared(uint i, float isoValue, uint *voxelVerts, uint *voxelOccupied,
                                                     const Vector4s &Pos_4s, const TVoxel *localVBA, const SCFUSION::SCVoxelArrayInfo *arrayInfo) {
    Vector3f points[8];
    float sdfVals[8];
    int cubeIndex = calCubeIndex<TVoxel::integrateType>(points, sdfVals, Vector3i(Pos_4s.x, Pos_4s.y, Pos_4s.z), localVBA, arrayInfo,
                                    isoValue);
    if (cubeIndex < 0 || edgeTable[cubeIndex] == 0) {
        voxelVerts[i] = 0;
        voxelOccupied[i] = 0;
        return;
    }

    uint numTriangle = 0;
    for (int j = 0; triangleTable[cubeIndex][j] != -1; j += 3)
        numTriangle++;
    voxelVerts[i] = numTriangle;
    voxelOccupied[i] = (numTriangle > 0);
}
#endif
template<SCFUSION::IntegrateType T>
_CPU_AND_GPU_CODE_  inline void classifyVoxel_shared(uint i, float isoValue, uint *voxelVerts, uint *voxelOccupied,
                                                     const Vector4s &Pos_4s, const float *localVBA, const Vector3s &dims) {
    Vector3f points[8];
    float sdfVals[8];
    int cubeIndex = calCubeIndex<T>(points, sdfVals, Vector3i(Pos_4s.x, Pos_4s.y, Pos_4s.z), localVBA, dims, isoValue);
//    if(localVBA[i]>0)printf("[%d %d %d]\n", Pos_4s.x,Pos_4s.y,Pos_4s.z);
    if (cubeIndex < 0 || edgeTable[cubeIndex] == 0) {
        voxelVerts[i] = 0;
        voxelOccupied[i] = 0;
        return;
    }

    uint numTriangle = 0;
    for (int j = 0; triangleTable[cubeIndex][j] != -1; j += 3)
        numTriangle++;
    voxelVerts[i] = numTriangle;
    voxelOccupied[i] = (numTriangle > 0);
}

_CPU_AND_GPU_CODE_ inline void compactVoxels(int i, uint *compactedVoxelArray, uint *voxelOccupied, uint *voxelOccupiedScan, uint numVoxels)
{
    if(voxelOccupied[i])
        if (voxelOccupiedScan[i] < numVoxels)
            compactedVoxelArray[voxelOccupiedScan[i]] = i;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void generateTriangles(uint i, ITMLib::ITMMesh::Triangle *triangles, ITMLib::ITMMesh::Normal *normals, ITMLib::ITMMesh::Color *colors, const Vector4f *labelColorList,
        float factor, uint noMaxTriangles, float isoValue, uint *compactedVoxelArray, uint *numVertsScanned, const Vector4s *visibleBlockGlobalPos, const TVoxel *localVBA, const ITMHashEntry *hashTable, bool checkVoxelState)
{
    const uint& voxel = compactedVoxelArray[i];
    const Vector4s globalPos_4s = visibleBlockGlobalPos[voxel];

    if (globalPos_4s.w == 0) return;

    Vector3i globalPos = Vector3i(globalPos_4s.x, globalPos_4s.y, globalPos_4s.z);

    Vector4f color(1, 1, 1, 1);
    if (TVoxel::hasLabelInformation && labelColorList != nullptr)
        color = labelColorList[accessLabel<TVoxel>::readMaxWNeighborLabel(globalPos,
                                                                                                      localVBA,
                                                                                                      hashTable,
                                                                                                      1)];

    Vector3f vertList[12];
    int cubeIndex = buildVertList<TVoxel>(vertList, globalPos, Vector3i(0,0,0), localVBA, hashTable, isoValue, checkVoxelState);

    if (cubeIndex < 0) return;

    for (int j = 0; triangleTable[cubeIndex][j] != -1; j += 3) {
        uint triangleId = numVertsScanned[voxel] + j / 3;
        if (triangleId >= noMaxTriangles) continue;
        triangles[triangleId].p0 = Vector4f(vertList[triangleTable[cubeIndex][j]] * factor, 1.f);
        triangles[triangleId].p1 = Vector4f(vertList[triangleTable[cubeIndex][j + 1]] * factor, 1.f);
        triangles[triangleId].p2 = Vector4f(vertList[triangleTable[cubeIndex][j + 2]] * factor, 1.f);

        if (normals != nullptr) {
            Vector4f n = calcNormal(triangles[triangleId].p0, triangles[triangleId].p1,
                                    triangles[triangleId].p2);
            n.w = 1.f;
            normals[triangleId].n0 = n;
            normals[triangleId].n1 = n;
            normals[triangleId].n2 = n;
        }

        if (colors != nullptr) {
            colors[triangleId].c0 = color;
            colors[triangleId].c1 = color;
            colors[triangleId].c2 = color;
        }

    }
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void generateTrianglesLabel(uint i, ITMLib::ITMMesh::Triangle *triangles, ITMLib::ITMMesh::Normal *normals, ITMLib::ITMMesh::Color *colors, const Vector4f *labelColorList,
 float factor, int noMaxTriangles, float isoValue, uint *compactedVoxelArray, uint *numVertsScanned, const Vector4s *visibleBlockGlobalPos, const TVoxel *localVBA, const ITMHashEntry *hashTable, bool checkState)
{
    const uint& voxel = compactedVoxelArray[i];
    const Vector4s globalPos_4s = visibleBlockGlobalPos[voxel];

    if (globalPos_4s.w == 0) return;

    Vector3i globalPos = Vector3i(globalPos_4s.x, globalPos_4s.y, globalPos_4s.z);
    Vector3f points[8];
    int cubeIndex = buildVertListLabel<TVoxel>(points, globalPos, Vector3i(0,0,0), localVBA, hashTable, isoValue, checkState);
    if(cubeIndex<0)return;

    auto addPolygon = [&](int triangleId, int p1, int p2, int p3){
        if(triangleId >= noMaxTriangles) return;
        triangles[triangleId].p0 = Vector4f(points[p1] * factor, 1.f);
        triangles[triangleId].p1 = Vector4f(points[p2] * factor, 1.f);
        triangles[triangleId].p2 = Vector4f(points[p3] * factor, 1.f);

        if(normals != nullptr) {
            Vector4f n = calcNormal(triangles[triangleId].p0, triangles[triangleId].p1,
                                    triangles[triangleId].p2);
            n.w = 1.f;
            normals[triangleId].n0 = n;
            normals[triangleId].n1 = n;
            normals[triangleId].n2 = n;
        }

        if(colors != nullptr) {
            Vector4f color (1,1,1,1);
            colors[triangleId].c0 = color;
            colors[triangleId].c1 = color;
            colors[triangleId].c2 = color;

            if(TVoxel::hasLabelInformation && labelColorList != nullptr) {
                uint label = accessLabel<TVoxel>::readMaxWNeighborLabel(globalPos,localVBA,hashTable,1);
                color = labelColorList[label];
//                if(label == 0 ) color.w = 1.0;
                if(label == 0 ) color.w = 0.6;
                colors[triangleId].c0 = color;
                colors[triangleId].c1 = color;
                colors[triangleId].c2 = color;

            }
        }
    };
    // 12
    addPolygon(numVertsScanned[voxel], 0, 1,3);
    addPolygon(numVertsScanned[voxel]+1, 0, 3,2);
    addPolygon(numVertsScanned[voxel]+2, 4, 0,2);
    addPolygon(numVertsScanned[voxel]+3, 4, 2,6);
    addPolygon(numVertsScanned[voxel]+4, 5, 4,6);
    addPolygon(numVertsScanned[voxel]+5, 5, 6,7);
    addPolygon(numVertsScanned[voxel]+6, 1, 5,7);
    addPolygon(numVertsScanned[voxel]+7, 1, 7,3);
    addPolygon(numVertsScanned[voxel]+8, 2, 3,7);
    addPolygon(numVertsScanned[voxel]+9, 2, 7,6);
    addPolygon(numVertsScanned[voxel]+10, 1, 0,4);
    addPolygon(numVertsScanned[voxel]+11, 1, 4,5);
}

#if 0
template<SCFUSION::IntegrateType T, class TVoxel>
_CPU_AND_GPU_CODE_ inline void generateTriangles(uint i, ITMLib::ITMMesh::Triangle *triangles, ITMLib::ITMMesh::Normal *normals, ITMLib::ITMMesh::Color *colors, Vector4f *labelColorList,
                                                 float factor, int noMaxTriangles, float isoValue, uint *compactedVoxelArray, uint *numVertsScanned, const TVoxel *localVBA, const SCFUSION::SCVoxelArrayInfo *arrayInfo)
{
    const uint& voxel = compactedVoxelArray[i];
    short z = (short) floor(double(voxel / (arrayInfo->size.x * arrayInfo->size.y)));
    short y = (short) floor(double((voxel - (z * arrayInfo->size.x * arrayInfo->size.y)) / arrayInfo->size.x));
    short x = voxel - (z * arrayInfo->size.x * arrayInfo->size.y) - (y * arrayInfo->size.x);
    Vector3i globalPos = Vector3i(x, y, z);

    Vector3f vertList[12];
    int cubeIndex = buildVertList<T,TVoxel>(vertList, globalPos, localVBA, arrayInfo, isoValue);

    if (cubeIndex < 0) return;

    for (int j = 0; triangleTable[cubeIndex][j] != -1; j += 3)
    {
        int triangleId = numVertsScanned[voxel] + j/3;
        if (triangleId < noMaxTriangles)
        {
            triangles[triangleId].p0 = Vector4f(vertList[triangleTable[cubeIndex][j]] * factor, 1.f);
            triangles[triangleId].p1 = Vector4f(vertList[triangleTable[cubeIndex][j + 1]] * factor, 1.f);
            triangles[triangleId].p2 = Vector4f(vertList[triangleTable[cubeIndex][j + 2]] * factor, 1.f);

            if(normals != nullptr) {
                Vector4f n = calcNormal(triangles[triangleId].p0, triangles[triangleId].p1,
                                        triangles[triangleId].p2);
                n.w = 1.f;
                normals[triangleId].n0 = n;
                normals[triangleId].n1 = n;
                normals[triangleId].n2 = n;
            }

            if(colors != nullptr) {
                Vector4f color (1,1,1,1);
                colors[triangleId].c0 = color;
                colors[triangleId].c1 = color;
                colors[triangleId].c2 = color;

                if(TVoxel::hasLabelInformation && labelColorList != NULL) {
                    int vmIndex;
                    TVoxel voxel = readVoxel(localVBA, arrayInfo, globalPos, vmIndex);

                    if (!vmIndex){
                        continue;
                    }

                    color = labelColorList[accessLabel<TVoxel>::readLabel(voxel)];
                    colors[triangleId].c0 = color;
                    colors[triangleId].c1 = color;
                    colors[triangleId].c2 = color;
                }
            }
        }
    }
}
#endif

#if 1
template<SCFUSION::IntegrateType T>
_CPU_AND_GPU_CODE_ inline void generateTriangles(uint i, ITMLib::ITMMesh::Triangle *triangles, ITMLib::ITMMesh::Normal *normals, ITMLib::ITMMesh::Color *colors, const Vector4f *labelColorList,
                                                 float factor, int noMaxTriangles, float isoValue, uint *compactedVoxelArray, uint *numVertsScanned, const float *localVBA, const Vector3f &origin, const Vector3s &dims)
{
    const uint& voxel = compactedVoxelArray[i];
    short z = (short) floor(double(voxel / (dims.x * dims.y)));
    short y = (short) floor(double((voxel - (z * dims.x * dims.y)) / dims.x));
    short x = voxel - (z * dims.x * dims.y) - (y * dims.x);
    Vector3i globalPos = Vector3i(x, y, z);

    Vector3f vertList[12];
    int cubeIndex = buildVertList<T>(vertList, globalPos, localVBA, dims, isoValue);
    if (cubeIndex < 0) return;

    for (int j = 0; triangleTable[cubeIndex][j] != -1; j += 3)
    {
        int triangleId = numVertsScanned[voxel] + j/3;

        if (triangleId < noMaxTriangles)
        {
            triangles[triangleId].p0 = Vector4f(origin + vertList[triangleTable[cubeIndex][j]] * factor, 1.f);
            triangles[triangleId].p1 = Vector4f(origin + vertList[triangleTable[cubeIndex][j + 1]] * factor, 1.f);
            triangles[triangleId].p2 = Vector4f(origin + vertList[triangleTable[cubeIndex][j + 2]] * factor, 1.f);

            if(normals != nullptr) {
                Vector4f n = calcNormal(triangles[triangleId].p0, triangles[triangleId].p1, triangles[triangleId].p2);
                n.w = 1.f;
                normals[triangleId].n0 = n;
                normals[triangleId].n1 = n;
                normals[triangleId].n2 = n;
            }

            if(colors != nullptr) {
                Vector4f color (0,0,1,1);
//                color.w = 0.1;
                colors[triangleId].c0 = color;
                colors[triangleId].c1 = color;
                colors[triangleId].c2 = color;

                if(labelColorList != nullptr) {
                    color = labelColorList[(uint)readMaxWNeighborLabel(Vector3i(x,y,z), localVBA, dims, 1)];
                    color.w = 1;
                    colors[triangleId].c0 = color;
                    colors[triangleId].c1 = color;
                    colors[triangleId].c2 = color;
                }
            }
        }
    }
}
#endif