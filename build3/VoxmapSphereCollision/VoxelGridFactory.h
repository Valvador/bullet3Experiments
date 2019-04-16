#pragma once
#include "VoxelGrid.h"
#include <vector>

namespace VSC
{
class VoxelGridFactory
{
public:
	// Assumes Stride of 12 bytes per vertex. numVertices implies # of 12 byte vertices. Assumes stride of 12 bytes per triangle.
	static VoxelGrid* generateVoxelGridFromMesh(const float* vertices, size_t numVertices, const size_t* indices, size_t numTriangles, float voxWidth);

};
}; //namespace VSC