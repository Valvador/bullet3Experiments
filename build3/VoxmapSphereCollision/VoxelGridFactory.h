#pragma once
#include "VoxelGrid.h"
#include <vector>

namespace VSC
{
// The intent of this class to make it so that we don't have to modify VoxelGrid structure on the fly.
// Anything that does work to generate/modify VoxelGrid exists here, in the factory.
// VoxelGrid itself should only be used for specific functions.
class VoxelGridFactory
{
public:
	// Assumes Stride of 12 bytes per vertex. numVertices implies # of 12 byte vertices. Assumes stride of 12 bytes per triangle.
	static VoxelGrid* generateVoxelGridFromMesh(const float* vertices, size_t numVertices, const size_t* indices, size_t numTriangles, float voxWidth);

	// Debug
	static void debug_MakeBoxVertexIndices(const Vector3& boxSize, const Vector3& boxOffset, std::vector<float>& vertices, std::vector<size_t>& indices);
private:
	// Helps the creation of Voxelmap Pointshells
	static void fillGridWithTriangleSurfaceVoxels(VoxelGrid* grid, const Vector3& v0, const Vector3& v1, const Vector3& v2);
	static void fillGridVoxelDistanceLayers(VoxelGrid* grid);
};
}; //namespace VSC