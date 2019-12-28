#pragma once
#include "VoxelGrid.h"
#include "SphereTree.h"
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
	static VoxelGridDistanceField* generateDistanceFieldFromMeshAndVoxelGrid(const SparseGrid<Vector3>& surfaceProjection, const SparseGrid<Vector3>& gradientGrid, const VoxelGrid* voxelGrid);
	static SphereTree* generateSphereTreeFromSurfaceProjections(const SparseGrid<Vector3>& surfaceProjection);
	
	// Visualization Requests for now
	static SparseGrid<Vector3> getVoxelGridGradient(const VoxelGrid* voxelGrid);
	static SparseGrid<Vector3> getSurfaceProjection(const SparseGrid<Vector3>& gradientGrid,
		const float* vertices, size_t numVertices, const size_t* indices, size_t numTriangles, float voxWidth, const VoxelGrid* voxelGrid);

	// Debug
	static void debug_MakeBoxVertexIndices(const Vector3& boxSize, const Vector3& boxOffset, std::vector<float>& vertices, std::vector<size_t>& indices);
private:
	// Helpers
	// // VoxelGrid helpers
	static void fillGridWithTriangleSurfaceVoxels(VoxelGrid* grid, const Vector3& v0, const Vector3& v1, const Vector3& v2);
	static void fillGridVoxelDistanceLayers(VoxelGrid* grid);

	// // Intermediate Structure Helpers (For Distance Fields and Sphere Shells)
	// // // Populates 'gradientGrid', which contains gradient value per voxel from from negative to positive "depths" of object
	static void generateVoxelGridGradient(SparseGrid<Vector3>& gradientGrid, const VoxelGrid* voxelGrid);
	// // // Populates 'surfaceProjection', which contains the outer-most triangle projection in the surface voxels
	static void generateSurfaceProjectionPoints(SparseGrid<Vector3>& surfaceProjection, const SparseGrid<Vector3>& gradientGrid,
		const float* vertices, size_t numVertices, const size_t* indices, size_t numTriangles, float voxWidth, const VoxelGrid* voxelGrid);
	static void findIntermediateClosestSurfacePoints(
		SparseGrid<Vector3>& surfaceProjection, const SparseGrid<Vector3>& gradientGrid, const Vector3int32& voxelId, const VoxelGrid* voxelGrid,
		const Vector3& v0, const Vector3& v1, const Vector3& v2);

	// // VoxelGridDistanceField helpers
	// // // Populates 'distanceFieldOut' for the surface. Uses gradient to figure out which direction to project the distance.
	static void fillDistanceFieldSurfaceValues(VoxelGridDistanceField* distanceFieldOut, const SparseGrid<Vector3>& surfaceProjection, const SparseGrid<Vector3>& gradientGrid, const VoxelGridDesc& gridDesc);
	// // // Populates 'distanceFieldOut' along the gradient line starting at 'startId' until we hit an existing 'distanceField' value.
	static void fillDistanceFieldAlongGradientLine(
		VoxelGridDistanceField* distanceFieldOut, const Vector3int32& startId, const SparseGrid<Vector3>& surfaceProjection, 
		const SparseGrid<Vector3>& gradientGrid, const VoxelGrid* voxelGrid);
};
}; //namespace VSC