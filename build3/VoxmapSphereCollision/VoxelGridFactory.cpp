#include "stdafx.h"
#include "Math/Vector3.h"
#include "Math/Vector3int32.h"
#include "VoxelGridFactory.h"

namespace VSC
{
	// Assumes Vertices have stride of 12 Bytes, 4 per axis 
	VoxelGrid* VoxelGridFactory::generateVoxelGridFromMesh(const float* vertices, size_t numVertices, const size_t* indices, size_t numTriangles, float voxWidth)
	{
		Vector3* verts = (Vector3*)vertices;
		VoxelGridDesc gridDesc(voxWidth, verts, numVertices);
		VoxelGrid* gridOut = new VoxelGrid(gridDesc);

		// Go through Triangles, Colliding Against Grid
		for (size_t i = 0; i < numTriangles; i++)
		{
			const Vector3& v0 = verts[indices[i * 3 + 0]];
			const Vector3& v1 = verts[indices[i * 3 + 1]];
			const Vector3& v2 = verts[indices[i * 3 + 2]];

			// Fill "Surface Voxels"
			gridOut->fillGridWithTriangleSurfaceVoxels(v0, v1, v2);
		}

		return gridOut;
	}
}; //namespace VSC