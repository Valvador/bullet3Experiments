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
		MeshToGrid meshToGrid;
		meshToGrid.voxWidth = voxWidth;
		meshToGrid.min = Vector3(FLT_MAX);
		meshToGrid.max = Vector3(-FLT_MAX);

		// Generate Bounding Box
		for (size_t i = 0; i < numVertices; i++)
		{
			const Vector3& vec = verts[numVertices];
			meshToGrid.min.setMinAxis(vec);
			meshToGrid.max.setMaxAxis(vec);
		}

		// Go through Triangles, Colliding Against Grid

	}
}; //namespace VSC