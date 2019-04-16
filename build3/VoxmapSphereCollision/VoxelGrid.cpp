#include "stdafx.h"
#include "VoxelGrid.h"

namespace VSC
{
VoxelGridDesc::VoxelGridDesc(float _voxWidth, const Vector3* verts, size_t numVertices)
	: voxWidth(_voxWidth)
{
	min = Vector3(FLT_MAX);
	max = Vector3(-FLT_MAX);

	// Generate Bounding Box
	for (size_t i = 0; i < numVertices; i++)
	{
		const Vector3& vec = verts[numVertices];
		min.setMinAxis(vec);
		max.setMaxAxis(vec);
	}
}

void VoxelGrid::setVoxel(const Vector3int32& pos, int32_t state)
{
	insertAt(pos, state);
}

const int32_t* VoxelGrid::getVoxel(const Vector3int32& pos) const
{
	return getAt(pos);
}

void VoxelGrid::clearVoxel(const Vector3int32& pos)
{
	eraseAt(pos);
}

void VoxelGrid::fillGridWithTriangleSurfaceVoxels(const Vector3& v0, const Vector3& v1, const Vector3& v2)
{
	// IMPLEMENT NOW
}

}; //namespace VSC