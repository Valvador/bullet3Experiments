#include "stdafx.h"
#include "VoxelGrid.h"
#include <math.h>

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

Vector3int32 VoxelGridDesc::coordToGrid(const Vector3& coord)
{
	// May not be a symmetric grid
	// What does Vector3int32(0,0,0) mean in this case?
	return Vector3int32((int)std::ceilf(coord.x / voxWidth), (int)std::ceilf(coord.y / voxWidth), (int)std::ceilf(coord.z / voxWidth));
}

void VoxelGridDesc::minMaxCoordsOfGrid(const Vector3int32& gridId, Vector3& min, Vector3& max)
{
	// Converting gridId to coord gives "most extreme" corner. (Furthest from Origin)
	// We want min - max as output.
	Vector3 extreme(gridId.x, gridId.y, gridId.z);
	extreme *= voxWidth;
	// TODO - NEED TO FIGURE OUT MY COORDINATE SYSTEM FOR GRID FIRST.
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
	// Find Triangle Bounding Box
	Vector3 min = Vector3(FLT_MAX);
	Vector3 max = Vector3(-FLT_MAX);
	min.setMinAxis(v0);
	min.setMinAxis(v1);
	min.setMinAxis(v2);
	max.setMaxAxis(v0);
	max.setMaxAxis(v1);
	max.setMaxAxis(v2);
	
	Vector3int32 minGrid = gridDesc.coordToGrid(min);
	Vector3int32 maxGrid = gridDesc.coordToGrid(max);

	// Scan the X-Z side of grid to check for Triangle Projection
	for (int32_t x = minGrid.x; x <= maxGrid.x; x++)
	{
		for (int32_t y = minGrid.y; y <= maxGrid.y; y++)
		{

		}
	}
}

}; //namespace VSC