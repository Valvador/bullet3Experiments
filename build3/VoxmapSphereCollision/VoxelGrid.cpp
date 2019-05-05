#include "stdafx.h"
#include "VoxelGrid.h"
#include "Math/Geometry2D.h"
#include "Math/Geometry3D.h"
#include <math.h>
#include <vector>
#include <assert.h>

namespace VSC
{
VoxelGridDesc::VoxelGridDesc(float _voxWidth, const Vector3* verts, size_t numVertices)
	: voxWidth(_voxWidth)
{
	Vector3 minCoord = Vector3(FLT_MAX);
	Vector3 maxCoord = Vector3(-FLT_MAX);

	// Generate Bounding Box
	for (size_t i = 0; i < numVertices; i++)
	{
		const Vector3& vec = verts[i];
		minCoord.setMinAxis(vec);
		maxCoord.setMaxAxis(vec);
	}

	min = coordToGrid(minCoord);
	max = coordToGrid(maxCoord);
}

// GRID MODEL:
// GridID * voxWidth == Coord Center of Grid Cell
Vector3int32 VoxelGridDesc::coordToGrid(const Vector3& coord) const
{
	return Vector3int32((int)std::ceilf(coord.x / voxWidth - 0.5f), (int)std::ceilf(coord.y / voxWidth - 0.5f), (int)std::ceilf(coord.z / voxWidth - 0.5f));
}

Vector3 VoxelGridDesc::gridCenterToCoord(const Vector3int32& grid) const
{
	return Vector3(grid.x, grid.y, grid.z) * voxWidth;
}

bool VoxelGridDesc::withinGrid(const Vector3int32& id) const
{
	return id >= min && id <= max;
}

void VoxelGridDesc::minMaxCoordsOfGrid(const Vector3int32& gridId, Vector3& min, Vector3& max) const
{
	Vector3 gridCoord((float)gridId.x, (float)gridId.y, (float)gridId.z);
	gridCoord *= voxWidth;
	float halfWidth = voxWidth * 0.5f;
	min = gridCoord - Vector3(halfWidth);
	max = gridCoord + Vector3(halfWidth);
}

void VoxelGridDesc::expandGridBy(const Vector3int32& expandBy)
{
	min = min - expandBy;
	max = max + expandBy;
}

void VoxelGrid::setVoxel(const Vector3int32& pos, int32_t state)
{
	const int32_t* currentState = getVoxel(pos);
	if (currentState && *currentState == 0)
	{
		assert(surfaceCount);
		surfaceCount--;
	}
	if (state == 0)
	{
		surfaceCount++;
	}

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

}; //namespace VSC