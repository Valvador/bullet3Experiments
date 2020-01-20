#include "stdafx.h"
#include "SpatialGrid.h"

namespace VSC
{
// GRID MODEL:
// GridID * voxWidth == Coord Center of Grid Cell
GridDesc::GridDesc(float _voxWidth, const Vector3* verts, size_t numVertices)
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

GridDesc::GridDesc(float _voxWidth) : voxWidth(_voxWidth)
{
	min = Vector3int32(INT_MAX);
	max = Vector3int32(-INT_MAX);
}

Vector3int32 GridDesc::coordToGrid(const Vector3& coord) const
{
	return Vector3int32((int)std::ceilf(coord.x / voxWidth - 0.5f), (int)std::ceilf(coord.y / voxWidth - 0.5f), (int)std::ceilf(coord.z / voxWidth - 0.5f));
}

Vector3 GridDesc::gridCenterToCoord(const Vector3int32& grid) const
{
	return Vector3(grid.x, grid.y, grid.z) * voxWidth;
}

bool GridDesc::withinGrid(const Vector3int32& id) const
{
	return id >= min && id <= max;
}

void GridDesc::minMaxCoordsOfGridVoxel(const Vector3int32& gridId, Vector3& min, Vector3& max) const
{
	Vector3 gridCoord((float)gridId.x, (float)gridId.y, (float)gridId.z);
	gridCoord *= voxWidth;
	float halfWidth = voxWidth * 0.5f;
	min = gridCoord - Vector3(halfWidth);
	max = gridCoord + Vector3(halfWidth);
}

void GridDesc::expandGridBy(const Vector3int32& expandBy)
{
	min = min - expandBy;
	max = max + expandBy;
}
}