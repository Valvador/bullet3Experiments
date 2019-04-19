#include "stdafx.h"
#include "VoxelGrid.h"
#include "Math/Geometry2D.h"
#include <math.h>
#include <vector>

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

// GRID MODEL:
// GridID * voxWidth == Coord Center of Grid Cell
Vector3int32 VoxelGridDesc::coordToGrid(const Vector3& coord)
{
	return Vector3int32((int)std::ceilf(coord.x / voxWidth - 0.5f), (int)std::ceilf(coord.y / voxWidth - 0.5f), (int)std::ceilf(coord.z / voxWidth - 0.5f));
}

void VoxelGridDesc::minMaxCoordsOfGrid(const Vector3int32& gridId, Vector3& min, Vector3& max)
{
	Vector3 gridCoord((float)gridId.x, (float)gridId.y, (float)gridId.z);
	gridCoord *= voxWidth;
	float halfWidth = voxWidth * 0.5f;
	min = gridCoord - Vector3(halfWidth);
	max = gridCoord + Vector3(halfWidth);
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

	Geometry2D::LineSegment2D edges[3] = { 
		Geometry2D::LineSegment2D(v0.xy(), v1.xy()), 
		Geometry2D::LineSegment2D(v0.xy(), v2.xy()), 
		Geometry2D::LineSegment2D(v1.xy(), v2.xy()) 
	};

	// Scan the XY side of grid to check for Triangle Projection
	for (int32_t x = minGrid.x; x <= maxGrid.x; x++)
	{
		for (int32_t y = minGrid.y; y <= maxGrid.y; y++)
		{
			Vector3 gridMin, gridMax;
			gridDesc.minMaxCoordsOfGrid(Vector3int32(x, y, minGrid.z), gridMin, gridMax);
			Vector2 aabbMin(gridMin.xy());
			Vector2 aabbMax(gridMax.xy());

			if (Geometry2D::lineSegmentsAABBIntersect(&edges[0], 3, aabbMin, aabbMax))
			{
				// We found an intersection, no we start at END and work our way back.
				// Once we find intersection from end, we can start traversing along Z
				// until we finish iteration.
				float foundClosingIntersect = false;
				for (int32_t yBack = maxGrid.y; yBack >= y; y--)
				{
					if (!foundClosingIntersect)
					{
						Vector3 gridMin, gridMax;
						gridDesc.minMaxCoordsOfGrid(Vector3int32(x, y, minGrid.z), gridMin, gridMax);
						Vector2 aabbMin(gridMin.xy());
						Vector2 aabbMax(gridMax.xy());
						foundClosingIntersect = Geometry2D::lineSegmentsAABBIntersect(&edges[0], 3, aabbMin, aabbMax);
					}

					if (foundClosingIntersect)
					{
						// Z TRAVERSAL + 3D VOXEL TRIANGLE COLLISION GOES HERE
					}
				}

				break;
			}
		}
	}
}

}; //namespace VSC