#include "stdafx.h"
#include <assert.h>

#include "Math/Vector3.h"
#include "Math/Vector3int32.h"
#include "Math/Geometry2D.h"
#include "Math/Geometry3D.h"
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
			fillGridWithTriangleSurfaceVoxels(gridOut, v0, v1, v2);
		}
		fillGridVoxelDistanceLayers(gridOut);

		return gridOut;
	}

	void VoxelGridFactory::fillGridWithTriangleSurfaceVoxels(VoxelGrid* grid, const Vector3& v0, const Vector3& v1, const Vector3& v2)
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

		Vector3int32 minGrid = grid->getGridDescConst().coordToGrid(min);
		Vector3int32 maxGrid = grid->getGridDescConst().coordToGrid(max);

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
				grid->getGridDescConst().minMaxCoordsOfGrid(Vector3int32(x, y, minGrid.z), gridMin, gridMax);
				Vector2 aabbMin(gridMin.xy());
				Vector2 aabbMax(gridMax.xy());

				if (Geometry2D::lineSegmentsAABBIntersect(&edges[0], 3, aabbMin, aabbMax))
				{
					// We found an intersection, no we start at END and work our way back.
					// Once we find intersection from end, we can start traversing along Z
					// until we finish iteration.
					bool foundClosingIntersect = false;
					for (int32_t yBack = maxGrid.y; yBack >= y; yBack--)
					{
						if (!foundClosingIntersect)
						{
							Vector3 gridMin, gridMax;
							grid->getGridDescConst().minMaxCoordsOfGrid(Vector3int32(x, y, minGrid.z), gridMin, gridMax);
							Vector2 aabbMin(gridMin.xy());
							Vector2 aabbMax(gridMax.xy());
							foundClosingIntersect = Geometry2D::lineSegmentsAABBIntersect(&edges[0], 3, aabbMin, aabbMax);
						}

						if (foundClosingIntersect)
						{
							for (int32_t z = minGrid.z; z <= maxGrid.z; z++)
							{
								Vector3 gridMin, gridMax;
								Vector3int32 gridId = Vector3int32(x, yBack, z);
								grid->getGridDescConst().minMaxCoordsOfGrid(gridId, gridMin, gridMax);
								if (Geometry3D::triangleAABBIntersect(v0, v1, v2, gridMin, gridMax))
								{
									grid->setVoxel(gridId, 0);
								}
							}
						}
					}

					break;
				}
			}
		}
	} 

	static const int32_t OUTSIDE_MARKER = -std::numeric_limits<int32_t>::max();
	static void markOutsideInRange(VoxelGrid* grid, const Vector3int32& min, const Vector3int32& max)
	{
		for (int32_t x = min.x; x <= max.x; x++)
		{
			for (int32_t y = min.y; y <= max.y; y++)
			{
				for (int32_t z = min.z; z <= max.z; z++)
				{
					grid->setVoxel(Vector3int32(x, y, z), OUTSIDE_MARKER);
				}
			}
		}
	}



	static void floodFillRecursiveOutside(VoxelGrid* grid, const Vector3int32& startVoxel)
	{
		std::vector<Vector3int32> voxelsToTraverse;
		size_t currentVoxel = 0;

		// Resize to Max Possible (if everything was outside)
		Vector3int32 deltaGrid = grid->getGridDescConst().max - grid->getGridDescConst().min;
		voxelsToTraverse.reserve(deltaGrid.x * deltaGrid.y * deltaGrid.z * 26); // Accoutn for redundancy

		// Seed the start value;
		voxelsToTraverse.push_back(startVoxel);

		// Start "flat recursive" flood-fill
		while (currentVoxel < voxelsToTraverse.size())
		{
			// We check every ID here, we could probably do this earlier.
			const Vector3int32& id = voxelsToTraverse[currentVoxel];
			assert(grid->getGridDescConst().withinGrid(id));
			if (grid->getVoxel(id) != nullptr)
			{
				currentVoxel++;
				continue;
			}

			grid->setVoxel(id, OUTSIDE_MARKER);
			for (int32_t x = -1; x <= 1; x++)
			{
				for (int32_t y = -1; y <= 1; y++)
				{
					for (int32_t z = -1; z <= 1; z++)
					{
						if (x == y && y == z && x == (int32_t)0)
						{
							continue;
						}

						// Next grid ID
						Vector3int32 nextGridId = id + Vector3int32(x, y, z);
						if (grid->getGridDescConst().withinGrid(nextGridId))
						{
							const int32_t* voxel = grid->getVoxel(nextGridId);
							if (voxel == nullptr)
							{
								voxelsToTraverse.push_back(nextGridId);
							}
						}
					}
				}
			}
			currentVoxel++;
		}
	}

	static void scanLineCheckId(VoxelGrid* grid, const Vector3int32& id, bool& foundFirstSurface, int32_t& fillValue)
	{
		if (foundFirstSurface)
		{
			const int32_t* value = grid->getVoxel(id);
			bool canFill = false;
			bool outside = false;
			if (value)
			{
				int32_t currentValue = *value;
				if (currentValue != 0)
				{
					if (currentValue < 0)
					{
						outside = true;
						if ((fillValue - 1) > currentValue)
						{
							canFill = true;
						}
					}
					else
					{
						outside = false;
						if ((fillValue + 1) < currentValue)
						{
							canFill = true;
						}
					}
				}
				else
				{
					// Back to surface, reset out "distance" counter
					fillValue = 0;
					canFill = false;
				}
			}
			else
			{
				canFill = true;
				outside = false;
			}

			if (canFill)
			{
				if (outside)
				{
					grid->setVoxel(id, --fillValue);
				}
				else
				{
					grid->setVoxel(id, ++fillValue);
				}
			}
		}
		else
		{
			if (const int32_t* value = grid->getVoxel(id))
			{
				if (*value == 0) // Surface
				{
					foundFirstSurface = true;
				}
			}
		}
	}

	static void scanLineFillX(VoxelGrid* grid, int32_t y, int32_t z)
	{
		const Vector3int32& min = grid->getGridDescConst().min;
		const Vector3int32& max = grid->getGridDescConst().max;
		{
			// Positive scan
			bool foundFirstSurface = false;
			int32_t lastFillValue = 0;
			for (int32_t x = min.x; x <= max.x; x++)
			{
				Vector3int32 id = Vector3int32(x, y, z);
				scanLineCheckId(grid, id, foundFirstSurface, lastFillValue);
			}
		}
		{
			// Negative scan
			bool foundFirstSurface = false;
			int32_t lastFillValue = 0;
			for (int32_t x = max.x; x >= max.x; x--)
			{
				Vector3int32 id = Vector3int32(x, y, z);
				scanLineCheckId(grid, id, foundFirstSurface, lastFillValue);
			}
		}
	}

	static void scanLineFillY(VoxelGrid* grid, int32_t x, int32_t z)
	{
		const Vector3int32& min = grid->getGridDescConst().min;
		const Vector3int32& max = grid->getGridDescConst().max;
		{
			// Positive scan
			bool foundFirstSurface = false;
			int32_t lastFillValue = 0;
			for (int32_t y = min.y; y <= max.y; y++)
			{
				Vector3int32 id = Vector3int32(x, y, z);
				scanLineCheckId(grid, id, foundFirstSurface, lastFillValue);
			}
		}
		{
			// Negative scan
			bool foundFirstSurface = false;
			int32_t lastFillValue = 0;
			for (int32_t y = max.y; y >= max.y; y--)
			{
				Vector3int32 id = Vector3int32(x, y, z);
				scanLineCheckId(grid, id, foundFirstSurface, lastFillValue);
			}
		}
	}

	static void scanLineFillZ(VoxelGrid* grid, int32_t x, int32_t y)
	{
		const Vector3int32& min = grid->getGridDescConst().min;
		const Vector3int32& max = grid->getGridDescConst().max;
		{
			// Positive scan
			bool foundFirstSurface = false;
			int32_t lastFillValue = 0;
			for (int32_t z = min.z; z <= max.z; z++)
			{
				Vector3int32 id = Vector3int32(x, y, z);
				scanLineCheckId(grid, id, foundFirstSurface, lastFillValue);
			}
		}
		{
			// Negative scan
			bool foundFirstSurface = false;
			int32_t lastFillValue = 0;
			for (int32_t z = max.z; z >= max.z; z--)
			{
				Vector3int32 id = Vector3int32(x, y, z);
				scanLineCheckId(grid, id, foundFirstSurface, lastFillValue);
			}
		}
	}

	static void sixAxisScanlineFill(VoxelGrid* grid)
	{
		const Vector3int32& min = grid->getGridDescConst().min;
		const Vector3int32& max = grid->getGridDescConst().max;

		// X Axis +/- scan
		for (int32_t y = min.y; y <= max.y; y++)
		{
			for (int32_t z = min.z; z <= max.z; z++)
			{
				scanLineFillX(grid, y, z);
			}
		}

		// Y Axis +/- scan
		for (int32_t x = min.x; x <= max.x; x++)
		{
			for (int32_t z = min.z; z <= max.z; z++)
			{
				scanLineFillY(grid, x, z);
			}
		}

		// Z Axis +/- scan
		for (int32_t x = min.x; x <= max.x; x++)
		{
			for (int32_t y = min.y; y < max.y; y++)
			{
				scanLineFillZ(grid, x, y);
			}
		}
	}

	void VoxelGridFactory::fillGridVoxelDistanceLayers(VoxelGrid* grid)
	{
		assert(grid->countSurfaceVoxels() > 0);

		// Expand grid by 2 in every direction.
		Vector3int32 oldMin = grid->gridDesc.min;
		Vector3int32 oldMax = grid->gridDesc.max;
		grid->gridDesc.expandGridBy(Vector3int32(2)); 
		const Vector3int32 newMin = grid->gridDesc.min;
		const Vector3int32 newMax = grid->gridDesc.max;

		/*
		// // COMMENTING OUT BELOW SECTION BECAUSE IF I PRE-FILE OBVIOUS EMPTY SLOTS IT MESSES WITH THE "Flood Fill" algorithm being able
		// // to start with a single voxel.
		// After expansion, we know that the 2 voxel layer outside is all "outside"
		// so we must pre-process the "Outside"
		// We will have to look at 6 sides to mark these as "outside"
		markOutsideInRange(grid, newMin, Vector3int32(oldMin.x - 1, newMax.y, newMax.z)); // 1. From new X min to old X min
		markOutsideInRange(grid, Vector3int32(oldMax.x + 1, newMin.y, newMin.z), newMax); // 2. From old X max to new X max
		markOutsideInRange(grid, newMin, Vector3int32(newMax.x, oldMin.y - 1, newMax.z)); // 3. From new Y min to old Y min
		markOutsideInRange(grid, Vector3int32(newMin.x, oldMax.y + 1, newMin.z), newMax); // 4. From old Y max to new Y max
		markOutsideInRange(grid, newMin, Vector3int32(newMax.x, newMax.y, oldMin.z - 1)); // 5. From new Z min to old Z min
		markOutsideInRange(grid, Vector3int32(newMin.x, newMin.y, oldMax.z + 1), newMax); // 6. From old Z max to new Z max
		*/

		// Now we have to flood fill remaining "Outside" voxels, by traversing to 26 adjacent voxels from a start voxel.
		floodFillRecursiveOutside(grid, newMin);

		// Now we need to do a 6-axis scan to fill "Outside" values
		sixAxisScanlineFill(grid);
	}
}; //namespace VSC