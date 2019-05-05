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

	// Debug
	void VoxelGridFactory::debug_MakeBoxVertexIndices(const Vector3& boxSize, const Vector3& boxOffset, std::vector<float>& vertices, std::vector<size_t>& indices)
	{
		size_t startIndicesSize = indices.size();
		size_t startingVertSize = vertices.size() / 3;
		Vector3 halfSize = boxSize * 0.5f;
		vertices.reserve(3 * 8); // 8 vertices
		indices.reserve(3 * 12); // 12 Triangles

		for (int x = -1; x <= 1; x += 2)
		{
			for (int y = -1; y <= 1; y += 2)
			{
				for (int z = -1; z <= 1; z += 2)
				{
					Vector3 vertex = boxOffset + halfSize * Vector3(x, y, z);
					vertices.push_back(vertex.x);
					vertices.push_back(vertex.y);
					vertices.push_back(vertex.z);
				}
			}
		}
		// 0 = -1, -1, -1
		// 1 = -1, -1,  1
		// 2 = -1,  1, -1
		// 3 = -1,  1,  1
		// 4 =  1, -1, -1
		// 5 =  1, -1,  1
		// 6 =  1,  1, -1
		// 7 =  1,  1,  1
		/*
		  3-----7
		 /|    /|
		2-----6 |
		| 1---|-5
		|/    |/
		0-----4
		*/
		assert((vertices.size() - startingVertSize * 3) == 3 * 8);
		// 12 Triangles for 6 Faces
		// Front Face
		indices.push_back(startingVertSize + 0); indices.push_back(startingVertSize + 4); indices.push_back(startingVertSize + 2);
		indices.push_back(startingVertSize + 4); indices.push_back(startingVertSize + 6); indices.push_back(startingVertSize + 2);
		// Right Face
		indices.push_back(startingVertSize + 4); indices.push_back(startingVertSize + 5); indices.push_back(startingVertSize + 6);
		indices.push_back(startingVertSize + 5); indices.push_back(startingVertSize + 7); indices.push_back(startingVertSize + 6);
		// Top Face
		indices.push_back(startingVertSize + 2); indices.push_back(startingVertSize + 6); indices.push_back(startingVertSize + 3);
		indices.push_back(startingVertSize + 6); indices.push_back(startingVertSize + 7); indices.push_back(startingVertSize + 3);
		// Left Face
		indices.push_back(startingVertSize + 3); indices.push_back(startingVertSize + 1); indices.push_back(startingVertSize + 0);
		indices.push_back(startingVertSize + 3); indices.push_back(startingVertSize + 0); indices.push_back(startingVertSize + 2);
		// Back Face
		indices.push_back(startingVertSize + 3); indices.push_back(startingVertSize + 7); indices.push_back(startingVertSize + 1);
		indices.push_back(startingVertSize + 7); indices.push_back(startingVertSize + 5); indices.push_back(startingVertSize + 1);
		// Bottom Face
		indices.push_back(startingVertSize + 5); indices.push_back(startingVertSize + 0); indices.push_back(startingVertSize + 1);
		indices.push_back(startingVertSize + 5); indices.push_back(startingVertSize + 4); indices.push_back(startingVertSize + 0);
		assert((indices.size() - startIndicesSize) == 3 * 12);
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

	static void fillIdUsingAdjacentData(VoxelGrid* grid, const Vector3int32& id)
	{
		int32_t largestValue = OUTSIDE_MARKER;
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

					if (const int32_t* value = grid->getVoxel(id + Vector3int32(x, y, z)))
					{
						if (*value > largestValue)
						{
							largestValue = *value;
						}
					}
				}
			}
		}

		grid->setVoxel(id, --largestValue);
	}

	static void fillCornerPocketsUsingAdjacentData(VoxelGrid* grid, const Vector3int32& start, const Vector3int32& end)
	{
		// Can no longer assume start and end are min, max. Have to check if each axis
		Vector3int32 delta = end - start;
		bool xPositive = delta.x >= 0;
		bool yPositive = delta.y >= 0;
		bool zPositive = delta.z >= 0;
		for (int32_t x = start.x; xPositive ? x <= end.x : x >= end.x; xPositive ? x++ : x--)
		{
			for (int32_t y = start.y; yPositive ? y <= end.y : y >= end.y; yPositive ? y++ : y--)
			{
				for (int32_t z = start.z; zPositive ? z <= end.z : z >= end.z; zPositive ? z++ : z--)
				{
					// We can assert the below boolean check if we want to optimize this scan. If the else
					// condition is ever called, it means we are scanning more voxels than we need to.
					if (grid->getVoxel(Vector3int32(x, y, z)) && *grid->getVoxel(Vector3int32(x, y, z)) == OUTSIDE_MARKER)
						fillIdUsingAdjacentData(grid, Vector3int32(x, y, z));
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
					fillValue = 0;
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
			for (int32_t x = max.x; x >= min.x; x--)
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
			for (int32_t y = max.y; y >= min.y; y--)
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
			for (int32_t z = max.z; z >= min.z; z--)
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

		// Now we have to flood fill "Outside" voxels, by traversing to 26 adjacent voxels from a start voxel.
		floodFillRecursiveOutside(grid, newMin);

		// Now we need to do a 6-axis scan to fill "Outside" values
		sixAxisScanlineFill(grid);

		// Fill added Corners, the scanline fill algorithm misses the corner pockets of what we added with
		// expandGridBy earlier in the function. To fix this, we need to manually check these from the inner-most
		// corner going out. 8 corners total
		Vector3int32 oldMinStart = oldMin - Vector3int32(1, 1, 1);
		Vector3int32 oldMaxStart = oldMax + Vector3int32(1, 1, 1);
		fillCornerPocketsUsingAdjacentData(grid, oldMinStart, newMin); // 1  [-, -, -]
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMaxStart.x, oldMinStart.y, oldMinStart.z), Vector3int32(newMax.x, newMin.y, newMin.z)); //2 [+, -, -]
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMaxStart.x, oldMaxStart.y, oldMinStart.z), Vector3int32(newMax.x, newMax.y, newMin.z)); //3 [+, +, -]
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMinStart.x, oldMinStart.y, oldMaxStart.z), Vector3int32(newMin.x, newMin.y, newMax.z)); //4 [-, -, +] 
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMinStart.x, oldMaxStart.y, oldMaxStart.z), Vector3int32(newMin.x, newMax.y, newMax.z)); //5 [-, +, +] 
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMinStart.x, oldMaxStart.y, oldMinStart.z), Vector3int32(newMin.x, newMax.y, newMin.z)); //6 [-, +, -] 
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMaxStart.x, oldMinStart.y, oldMaxStart.z), Vector3int32(newMax.x, newMin.y, newMax.z)); //7 [+, -, +]
		fillCornerPocketsUsingAdjacentData(grid, oldMaxStart, newMax); //8 [+, +, +]
		
		// Now the 12 Edges between them.
		// Front Face [ x, x, -]
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMin.x, oldMin.y, oldMin.z), Vector3int32(oldMax.x, newMin.y, newMin.z)); // 1 - 2
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMax.x, oldMin.y, oldMin.z), Vector3int32(newMax.x, oldMax.y, newMin.z)); // 2 - 3
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMin.x, oldMin.y, oldMin.z), Vector3int32(newMin.x, oldMax.y, newMin.z)); // 1 - 6
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMin.x, oldMax.y, oldMin.z), Vector3int32(oldMax.x, newMax.y, newMin.z)); // 6 - 3
		
		// Back Face [ x, x, +]
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMin.x, oldMin.y, oldMax.z), Vector3int32(oldMax.x, newMin.y, newMax.z)); // 4 - 7
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMax.x, oldMin.y, oldMax.z), Vector3int32(newMax.x, oldMax.y, newMax.z)); // 7 - 8
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMin.x, oldMax.y, oldMax.z), Vector3int32(oldMax.x, newMax.y, newMax.z)); // 5 - 8
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMin.x, oldMin.y, oldMax.z), Vector3int32(newMin.x, oldMax.y, newMax.z)); // 4 - 5

		// Merging Faces
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMin.x, oldMin.y, oldMin.z), Vector3int32(newMin.x, newMin.y, oldMax.z));									  // 1 - 4
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMax.x, oldMax.y, oldMin.z), Vector3int32(newMax.x, newMax.y, oldMax.z)); // 3 - 8
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMax.x, oldMin.y, oldMin.z), Vector3int32(newMax.x, newMin.y, oldMax.z)); // 2 - 7
		fillCornerPocketsUsingAdjacentData(grid, Vector3int32(oldMin.x, oldMax.y, oldMin.z), Vector3int32(newMin.x, newMax.y, oldMax.z)); // 6 - 5
	}
}; //namespace VSC