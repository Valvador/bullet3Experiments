#include "stdafx.h"
#include <assert.h>
#include <stack>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <functional>

#include "Math/Vector3.h"
#include "Math/Vector3int32.h"
#include "Math/Geometry2D.h"
#include "Math/Geometry3D.h"
#include "VoxelGridFactory.h"


namespace VSC
{
// Assumes Vertices have stride of 12 Bytes, 4 per axis 
VoxelGrid* VoxelGridFactory::generateVoxelGridFromMesh(const float* vertices, size_t numVertices, const size_t* indices, size_t numTriangles, float voxWidth, size_t expandBy)
{
	Vector3* verts = (Vector3*)vertices;
	GridDesc gridDesc(voxWidth, verts, numVertices);
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
	fillGridVoxelDistanceLayers(gridOut, expandBy);

	return gridOut;
}

// Brute forcish method that finds "0" voxels in the grid, which are surface voxels, and then finds any triangles that 
// are part of this voxel grid.
// vgorbunov TODO - We need to TEST this method.
VoxelGridDistanceField* VoxelGridFactory::generateDistanceFieldFromMeshAndVoxelGrid(
	const SparseGrid<Vector3>& surfaceProjection, const SparseGrid<Vector3>& gradientGrid, const VoxelGrid* voxelGrid)
{
	// surfaceProjection and gradientGrid should be pre-computed before using this.
	assert(surfaceProjection.countVoxels() != 0);
	assert(gradientGrid.countVoxels() != 0);
	VoxelGridDistanceField* resultDistanceField = new VoxelGridDistanceField();

	// We now have EVERY surface voxel with its closest point projection inside of surfaceProjections
	// We now use this information to generate distance fields for every voxel in the grid...
	
	
	// Start with Surface and generate the first distance field entries on the surface voxels.
	// We will use these as "termination" points for the gradient traversal
	fillDistanceFieldSurfaceValues(resultDistanceField, surfaceProjection, gradientGrid, voxelGrid->getGridDescConst());

	// Now, we traverse the non-surface points along the gradient lines until we hit the surface points calculated above.
	// We use the values in the distance field calculated above to properly populate the rest of the non-surface distance values.
	const Vector3int32& min = voxelGrid->getGridDescConst().min;
	const Vector3int32& max = voxelGrid->getGridDescConst().max;
	for (int32_t x = min.x; x <= max.x; x++)
	{
		for (int32_t y = min.y; y <= max.y; y++)
		{
			for (int32_t z = min.z; z <= max.z; z++)
			{
				const Vector3int32& gridId = Vector3int32(x, y, z);
				if (const float* distanceValue = resultDistanceField->getVoxel(gridId))
				{
					continue;
				}

				fillDistanceFieldAlongGradientLine(resultDistanceField, gridId, surfaceProjection, gradientGrid, voxelGrid);
			}
		}
	}

	return resultDistanceField;
}

struct Vector3Visited
{
	bool visited;
	Vector3 v;
	SphereTreeNode<SphereTree::SphereTreeNodeMax>* node;

	Vector3Visited(const Vector3& pos, SphereTreeNode<SphereTree::SphereTreeNodeMax>* treeNode) : visited(false), v(pos), node(treeNode) {};
};

Vector3int32 SpatialSparseGrid<Vector3Visited>::getGridId(const Vector3Visited& v) const
{
	return desc.coordToGrid(v.v);
}

void sortByDistance(std::vector<Vector3Visited*>& points, const Vector3Visited& target)
{
	struct SortDistance
	{
		Vector3 target;

		SortDistance(const Vector3Visited& _target) : target(_target.v) {}

		bool operator() (const Vector3Visited* l, const Vector3Visited* r)
		{
			return (l->v - target).sqrMagnitude() < (r->v - target).sqrMagnitude();
		}
	} sortFromTarget(target);

	std::sort(points.begin(), points.end(), sortFromTarget);
}

Vector3Visited* getNextUnvisitedPoint(std::vector<Vector3Visited>& points)
{
	for (auto& point : points)
	{
		if (!point.visited)
		{
			return &point;
		}
	}
	return nullptr;
}

bool debug_allPointsVisited(std::vector<Vector3Visited>& points)
{
	for (auto& point : points)
	{
		if (point.visited == false)
		{
			return false;
		}
	}

	return true;
}

#define ALLOW_POINT_HOP
#define ALLOW_OVERFLOW_BACKFILL // Sometimes we leave individual notes without children due to the filling process. This lets us hold on to orphaned nodes and refil.
const float leafNodeRadius = 0.1f; // TODO <- THIS SHOULD ACTUALLY DEPEND ON THE VOXEL WIDTH
// Generates Sphere Tree, final piece of the Voxelmap Pointshell Algorithm
SphereTree* VoxelGridFactory::generateSphereTreeFromSurfaceProjections(const SparseGrid<Vector3>& surfaceProjection)
{
	std::vector<Vector3Visited> rootPoints; // to keep track of points and sort them to help creation of nodes.
	std::vector<SphereTreeNode<SphereTree::SphereTreeNodeMax>*> rootNodes;
	std::vector<SphereTreeNode<SphereTree::SphereTreeNodeMax>*> nextRootNodes;

	rootPoints.reserve(surfaceProjection.countVoxels());
	rootNodes.reserve(surfaceProjection.countVoxels());
	// Create the original leaf nodes that have no children;
	for (auto it = surfaceProjection.cbegin(); it != surfaceProjection.cend(); it++)
	{
		SphereTreeNode<SphereTree::SphereTreeNodeMax>* leafNode = new SphereTreeNode<SphereTree::SphereTreeNodeMax>(leafNodeRadius, it->second);
		rootPoints.emplace_back(Vector3Visited(it->second, leafNode)); // Vector3Visited( visited, position, treeNode)
		rootNodes.push_back(leafNode);
	}

	// Main loop, has to end when we only have 1 top point.
	std::vector<Vector3Visited*> orphanedNodes;
	std::vector<Vector3Visited*> broadphaseBuffer;
	while (rootNodes.size() > 1)
	{
		assert(rootNodes.size() == rootPoints.size());

		// Find Bounds of current Root Point
		Vector3 min = FLT_MAX;
		Vector3 max = -FLT_MAX;
		for (const auto& point : rootPoints)
		{
			min.setMinAxis(point.v);
			max.setMaxAxis(point.v);
		}
		double volume = (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
		float gridWidth = std::cbrt(volume) / sqrt(rootPoints.size());
		// Generate a Spatial Grid for the current set of Root Points
		SpatialSparseGrid<Vector3Visited> spatialGrid(gridWidth);
		for (auto& point : rootPoints)
		{
			spatialGrid.addToSpatialGrid(&point);
		}
		// Find point to start a new cluster
		orphanedNodes.clear();
		while (Vector3Visited* currentTarget = getNextUnvisitedPoint(rootPoints))
		{
			broadphaseBuffer.clear();
			int numBroadPhasePointsToFind = SphereTree::SphereTreeNodeGoal * 4;
			spatialGrid.gatherNClosestToTarget(*currentTarget, broadphaseBuffer, numBroadPhasePointsToFind);
			sortByDistance(broadphaseBuffer, *currentTarget);

			assert(broadphaseBuffer[0] == currentTarget);
			SphereTreeNode<SphereTree::SphereTreeNodeMax>* futureRoot = new SphereTreeNode<SphereTree::SphereTreeNodeMax>();
			futureRoot->addChild(currentTarget->node);
			futureRoot->setPrimaryNode(0);
			currentTarget->visited = true;

			int index = 0;
			for (int i = 1; i < SphereTree::SphereTreeNodeGoal && index < broadphaseBuffer.size(); i++)
			{
				Vector3Visited& newChild = *broadphaseBuffer[index];
				// If next closest point is visisted, we are probably hopping over objects
				// should stop here.
				if (newChild.visited)
				{
#ifdef ALLOW_POINT_HOP
					i--;
					index++;
					continue;
#else
					break
#endif
				}

				futureRoot->addChild(newChild.node);
				newChild.visited = true;
				index++;
			}

#ifdef ALLOW_OVERFLOW_BACKFILL
			if (futureRoot->getNumChildren() == 1)
			{
				// Collect points that could not find other nearby unassigned children
				orphanedNodes.push_back(currentTarget);
				// This prevents destructor from killing the child nodes with the root
				// by disassociating the orphaned node from futureRoot
				futureRoot->resetChildren(); 
				delete futureRoot;
			}
			else
			{
				futureRoot->computeRadius();
				nextRootNodes.push_back(futureRoot);
			}
#else
			futureRoot->computeRadius();
			nextRootNodes.push_back(futureRoot);
#endif
		}

#ifdef ALLOW_OVERFLOW_BACKFILL
		// Go through nodes that weren't assigned to groups and force them into a few "overflow" groups.
		for (auto soloNode : orphanedNodes)
		{
			broadphaseBuffer.clear();
			int numBroadPhasePointsToFind = SphereTree::SphereTreeNodeGoal * 4;
			spatialGrid.gatherNClosestToTarget(*soloNode, broadphaseBuffer, numBroadPhasePointsToFind);
			sortByDistance(broadphaseBuffer, *soloNode);

			for (size_t i = 0; i < broadphaseBuffer.size(); i++)
			{
				Vector3Visited& newChild = *broadphaseBuffer[i];
				// This time we do look specifically FOR visited nodes.
				if ((&newChild != soloNode) && newChild.visited)
				{
					SphereTreeNode<SphereTree::SphereTreeNodeMax>* overflowRoot = static_cast<SphereTreeNode<SphereTree::SphereTreeNodeMax>*>(newChild.node->getParent());
					if (overflowRoot->getNumChildren() < SphereTree::SphereTreeNodeMax)
					{
						overflowRoot->addChild(soloNode->node);
						overflowRoot->computeRadius();
						break;
					}
				}
			}
		}
#endif

		assert(debug_allPointsVisited(rootPoints));
		rootNodes.clear();
		rootPoints.clear();
		for (auto newRoot : nextRootNodes)
		{
			rootPoints.emplace_back(Vector3Visited(newRoot->getPosition(), newRoot)); // Vector3Visited( visited, position, treeNode)
			rootNodes.push_back(newRoot);
		}
		nextRootNodes.clear();
	}

	assert(rootNodes.size() == 1);
	return new SphereTree(rootNodes[0]);
}

SparseGrid<Vector3> VoxelGridFactory::getVoxelGridGradient(const VoxelGrid* voxelGrid)
{
	SparseGrid<Vector3> result;
	generateVoxelGridGradient(result, voxelGrid);
	return result;
}

SparseGrid<Vector3> VoxelGridFactory::getSurfaceProjection(const SparseGrid<Vector3>& gradientGrid,
	const float* vertices, size_t numVertices, const size_t* indices, size_t numTriangles, float voxWidth, const VoxelGrid* voxelGrid)
{
	SparseGrid<Vector3> result;
	generateSurfaceProjectionPoints(result, gradientGrid, vertices, numVertices, indices, numTriangles, voxWidth, voxelGrid);
	return result;
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
			grid->getGridDescConst().minMaxCoordsOfGridVoxel(Vector3int32(x, y, minGrid.z), gridMin, gridMax);
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
						grid->getGridDescConst().minMaxCoordsOfGridVoxel(Vector3int32(x, y, minGrid.z), gridMin, gridMax);
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
							grid->getGridDescConst().minMaxCoordsOfGridVoxel(gridId, gridMin, gridMax);
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

void VoxelGridFactory::fillGridVoxelDistanceLayers(VoxelGrid* grid, size_t expandBy)
{
	assert(grid->countSurfaceVoxels() > 0);

	// Expand grid by 2 in every direction.
	Vector3int32 oldMin = grid->gridDesc.min;
	Vector3int32 oldMax = grid->gridDesc.max;
	grid->gridDesc.expandGridBy(Vector3int32(expandBy));
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

Vector3 computeVoxelGradient(const VoxelGrid* voxelGrid, const Vector3int32& gridId)
{
	Vector3 result(0);
	int count = 0;

	const int32_t& centerValue = *voxelGrid->getVoxel(gridId);
	for (int32_t x = -1; x <= 1; x++)
	{
		for (int32_t y = -1; y <= 1; y++)
		{
			for (int32_t z = -1; z <= 1; z++)
			{
				if (x == y && x == z && x == 0)
					continue;

				Vector3int32 offsetId = gridId + Vector3int32(x, y, z);
				if (const int32_t* value = voxelGrid->getVoxel(offsetId))
				{
					int32_t delta = *value - centerValue;
					Vector3 deltaDir(x, y, z);
					deltaDir = deltaDir.normalized();

					result += deltaDir * (float)delta;
					count++;
				}
			}
		}
	}

	if (count)
		return result * (1 /((float)count));
		
	return Vector3(0);
}

void VoxelGridFactory::generateSurfaceProjectionPoints(SparseGrid<Vector3>& surfaceProjection, const SparseGrid<Vector3>& gradientGrid,
	const float* vertices, size_t numVertices, const size_t* indices, size_t numTriangles, float voxWidth, const VoxelGrid* voxelGrid)
{
	assert(surfaceProjection.countVoxels() == 0);
	Vector3* verts = (Vector3*)vertices;

	// Go through Grid, looking for surface voxels.
	const Vector3int32& min = voxelGrid->getGridDescConst().min;
	const Vector3int32& max = voxelGrid->getGridDescConst().max;
	for (int32_t x = min.x; x <= max.x; x++)
	{
		for (int32_t y = min.y; y <= max.y; y++)
		{
			for (int32_t z = min.z; z <= max.z; z++)
			{
				const Vector3int32& gridId = Vector3int32(x, y, z);
				if (const int32_t* voxel = voxelGrid->getVoxel(gridId))
				{
					if (*voxel == 0)
					{
						// Find which triangles collide with Voxel, and store the best projection of
						// voxel center on triangle surface in surfaceProjections.
						for (size_t i = 0; i < numTriangles; i++)
						{
							const Vector3& v0 = verts[indices[i * 3 + 0]];
							const Vector3& v1 = verts[indices[i * 3 + 1]];
							const Vector3& v2 = verts[indices[i * 3 + 2]];

							findIntermediateClosestSurfacePoints(surfaceProjection, gradientGrid, gridId, voxelGrid, v0, v1, v2);
						}
					}
				}
			}
		}
	}
}

void VoxelGridFactory::generateVoxelGridGradient(SparseGrid<Vector3>& gradientGrid, const VoxelGrid* voxelGrid)
{
	const Vector3int32& gridMin = voxelGrid->getGridDescConst().min;
	const Vector3int32& gridMax = voxelGrid->getGridDescConst().max;

	for (int32_t x = gridMin.x; x <= gridMax.x; x++)
	{
		for (int32_t y = gridMin.y; y <= gridMax.y; y++)
		{
			for (int32_t z = gridMin.z; z <= gridMax.z; z++)
			{
				Vector3int32 gridId(x, y, z);
				gradientGrid.insertAt(gridId, computeVoxelGradient(voxelGrid, gridId));
			}
		}
	}
}

void updateBestAdjacentTriangleProjectionDistance(
	float& existingBestDistanceSqr, float& newBestDistanceSqr, const VoxelGrid* voxelGrid, const Vector3int32& offsetId,
	const Vector3& currentProjection, const Vector3& newProjection)
{
	if (const int32_t* adjacentVoxel = voxelGrid->getVoxel(offsetId))
	{
		if (*adjacentVoxel < 0)
		{
			Vector3 adjacentCoord = voxelGrid->getGridDescConst().gridCenterToCoord(offsetId);
			float existingCoordDistSqr = (adjacentCoord - currentProjection).sqrMagnitude();
			float newCoordDistSqr = (adjacentCoord - newProjection).sqrMagnitude();

			if (existingCoordDistSqr < existingBestDistanceSqr)
			{
				existingBestDistanceSqr = existingCoordDistSqr;
			}
			if (newCoordDistSqr < newBestDistanceSqr)
			{
				newBestDistanceSqr = newCoordDistSqr;
			}
		}
	}
}

void VoxelGridFactory::findIntermediateClosestSurfacePoints(
	SparseGrid<Vector3>& surfaceProjection, const SparseGrid<Vector3>& gradientGrid, const Vector3int32& voxelId, const VoxelGrid* voxelGrid, 
	const Vector3& v0, const Vector3& v1, const Vector3& v2)
{
	Vector3 min, max;
	voxelGrid->getGridDescConst().minMaxCoordsOfGridVoxel(voxelId, min, max);
	if (Geometry3D::triangleAABBIntersect(v0, v1, v2, min, max))
	{
		Vector3 voxelCenterCoord = voxelGrid->getGridDescConst().gridCenterToCoord(voxelId);
		Vector3 closestPoint = Geometry3D::closestPointBetweenPointTriangle(voxelCenterCoord, v0, v1, v2);

		// We can also "ignore" if we find too many triangles on the same voxel...
		// This will generate less accurate topology, but may save generation time.
		if (const Vector3* currentProjection = surfaceProjection.getAt(voxelId))
		{
			if (!(*currentProjection).fuzzyEquals(closestPoint))
			{
				// Check adjacent voxels, as we care about the triangle projection closest to the "outside" of the shape.
				// Lets look for closest negative voxel.
				const Vector3 gradientAtId = *gradientGrid.getAt(voxelId);
				Vector3 testCoord = voxelCenterCoord - gradientAtId * voxelGrid->getGridDescConst().voxWidth;
				float existingBestDistanceSqr = (testCoord - *currentProjection).sqrMagnitude(); // check current projection against adjacents as we scan
				float newBestDistanceSqr = (testCoord - closestPoint).sqrMagnitude(); // check this new projection against adjacents as we scan

				// else, we keep the old value.
				if (newBestDistanceSqr < existingBestDistanceSqr)
				{
					surfaceProjection.insertAt(voxelId, closestPoint);
				}
			}
		}
		else
		{
			surfaceProjection.insertAt(voxelId, closestPoint);
		}
	}
}

void VoxelGridFactory::fillDistanceFieldSurfaceValues(
	VoxelGridDistanceField* distanceFieldOut, const SparseGrid<Vector3>& surfaceProjection, const SparseGrid<Vector3>& gradientGrid, 
	const GridDesc& gridDesc)
{
	for (auto gridIt = surfaceProjection.cbegin(); gridIt != surfaceProjection.cend(); gridIt++)
	{
		const Vector3int32& gridId = gridIt->first;
		const Vector3& surfaceProjection = gridIt->second;
		const Vector3& voxelCenter = gridDesc.gridCenterToCoord(gridId);
		assert(gradientGrid.getAt(gridId) != nullptr);
		const Vector3& gardientAtVoxel = *gradientGrid.getAt(gridId);

		Vector3 delta = surfaceProjection - voxelCenter;
		float distance = gardientAtVoxel.sqrMagnitude() > 0.f ? delta.dot(gardientAtVoxel.normalized()) : 0.f;

		distanceFieldOut->setVoxel(gridId, distance);
	}
}


void VoxelGridFactory::fillDistanceFieldAlongGradientLine(
	VoxelGridDistanceField* distanceFieldOut, const Vector3int32& startId, const SparseGrid<Vector3>& surfaceProjection, 
	const SparseGrid<Vector3>& gradientGrid, const VoxelGrid* voxelGrid)
{
	const GridDesc& gridDesc = voxelGrid->getGridDescConst();
	// If we're "inside" the shape, we go against the gradient towards the surface.
	// The gradient is always towards the center.
	bool reverseGradient = (*voxelGrid->getVoxel(startId)) > 0; 
		
	float lastDistanceValue = 0.f;
	Vector3int32 lastGridId;
	std::stack<Vector3int32> idsAlongGradientStack;
	idsAlongGradientStack.push(startId);
	for (;;)
	{
		// Found something we already processed (surface, or something that was hit by another gradient line)
		// Use this as our distance starting value.
		// At first this will be the surfaceProjection values we scanned earlier.
		const Vector3int32& currentValue = idsAlongGradientStack.top();
		if (const float* distanceAtVoxel = distanceFieldOut->getVoxel(currentValue))
		{
			lastDistanceValue = *distanceAtVoxel;
			lastGridId = idsAlongGradientStack.top();
			idsAlongGradientStack.pop();
			break;
		}
			
		const Vector3* gradientAtVoxel = gradientGrid.getAt(currentValue);
		// Check for Dead End
		if (!gradientAtVoxel)
		{
			idsAlongGradientStack.pop();
			break;
		}

		// Find next voxel to check.
		Vector3 rescaledGradient = (*gradientAtVoxel) * ( 1.0f / gradientAtVoxel->maxAbsValue());
		if (reverseGradient)
		{
			// Flip direction
			rescaledGradient *= -1.0f;
		}

		// Follow gradient to next voxel
		idsAlongGradientStack.push(currentValue +
			Vector3int32((int32_t)std::roundf(rescaledGradient.x), (int32_t)std::roundf(rescaledGradient.y), (int32_t)std::roundf(rescaledGradient.z)));
	}

	while (idsAlongGradientStack.size())
	{
		const Vector3int32& topId = idsAlongGradientStack.top();
		Vector3int32 deltaGrid = topId - lastGridId;
		Vector3 deltaGridFloat = Vector3(deltaGrid.x, deltaGrid.y, deltaGrid.z);
		float distanceAdjust = sqrtf(deltaGridFloat.sqrMagnitude()) * gridDesc.voxWidth;
		if (!reverseGradient)
		{
			// Outside points get more negative
			distanceAdjust *= -1.0f;
		}

		lastDistanceValue += distanceAdjust;
		distanceFieldOut->setVoxel(topId, lastDistanceValue);
		lastGridId = topId;
		idsAlongGradientStack.pop();
	}
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
}; //namespace VSC