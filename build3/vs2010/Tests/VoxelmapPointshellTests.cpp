#include "VoxelmapPointshellTests.h"
#include "../../VoxmapSphereCollision/Math/Geometry2D.h"
#include "../../VoxmapSphereCollision/Math/Geometry3D.h"
#include "../../VoxmapSphereCollision/Grid.h"
#include "../../VoxmapSphereCollision/VoxelGrid.h"
#include "../../VoxmapSphereCollision/VoxelGridFactory.h"
#include <assert.h>

using namespace VSC;
bool Geometry2DTest::runTest()
{
	std::vector<bool> testResults;
	{
		// Simple criss-cross lines.
		Geometry2D::LineSegment2D line0(Vector2(-1.0, 0.0), Vector2(1.0, 0.0));
		Geometry2D::LineSegment2D line1(Vector2(0.0, -1.0), Vector2(0.0, 1.0));
		Vector2 pos;
		bool shouldIntersect = Geometry2D::lineSegmentIntersect(line0, line1, pos);
		bool shouldBeZero = pos.fuzzyEquals(Vector2(0));
		assert(shouldBeZero);
		assert(shouldIntersect);
		testResults.push_back(shouldIntersect);
		testResults.push_back(shouldBeZero);
	}
	{
		// Simple near-miss
		Geometry2D::LineSegment2D line0(Vector2(-1.0, 0.0), Vector2(-0.01, 0.0));
		Geometry2D::LineSegment2D line1(Vector2(0.0, -1.0), Vector2(0.0, 1.0));
		Geometry2D::LineSegment2D line2(Vector2(-1.0, 0.0), Vector2(0.01, 0.0));
		Vector2 pos;
		bool shouldMiss = Geometry2D::lineSegmentIntersect(line0, line1, pos);
		bool shouldIntersect = Geometry2D::lineSegmentIntersect(line2, line1, pos);
		bool shouldBeZero = pos.fuzzyEquals(Vector2(0));
		assert(!shouldMiss);
		assert(shouldIntersect);
		assert(shouldBeZero);
		testResults.push_back(!shouldMiss);
		testResults.push_back(shouldIntersect);
		testResults.push_back(shouldBeZero);
	}
	{
		// Should miss, parallel lines.
		Geometry2D::LineSegment2D line0(Vector2(-1.0, 0.0), Vector2(1.0, 0.0));
		Geometry2D::LineSegment2D line1(Vector2(-1.0, 0.0), Vector2(1.0, 0.0));
		Geometry2D::LineSegment2D line2(Vector2(-2.0, 0.0), Vector2(2.0, 0.0));
		Vector2 pos;
		bool shouldMiss0 = Geometry2D::lineSegmentIntersect(line0, line1, pos);
		bool shouldMiss1 = Geometry2D::lineSegmentIntersect(line0, line2, pos);
		assert(!shouldMiss0);
		assert(!shouldMiss1);
		testResults.push_back(!shouldMiss0);
		testResults.push_back(!shouldMiss1);
	}
	{
		//2D AABB vs Line Segment Intersect
		Geometry2D::LineSegment2D lineIntersect(Vector2(-1.1, 0.5), Vector2(-0.8, 1.2));
		Geometry2D::LineSegment2D lineNoIntersect(Vector2(-2.0, 0.9), Vector2(-0.8, 1.3));
		Vector2 aabbMin(-1.0, -1.0);
		Vector2 aabbMax(1.0, 1.0);
		bool shouldHit = Geometry2D::lineSegmentAABBIntersect(lineIntersect, aabbMin, aabbMax);
		bool shouldMiss = Geometry2D::lineSegmentAABBIntersect(lineNoIntersect, aabbMin, aabbMax);
		assert(shouldHit);
		assert(!shouldMiss);
		testResults.push_back(shouldHit);
		testResults.push_back(!shouldMiss);
	}
	
	for (auto testResult : testResults)
	{
		if (!testResult)
		{
			return false;
		}
	}
	return true;
}

bool Geometry3DTest::runTest()
{
	using namespace VSC;
	std::vector<bool> testResults;
	{
		// Plane AABB
		Geometry3D::Plane p0;
		Geometry3D::Plane p1;
		p0.normal = Vector3(1.0f).normalized();
		p0.distance = 0.5f;
		p1.normal = p0.normal;
		p1.distance = 3.0f;
		Vector3 aabbMin(Vector3(-1.0f));
		Vector3 aabbMax(Vector3(1.0f));

		bool shouldHit = Geometry3D::planeAABBIntersect(p0, aabbMin, aabbMax);
		bool shouldMiss = Geometry3D::planeAABBIntersect(p1, aabbMin, aabbMax);
		assert(shouldHit);
		assert(!shouldMiss);
		testResults.push_back(shouldHit);
		testResults.push_back(!shouldMiss);
	}
	{
		// Triangle AABB
		Vector3 aabbMin(Vector3(-1.0f));
		Vector3 aabbMax(Vector3(2.0f));

		Vector3 tMiss[3] = { Vector3(-1.2f, 2.0f, 3.0f), Vector3(-1.2f, 1.2f, 4.0f), Vector3(-1.2f, -0.5, -2.0f) }; // outside AABB
		Vector3 tHit0[3] = { Vector3(-0.8f), Vector3(-0.8f, 1.8f, 1.8f), Vector3(1.8f) }; // completely inside AABB
		Vector3 tHit1[3] = { Vector3(-2.0f), Vector3(-2.0f, 1.8f, 1.8f), Vector3(-0.8f, 0.0f, 1.2f) }; // intersecting slightly
		Vector3 tHit2[3] = { Vector3(-5.0f, -5.0f, 0.0f), Vector3(5.0f, -5.0f, 0.0f), Vector3(0.0f, 15.0f, 0.0f) }; // surrounds AABB

		bool shouldMiss = Geometry3D::triangleAABBIntersect(tMiss[0], tMiss[1], tMiss[2], aabbMin, aabbMax);
		bool shouldHit0 = Geometry3D::triangleAABBIntersect(tHit0[0], tHit0[1], tHit0[2], aabbMin, aabbMax);
		bool shouldHit1 = Geometry3D::triangleAABBIntersect(tHit1[0], tHit1[1], tHit1[2], aabbMin, aabbMax);
		bool shouldHit2 = Geometry3D::triangleAABBIntersect(tHit2[0], tHit2[1], tHit2[2], aabbMin, aabbMax);
		assert(!shouldMiss);
		assert(shouldHit0);
		assert(shouldHit1);
		assert(shouldHit2);
		testResults.push_back(!shouldMiss);
		testResults.push_back(shouldHit0);
		testResults.push_back(shouldHit1);
		testResults.push_back(shouldHit2);
	}

	for (auto testResult : testResults)
	{
		if (!testResult)
		{
			return false;
		}
	}
	return true;
}


void makeBoxVertexIndices(const Vector3& boxSize, const Vector3& boxOffset, std::vector<float>& vertices, std::vector<size_t>& indices)
{
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
	assert(vertices.size() == 3 * 8);
	// 12 Triangles for 6 Faces
	// Front Face
	indices.push_back(0); indices.push_back(4); indices.push_back(2); 
	indices.push_back(4); indices.push_back(6); indices.push_back(2);
	// Right Face
	indices.push_back(4); indices.push_back(5); indices.push_back(6);
	indices.push_back(5); indices.push_back(7); indices.push_back(6);
	// Top Face
	indices.push_back(2); indices.push_back(6); indices.push_back(3);
	indices.push_back(6); indices.push_back(7); indices.push_back(3);
	// Left Face
	indices.push_back(3); indices.push_back(1); indices.push_back(0);
	indices.push_back(3); indices.push_back(0); indices.push_back(2);
	// Back Face
	indices.push_back(3); indices.push_back(7); indices.push_back(1);
	indices.push_back(7); indices.push_back(5); indices.push_back(1);
	// Bottom Face
	indices.push_back(5); indices.push_back(0); indices.push_back(1);
	indices.push_back(5); indices.push_back(4); indices.push_back(0);
	assert(indices.size() == 3 * 12);
}


bool VoxelmapTest::runTest()
{
	std::vector<bool> testResults;
	{
		// Basic Test, simple grid.
		std::vector<float> boxVert;
		std::vector<size_t> boxInd;
		makeBoxVertexIndices(Vector3(1.2f), Vector3(0.0f), boxVert, boxInd);

		float voxelWidth = 1.0f; // With box size 1.2f, we should have center voxel empty, but immediately surrounded voxels full.
		VoxelGrid* resultGrid = VoxelGridFactory::generateVoxelGridFromMesh((const float*)&boxVert[0], 8, &boxInd[0], 12, voxelWidth);
		bool hasEightEntries = resultGrid->numVoxels() == (size_t)8; 
		assert(hasEightEntries);
		testResults.push_back(hasEightEntries);
	}

	for (auto testResult : testResults)
	{
		if (!testResult)
		{
			return false;
		}
	}
	return true;
}