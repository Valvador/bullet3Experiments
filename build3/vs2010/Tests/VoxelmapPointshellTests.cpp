#include "VoxelmapPointshellTests.h"
#include "../../VoxmapSphereCollision/Math/Geometry2D.h"
#include "../../VoxmapSphereCollision/Math/Geometry3D.h"
#include "../../VoxmapSphereCollision/Base/Grid.h"
#include "../../VoxmapSphereCollision/VoxelGrid.h"
#include "../../VoxmapSphereCollision/VoxelGridFactory.h"
#include "../../VoxmapSphereCollision/Math/Transform.h"
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


bool VoxelmapTest::runTest()
{
	std::vector<bool> testResults;
	{
		{
			// Basic Test, simple grid.
			std::vector<float> boxVert;
			std::vector<size_t> boxInd;
			VoxelGridFactory::debug_MakeBoxVertexIndices(Vector3(1.2f), Vector3(0.0f), boxVert, boxInd);

			float voxelWidth = 1.0f; // With box size 1.2f, we should have center voxel empty, but immediately surrounded voxels full.
			{
				VoxelGrid* resultGrid = VoxelGridFactory::generateVoxelGridFromMesh((const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth);
				int countSurfaceVoxels = resultGrid->countSurfaceVoxels();
				bool hasCorrectNumEntries = countSurfaceVoxels == 26; //27 - 1 [This test is set up so that the central box is empty]
				assert(hasCorrectNumEntries);
				testResults.push_back(hasCorrectNumEntries);
			}

			{
				// Add another box around 4.0, 4.0, 4.0
				VoxelGridFactory::debug_MakeBoxVertexIndices(Vector3(1.2f), Vector3(4.0f), boxVert, boxInd);
				VoxelGrid* resultGrid = VoxelGridFactory::generateVoxelGridFromMesh((const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth);
				int countSurfaceVoxels = resultGrid->countSurfaceVoxels();
				bool hasCorrectNumEntries = countSurfaceVoxels == 52; //27 - 1 [This test is set up so that the central box is empty]
				assert(hasCorrectNumEntries);
				testResults.push_back(hasCorrectNumEntries);
			}
		}

		{
			// Finer grid test
			// Basic Test, simple grid.
			std::vector<float> boxVert;
			std::vector<size_t> boxInd;
			VoxelGridFactory::debug_MakeBoxVertexIndices(Vector3(1.2f), Vector3(0.0f), boxVert, boxInd);

			float voxelWidth = 0.2f; // With box size 1.2f, we should have center voxel empty, but immediately surrounded voxels full.
			{
				VoxelGrid* resultGrid = VoxelGridFactory::generateVoxelGridFromMesh((const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth);
				int countSurfaceVoxels = resultGrid->countSurfaceVoxels();
				bool hasCorrectNumEntries = countSurfaceVoxels == 7 * 7 * 7 - 5 * 5 * 5;
				assert(hasCorrectNumEntries);
				testResults.push_back(hasCorrectNumEntries);
			}
		}
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

float TransformTest::random(float max)
{
	return static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / max));
}

int TransformTest::random(int max)
{
	return (rand() / (RAND_MAX / max));
}

// Hacky way to compare two vectors despite floating point drift
// Basically checks if the smallest vector is within 'factor' % of the larger vector.
bool TransformTest::compareVectors(const Vector3& v0, const Vector3& v1, float factor)
{
	float adjustedEps = v0.sqrMagnitude() > v1.sqrMagnitude() ? 
		sqrt(v0.sqrMagnitude()) * factor: 
		sqrt(v1.sqrMagnitude()) * factor;

	if (adjustedEps < 1e-5f)
	{
		adjustedEps = 1e-5f;
	}

	if ((v1 - v0).sqrMagnitude() <= adjustedEps)
	{
		return true;
	}
	return false;
}

Vector3 randomVector()
{
	return Vector3(TransformTest::random(1000.0f), TransformTest::random(1000.0f), TransformTest::random(1000.0f));
}

Quaternion randomQuat()
{
	Quaternion result = Quaternion(TransformTest::random(1000.0f), randomVector());
	result.normalize();
	return result;
}

Transform randomTransform()
{
	return Transform(randomVector(), randomQuat());
}

bool TransformTest::runTest()
{
	std::srand(1);

	// INVERSE TESTS
	for (int i = 0; i < 100; i++)
	{
		Transform randT = randomTransform();
		Transform finalT = randT * randT.inverse();

		bool positionReset = (finalT.getPosition()).sqrMagnitude() < 1e-5f;
		bool rotationScalar = (finalT.getRotation().getScalarW() - 1.0) < 1e-5f;
		bool rotationVector = (finalT.getRotation().getVectorIJK().sqrMagnitude()) < 1e-5f;
		if (!positionReset || !rotationScalar || !rotationVector)
		{
			assert(positionReset);  // Position did not return to origin
			assert(rotationScalar); // W in Quaternion did not get set to 1
			assert(rotationVector); // IJK in Quaternion did not get set to 0,0,0
			return false;
		}
	}

	// pointToWorldSpace and pointToLocalSpace
	for (int i = 0; i < 100; i++)
	{
		Vector3 randV = randomVector();
		Transform randTransform = randomTransform();

		// Assume the starting vector is world space
		Vector3 localSpace = randTransform.pointToLocalSpace(randV);
		Vector3 worldSpace = randTransform.pointToWorldSpace(localSpace);

		if (!TransformTest::compareVectors(worldSpace, randV))
		{
			return false;
		}

		// Assume the starting vector is local space
		worldSpace = randTransform.pointToWorldSpace(randV);
		localSpace = randTransform.pointToLocalSpace(worldSpace);

		if (!TransformTest::compareVectors(localSpace, randV))
		{
			return false;
		}
	}

	// 90 degree rotation of 1, 0, 0 using 0, 1, 0 axis.
	Vector3 start = Vector3(1, 0, 0);
	Vector3 rotAxis = Vector3(0, 1, 0);
	float rotAngle = 90;
	Quaternion rotQuat = Quaternion::createRotationQuaternionDegrees(rotAngle, rotAxis);

	Vector3 result = rotQuat.rotateVector(start);
	if (!TransformTest::compareVectors(result, Vector3(0, 0, -1)))
	{
		return false;
	}

	Vector3 result2 = rotQuat.rotateVector(result);
	if (!TransformTest::compareVectors(result2, Vector3(-1, 0, 0)))
	{
		return false;
	}

	Vector3 result3 = rotQuat.rotateVector(result2);
	if (!TransformTest::compareVectors(result3, Vector3(0, 0, 1)))
	{
		return false;
	}

	Vector3 result4 = rotQuat.rotateVector(result3);
	if (!TransformTest::compareVectors(result4, start))
	{
		return false;
	}

	// Random Axes and rotate around in full circles.
	for (int i = 0; i < 100; i++)
	{
		int numToRotate = random(100);
		Vector3 rotAxis = randomVector();
		Vector3 startVector = randomVector();
		float rotAngle = 360.0f / numToRotate;
		Quaternion rotQuat = Quaternion::createRotationQuaternionDegrees(rotAngle, rotAxis);
		Vector3 workingVector = startVector;
		for (int j = 0; j < numToRotate; j++)
		{
			workingVector = rotQuat.rotateVector(workingVector);
		}

		if (!TransformTest::compareVectors(workingVector, startVector))
		{
			return false;
		}
	}

	return true;
}

bool VoxelDistanceFieldAndSphereMapTests::runTest()
{
	std::vector<float> boxVert;
	std::vector<size_t> boxInd;
	VoxelGridFactory::debug_MakeBoxVertexIndices(Vector3(1.11f), Vector3(0.0f), boxVert, boxInd);
	float voxelWidth = 0.2f; // With box size 1.2f, we should have center voxel empty, but immediately surrounded voxels full.
	VoxelGrid* resultGrid = VoxelGridFactory::generateVoxelGridFromMesh((const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth);
	SparseGrid<Vector3> gridGradient = VoxelGridFactory::getVoxelGridGradient(resultGrid);
	SparseGrid<Vector3> surfaceProjection = VoxelGridFactory::getSurfaceProjection(gridGradient, (const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth, resultGrid);
	VoxelGridDistanceField* distanceField = VoxelGridFactory::generateDistanceFieldFromMeshAndVoxelGrid(surfaceProjection, gridGradient, resultGrid);
	SphereTree* sphereTree = VoxelGridFactory::generateSphereTreeFromSurfaceProjections(surfaceProjection);
}