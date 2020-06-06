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

bool VoxelDistanceFieldAndSphereMapTests::checkBlockVoxelGridValues(VoxelGrid* testGrid, float voxWidth, const Vector3& boxSize)
{
	assert(boxSize.x == boxSize.y && boxSize.y == boxSize.z);

	int centralValue = (boxSize.maxAbsValue() / voxWidth) / 2.0f;

	int maxLayer = testGrid->getGridDescConst().max.z;
	int minLayer = testGrid->getGridDescConst().min.z;
	int layerTraverse = maxLayer - (maxLayer - minLayer) / 2.0f;
	for (int i = 0; i < layerTraverse; i++)
	{
		int testValue = centralValue - i;
		for (int x = -i; x <= i; x++)
		{
			for (int y = -i; y <= i; y++)
			{
				for (int z = -i; z <= i; z++)
				{
					if (std::abs(z) == i || std::abs(y) == i || std::abs(x) == i)
					{
						if (const int32_t* result = testGrid->getVoxel(Vector3int32(x, y, z)))
						{
							if (*result != testValue)
							{
								return false;
							}
						}
					}
				}
			}
		}
	}

	return true;
}

bool testSurfaceProjection(const SparseGrid<Vector3>& surfaceProjection, float voxWidth, Vector3& blockSize)
{
	assert(blockSize.x == blockSize.y && blockSize.y == blockSize.z);
	int depth = (blockSize.maxAbsValue() / voxWidth) / 2.0f;
	float remainder = (blockSize.maxAbsValue() / 2.0f);

	for (int x = depth; x >= -depth; x--)
	{
		for (int y = depth; y >= -depth; y--)
		{
			for (int z = depth; z >= -depth; z--)
			{
				if (std::abs(x) == depth || std::abs(z) == depth || std::abs(y) == depth)
				{
					// We are on the surface voxels.
					Vector3int32 id(x, y, z);
					const Vector3* value = surfaceProjection.getAt(id);
					if (!value)
						return false;

					// Check various expected cases.
					if (std::abs(x) == std::abs(y) && std::abs(y) == std::abs(z) && std::abs(x) == depth)
					{
						// 3 face corners
						Vector3 expectedValue =  Vector3( x / std::abs(x), y / std::abs(y), z / std::abs(z)) * remainder;
						if (!value->fuzzyEquals(expectedValue, 1E-4f))
						{
							return false;
						}
					}
					else if (std::abs(x) == std::abs(y) && std::abs(x) == depth)
					{
						// 2 face corners, x y
						Vector3 expectedValue = Vector3(x / std::abs(x), y / std::abs(y), 0) * remainder + Vector3(0,0, value->z);
						if (!value->fuzzyEquals(expectedValue, 1E-4f))
						{
							return false;
						}
					}
					else if (std::abs(x) == std::abs(z) && std::abs(x) == depth)
					{
						// 2 face corners, x z
						Vector3 expectedValue = Vector3(x / std::abs(x), 0, z / std::abs(z)) * remainder + Vector3(0, value->y, 0);
						if (!value->fuzzyEquals(expectedValue, 1E-4f))
						{
							return false;
						}
					}
					else if (std::abs(z) == std::abs(y) && std::abs(y) == depth)
					{
						// 2 face corners, z y
						Vector3 expectedValue = Vector3(0, y / std::abs(y), z / std::abs(z)) * remainder + Vector3(value->x, 0, 0);
						if (!value->fuzzyEquals(expectedValue, 1E-4f))
						{
							return false;
						}
					}
					else
					{
						// Simple cases.
						Vector3 expectedValue = (std::abs(x) == depth) ?
							Vector3(x / std::abs(x) * remainder, value->y, value->z) :
							(std::abs(y) == depth) ? 
							    (Vector3(value->x, y / std::abs(y) * remainder, value->z)) :
							    Vector3(value->x, value->y, z / std::abs(z) * remainder);

						if (!value->fuzzyEquals(expectedValue, 1E-4f))
						{
							return false;
						}
					}
				}
			}
		}
	}
	// centralValue 

	return true;
}

bool checkDistanceFields(VoxelGridDistanceField* distanceField, float voxWidth, Vector3& blockSize)
{
	assert(blockSize.x == blockSize.y && blockSize.y == blockSize.z);
	int depth = (blockSize.maxAbsValue() / voxWidth) / 2.0f;

	// Assume centered around 0, 0, 0.
	Vector3int32 centerVoxel = Vector3int32(0, 0, 0);
	const float* centerValue = distanceField->getVoxel(centerVoxel);
	if (!centerValue)
		return false;

	float expectedCenter = depth * voxWidth;// +remainder;
	if (std::fabs((*centerValue) - expectedCenter) > 1e-4f)
	{
		return false;
	}

	// Go up and down X.
	for (int x = 0; x <= depth; x++)
	{
		const float* testValue = distanceField->getVoxel(centerVoxel + Vector3int32(x, 0, 0));
		if (!testValue)
			return false;

		float sideExpectedValue = expectedCenter - std::abs(x) * voxWidth;
		if (std::fabs((*testValue) - sideExpectedValue) > 1e-4f)
		{
			return false;
		}
	}
	for (int x = 0; x >= -depth; x--)
	{
		const float* testValue = distanceField->getVoxel(centerVoxel + Vector3int32(x, 0, 0));
		if (!testValue)
			return false;

		float sideExpectedValue = expectedCenter - std::abs(x) * voxWidth;
		if (std::fabs((*testValue) - sideExpectedValue) > 1e-4f)
		{
			return false;
		}
	}

	// Go up and down Y.
	for (int y = 0; y <= depth; y++)
	{
		const float* testValue = distanceField->getVoxel(centerVoxel + Vector3int32(0, y, 0));
		if (!testValue)
			return false;

		float sideExpectedValue = expectedCenter - std::abs(y) * voxWidth;
		if (std::fabs((*testValue) - sideExpectedValue) > 1e-4f)
		{
			return false;
		}
	}
	for (int y = 0; y >= -depth; y--)
	{
		const float* testValue = distanceField->getVoxel(centerVoxel + Vector3int32(0, y, 0));
		if (!testValue)
			return false;

		float sideExpectedValue = expectedCenter - std::abs(y) * voxWidth;
		if (std::fabs((*testValue) - sideExpectedValue) > 1e-4f)
		{
			return false;
		}
	}

	// Go up and down Z.
	for (int z = 0; z <= depth; z++)
	{
		const float* testValue = distanceField->getVoxel(centerVoxel + Vector3int32(0, 0, z));
		if (!testValue)
			return false;

		float sideExpectedValue = expectedCenter - std::abs(z) * voxWidth;
		if (std::fabs((*testValue) - sideExpectedValue) > 1e-4f)
		{
			return false;
		}
	}
	for (int z = 0; z >= -depth; z--)
	{
		const float* testValue = distanceField->getVoxel(centerVoxel + Vector3int32(0, 0, z));
		if (!testValue)
			return false;

		float sideExpectedValue = expectedCenter - std::abs(z) * voxWidth;
		if (std::fabs((*testValue) - sideExpectedValue) > 1e-4f)
		{
			return false;
		}
	}

	// This could use some improvement. Going to be very lazy here.
	return true;
}

bool VoxelDistanceFieldAndSphereMapTests::runTest()
{
	std::vector<float> boxVert;
	std::vector<size_t> boxInd;
	Vector3 blockSize = Vector3(1.2f);
	VoxelGridFactory::debug_MakeBoxVertexIndices(blockSize, Vector3(0.0f), boxVert, boxInd);
	float voxelWidth = 0.2f; // With box size 1.2f, we should have center voxel empty, but immediately surrounded voxels full.
	VoxelGrid* resultGrid = VoxelGridFactory::generateVoxelGridFromMesh((const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth, 4 /*expandBy*/);
	SparseGrid<Vector3> gridGradient = VoxelGridFactory::getVoxelGridGradient(resultGrid);
	SparseGrid<Vector3> surfaceProjection = VoxelGridFactory::getSurfaceProjection(gridGradient, (const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth, resultGrid);
	VoxelGridDistanceField* distanceField = VoxelGridFactory::generateDistanceFieldFromMeshAndVoxelGrid(surfaceProjection, gridGradient, resultGrid);
	SphereTree* sphereTree = VoxelGridFactory::generateSphereTreeFromSurfaceProjections(surfaceProjection);

	/*  Our Test Box
         Voxel Grid Expected
	     -3-2-1 0 1 2 3
		 3|-----------|
		 2| 1 1 1 1 1 |
		 1| 1 2 2 2 1 |
		 0| 1 2 3 2 1 |
		-1| 1 2 2 2 1 |
		-2| 1 1 1 1 1 |
		-3|-----------|
	*/
	if (!checkBlockVoxelGridValues(resultGrid, voxelWidth, blockSize))
	{
		return false;
	}
	/*
	     Gradient Expected
		 -3-2-1 0 1 2 3
		 3|-----------|
		 2| \   |   / |
		 1|   \ | /   |
		 0|-----c-----|
		-1|   / | \   |
		-2| /   |   \ |
		-3|-----------|
	
	
	*/
	if (!testSurfaceProjection(surfaceProjection, voxelWidth, blockSize))
	{
		return false;
	}
	if (!checkDistanceFields(distanceField, voxelWidth, blockSize))
	{
		return false;
	}

	return true;
}