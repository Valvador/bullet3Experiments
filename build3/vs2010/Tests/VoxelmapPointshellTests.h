#pragma once
#include "TestRunner.h"
#include "../../VoxmapSphereCollision/Math/Vector3.h"

class Geometry3DTest : public TestRunnerTest
{
public:
	bool runTest() override;

private:
};

class Geometry2DTest : public TestRunnerTest
{
public:
	bool runTest() override;
};

class VoxelmapTest : public TestRunnerTest
{
public:
	bool runTest() override;
};

class TransformTest : public TestRunnerTest
{
public:
	static float random(float maxRand);
	static int random(int maxRand);

	static bool compareVectors(const VSC::Vector3& v0, const VSC::Vector3& v1, float factor = 1E-5f);

	bool runTest() override;
};

class VoxelDistanceFieldAndSphereMapTests : public TestRunnerTest
{
public:
	bool runTest() override;
};