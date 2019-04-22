#pragma once
#include "TestRunner.h"

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