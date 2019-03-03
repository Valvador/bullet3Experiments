#include "GJKConvex.h"
#include "btVector3.h"
#include "btTransform.h"
#include "TestRunner.h"

#pragma once

class GJKTest : public TestRunnerTest
{	
public:
	bool runTest() override;

private:
	class GJKTestHelper
	{
	public:
		static ProjGJK::GJKConvexHull* newGJKConvexHullTetrahedron(btVector3& size, btTransform& transform);
	};
};