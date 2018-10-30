#include "GJKConvex.h"
#include "btVector3.h"
#include "btTransform.h"

#pragma once

namespace GJKTest
{
	class GJKTestHelper
	{
	public:
		static ProjGJK::GJKConvexHull* newGJKConvexHullTetrahedron(btVector3& size, btTransform& transform);
	};
}