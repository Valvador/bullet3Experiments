#include "GJK.h"
#include "GJKConvex.h"
#include "btVector3.h"
#include "btTransform.h"
#include "btMatrix3x3.h"
#include "btAlignedObjectArray.h"

#include "GJKTest.h"
#include <utility>
#include <assert.h>

bool GJKTest::runTest()
{
	// GJKTest
	btVector3 partSize = btVector3(1.0f, 1.0f, 1.0f);
	btTransform originTForm = btTransform(btMatrix3x3(1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f));
	btTransform collidingTForm = originTForm;
	collidingTForm.setOrigin(partSize / 2.0f);
	btTransform notCollidingTForm = originTForm;
	notCollidingTForm.setOrigin(partSize * 2.0f);
	ProjGJK::GJKConvexHull* obj0 = GJKTest::GJKTestHelper::newGJKConvexHullTetrahedron(partSize, btTransform(originTForm));
	ProjGJK::GJKConvexHull* obj1 = GJKTest::GJKTestHelper::newGJKConvexHullTetrahedron(partSize, btTransform(collidingTForm));
	ProjGJK::GJKConvexHull* obj2 = GJKTest::GJKTestHelper::newGJKConvexHullTetrahedron(partSize, btTransform(notCollidingTForm));

	ProjGJK::GJKPair collidingPair(obj0, obj1);
	ProjGJK::GJKPair nonCollidingPair(obj0, obj2);

	ProjGJK::GJKContactResult result0;
	ProjGJK::GJKContactResult result1;
	bool collidingGJK = collidingPair.doGJK(&result0);
	bool nonCollidingGJK = nonCollidingPair.doGJK(&result1);

	assert(collidingGJK);
	assert(!nonCollidingGJK);

	return collidingGJK && !nonCollidingGJK;
}

ProjGJK::GJKConvexHull* GJKTest::GJKTestHelper::newGJKConvexHullTetrahedron(btVector3& size, btTransform& transform)
{
	btVector3 halfSize = size / 2.0f;
	btAlignedObjectArray<btVector3> boxVertices;
	// VERTICES
	// Top Face
	boxVertices.push_back(btVector3(halfSize.getX(), halfSize.getY(), halfSize.getZ()));		//0
	boxVertices.push_back(btVector3(-halfSize.getX(), halfSize.getY(), halfSize.getZ()));		//1
	boxVertices.push_back(btVector3(halfSize.getX(), halfSize.getY(), -halfSize.getZ()));		//2
	boxVertices.push_back(btVector3(-halfSize.getX(), halfSize.getY(), -halfSize.getZ()));		//3
	// Bottom Face
	boxVertices.push_back(btVector3(halfSize.getX(), -halfSize.getY(), halfSize.getZ()));		//4
	boxVertices.push_back(btVector3(-halfSize.getX(), -halfSize.getY(), halfSize.getZ()));		//5
	boxVertices.push_back(btVector3(halfSize.getX(), -halfSize.getY(), -halfSize.getZ()));		//6
	boxVertices.push_back(btVector3(-halfSize.getX(), -halfSize.getY(), -halfSize.getZ()));		//7

	btAlignedObjectArray<std::pair<int,int>> boxEdges;
	// EDGES
	// TOP FACES
	boxEdges.push_back(std::pair<int, int>(0, 1));
	boxEdges.push_back(std::pair<int, int>(0, 2));
	boxEdges.push_back(std::pair<int, int>(2, 3));
	boxEdges.push_back(std::pair<int, int>(1, 3));

	// BOTTOM FACES
	boxEdges.push_back(std::pair<int, int>(4, 5));
	boxEdges.push_back(std::pair<int, int>(4, 6));
	boxEdges.push_back(std::pair<int, int>(6, 7));
	boxEdges.push_back(std::pair<int, int>(5, 7));

	// TOP TO BOTTOM
	boxEdges.push_back(std::pair<int, int>(0, 4));
	boxEdges.push_back(std::pair<int, int>(1, 5));
	boxEdges.push_back(std::pair<int, int>(2, 6));
	boxEdges.push_back(std::pair<int, int>(3, 7));

	return new ProjGJK::GJKConvexHull(transform, boxVertices, boxEdges);
}
