#include "GJKTest.h"
#include "btAlignedObjectArray.h"
#include <utility>

namespace GJKTest
{
	ProjGJK::GJKConvexHull* GJKTestHelper::newGJKConvexHullTetrahedron(btVector3& size, btTransform& transform)
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
}