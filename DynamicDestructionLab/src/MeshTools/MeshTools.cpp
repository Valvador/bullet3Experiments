//#include "stdafx.h"
#include "MeshTools.h"
#include <assert.h>

MeshTools::SplitMeshResult MeshTools::splitMeshZ(TriangleMeshData* oldMesh, float zCut, float zInsert, bool marginalTrisInLeft, bool marginalTrisInRight)
{
	TriangleMeshData* leftMesh = new TriangleMeshData();
	TriangleMeshData* rightMesh = new TriangleMeshData();


	for (size_t i = 0; i < oldMesh->numIndices; i += 3)
	{
		int a = oldMesh->indices[i];
		int b = oldMesh->indices[i + 1];
		int c = oldMesh->indices[i + 2];
	}

	//PLACEHOLDER
	return MeshTools::SplitMeshResult();
}

// SOURCE FUNCTION: http://www.xbdev.net/java/tutorials_3d/clipping/index.php
int MeshTools::ClipTriangle(const Triangle& triangle, const btVector3& planeNormal, const btVector3& pointOnPlane, Triangles& leftTrianglesOut, Triangles& rightTrianglesOut)
{
	assert(triangle.size() == 3);
	btVector3 normalizedPlaneNorm = planeNormal.normalized();

	btVector3 p0 = triangle[0];
	btVector3 p1 = triangle[1];
	btVector3 p2 = triangle[2];

	// Check if Triangle Intersects Plane
	float k = normalizedPlaneNorm.dot(pointOnPlane);

	float a0 = normalizedPlaneNorm.dot(triangle[0]);
	float a1 = normalizedPlaneNorm.dot(triangle[1]);
	float a2 = normalizedPlaneNorm.dot(triangle[2]);

	// Determine How Many points Are On Positive Side of Plane
	int iCount = 0;
	bool p0in = false;
	bool p1in = false;
	bool p2in = false;

	if ((k - a0) > 0){ iCount++; p0in = true; }
	if ((k - a1) > 0){ iCount++; p1in = true; }
	if ((k - a2) > 0){ iCount++; p2in = true; }

	// If Triangle is fully on one side.
	if (iCount == 0)
	{
		// All on the positive side of Plane (Left)
		leftTrianglesOut.push_back(triangle);
		return 3;
	}

	if (iCount == 3)
	{
		// All on the negative side of Plane (Right)
		rightTrianglesOut.push_back(triangle);
		return 0;
	}

	// Two Vectors for the points created by the plane line
	btVector3 px0;
	btVector3 px1;

	// No we have two points on the plane
	// that make up the intersection points. 
	// We can create up to 3 new triangles from this.
	if (iCount == 1)
	{
		btVector3 pA;
		btVector3 pB;
		btVector3 pC;

		if (p0in) { pA = p0; pB = p1; pC = p2; }
		if (p1in) { pA = p1; pB = p0; pC = p2; }
		if (p2in) { pA = p2; pB = p1; pC = p0; }

		px0 = PointOnPlane(pA, pB, normalizedPlaneNorm, pointOnPlane);
		px1 = PointOnPlane(pA, pC, normalizedPlaneNorm, pointOnPlane);

		// Left Side
		Triangle t1 = std::vector<btVector3>();
		t1.resize(3);
		t1[0] = pA; t1[1] = px0; t1[2] = px1;
		leftTrianglesOut.push_back(t1);

		// Right side
		Triangle t2 = std::vector<btVector3>();
		Triangle t3 = std::vector<btVector3>();
		t2.resize(3);
		t3.resize(3);
		t2[0] = pB; t2[1] = px1; t2[2] = pC;
		t3[0] = pB; t3[1] = px0; t3[2] = px1;
		rightTrianglesOut.push_back(t2);
		rightTrianglesOut.push_back(t3);


		return 1;
	}

	if (iCount == 2)
	{
		btVector3 pA;
		btVector3 pB;
		btVector3 pC;

		if (!p0in) { pA = p1; pB = p2; pC = p0; }
		if (!p1in) { pA = p0; pB = p2; pC = p1; }
		if (!p2in) { pA = p0; pB = p1; pC = p2; }

		px0 = PointOnPlane(pB, pC, normalizedPlaneNorm, pointOnPlane);
		px1 = PointOnPlane(pA, pC, normalizedPlaneNorm, pointOnPlane);

		// Left Side
		Triangle t1 = std::vector<btVector3>();
		t1.resize(3);
		t1[0] = pC; t1[1] = px0; t1[2] = px1;
		rightTrianglesOut.push_back(t1);

		// Right side
		Triangle t2 = std::vector<btVector3>();
		Triangle t3 = std::vector<btVector3>();
		t2.resize(3);
		t3.resize(3);
		t2[0] = pB; t2[1] = pA; t2[2] = px0;
		t3[0] = pA; t3[1] = px0; t3[2] = px1;
		leftTrianglesOut.push_back(t2);
		leftTrianglesOut.push_back(t3);

		return 2;
	}

	return 3;
}

float MeshTools::distRayPlane(const btVector3& vStart, const btVector3& vEnd, const btVector3& vnPlaneNormal, const btVector3& vPointOnPlane)
{
	float cosAlpha;
	float deltaD;

	float planeD = vnPlaneNormal.dot(vPointOnPlane);

	btVector3 vRayVector = vEnd - vStart;
	btVector3 vnRayVector = vRayVector.normalized();

	cosAlpha = vnPlaneNormal.dot(vnRayVector);


	// parallel to the plane (alpha=90)
	if (std::fabs(cosAlpha) < 0.001f) return vRayVector.length();

	deltaD = planeD - vStart.dot(vnPlaneNormal);

	float l = (deltaD / cosAlpha);

	//float k = Vector3.dot( vnPlaneNormal, vPointOnPlane );
	//float a0 = Vector3.dot( vnPlaneNormal, vStart );
	//l = (k - a0)/cosAlpha;

	return l;
}

btVector3 MeshTools::PointOnPlane(const btVector3& vStart, const btVector3& vEnd, const btVector3& vPlaneNormal, const btVector3& vPointOnPlane)
{
	btVector3 vn = vEnd - vStart;
	vn = vn.normalized();
	float length = distRayPlane(vStart, vEnd, vPlaneNormal, vPointOnPlane);
	btVector3 px = vn * length;
	px += vStart;
	return px;
}