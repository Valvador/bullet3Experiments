//#include "stdafx.h"
#include "MeshTools.h"
#include <assert.h>

namespace MeshTools
{
	SplitMeshResult MeshTools::SplitMeshSlow(	const TriangleMeshData& originalTriangleMesh,
												const btVector3& planeNormal,
												const btVector3& pointOnPlane)
	{
		std::vector<btVector3> sharedVertices = originalTriangleMesh.vertices;
		std::vector<unsigned int> sharedIndices = originalTriangleMesh.indices;
		std::vector<MeshTriangle> leftTriangles;
		std::vector<MeshTriangle> rightTriangles;
		assert(sharedIndices.size() % 3 == 0);

		// Split Triangles, create new Vertices and Indices
		for (unsigned int i = 0; i < originalTriangleMesh.indices.size(); i+=3)
		{
			MeshTriangle currentTriangle(	originalTriangleMesh.indices[i], 
											originalTriangleMesh.indices[i + 1], 
											originalTriangleMesh.indices[i + 2]);

			ClipTriangle(	currentTriangle, planeNormal, pointOnPlane, sharedVertices,
							sharedIndices, leftTriangles, rightTriangles);
		}

		// TODO: Close Mesh here!!!
		// CLOSING MESH AND STUFFS
		// END TODO

		// Create Left and Right Mesh
		SplitMeshResult result(	assembleMeshFromSharedData(sharedVertices, leftTriangles), 
								assembleMeshFromSharedData(sharedVertices, rightTriangles));
		return result;
	}
	
	int MeshTools::ClipTriangle(const MeshTriangle& triangle,
								const btVector3& planeNormal,
								const btVector3& pointOnPlane,
								std::vector<btVector3>& sharedVertices,
								std::vector<unsigned int>& sharedIndices,
								std::vector<MeshTriangle>& leftTrianglesOut,
								std::vector<MeshTriangle>& rightTrianglesOut)
	{
		btVector3 normalizedPlaneNorm = planeNormal.normalized();
		btVector3 p0 = sharedVertices[triangle.a()];
		btVector3 p1 = sharedVertices[triangle.b()];
		btVector3 p2 = sharedVertices[triangle.c()];

		// Check if Triangle Intersects Plane
		float k = normalizedPlaneNorm.dot(pointOnPlane);

		float a0 = normalizedPlaneNorm.dot(p0);
		float a1 = normalizedPlaneNorm.dot(p1);
		float a2 = normalizedPlaneNorm.dot(p2);

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
			unsigned int pIA;
			unsigned int pIB;
			unsigned int pIC;

			if (p0in) { pIA = triangle.a(); pIB = triangle.b(); pIC = triangle.c(); }
			if (p1in) { pIA = triangle.b(); pIB = triangle.a(); pIC = triangle.c(); }
			if (p2in) { pIA = triangle.c(); pIB = triangle.b(); pIC = triangle.a(); }

			btVector3 pA = sharedVertices[pIA];
			btVector3 pB = sharedVertices[pIB];
			btVector3 pC = sharedVertices[pIC];

			px0 = PointOnPlane(pA, pB, normalizedPlaneNorm, pointOnPlane);
			px1 = PointOnPlane(pA, pC, normalizedPlaneNorm, pointOnPlane);
			unsigned int px0_index = sharedVertices.size();
			unsigned int px1_index = px0_index + 1;

			// Add these new vertices to sharedVertices
			sharedVertices.push_back(px0);
			sharedVertices.push_back(px1);
			assert(sharedVertices[px0_index] == px0);
			assert(sharedVertices[px1_index] == px1);

			// Left Side + Add these indices to sharedIndices
			sharedIndices.push_back(pIA);
			sharedIndices.push_back(px0_index);
			sharedIndices.push_back(px1_index);
			MeshTriangle t1(pIA, px0_index, px1_index);
			//leftTrianglesOut.push_back(t1);
			rightTrianglesOut.push_back(t1);

			// Right side + Add these new indices to shared Indices
			sharedIndices.push_back(pIB);
			sharedIndices.push_back(px1_index);
			sharedIndices.push_back(pIC);
			sharedIndices.push_back(pIB);
			sharedIndices.push_back(px0_index);
			sharedIndices.push_back(px1_index);
			MeshTriangle t2(pIB, px1_index, pIC);
			MeshTriangle t3(pIB, px0_index, px1_index);
			//rightTrianglesOut.push_back(t2);
			//rightTrianglesOut.push_back(t3);
			leftTrianglesOut.push_back(t2);
			leftTrianglesOut.push_back(t3);


			return 1;
		}

		if (iCount == 2)
		{
			unsigned int pIA;
			unsigned int pIB;
			unsigned int pIC;

			if (!p0in) { pIA = triangle.b(); pIB = triangle.c(); pIC = triangle.a(); }
			if (!p1in) { pIA = triangle.a(); pIB = triangle.c(); pIC = triangle.b(); }
			if (!p2in) { pIA = triangle.a(); pIB = triangle.b(); pIC = triangle.c(); }

			btVector3 pA = sharedVertices[pIA];
			btVector3 pB = sharedVertices[pIB];
			btVector3 pC = sharedVertices[pIC];

			px0 = PointOnPlane(pB, pC, normalizedPlaneNorm, pointOnPlane);
			px1 = PointOnPlane(pA, pC, normalizedPlaneNorm, pointOnPlane);
			unsigned int px0_index = sharedVertices.size();
			unsigned int px1_index = px0_index + 1;

			// Add these new vertices to sharedVertices
			sharedVertices.push_back(px0);
			sharedVertices.push_back(px1);
			assert(sharedVertices[px0_index] == px0);
			assert(sharedVertices[px1_index] == px1);

			// Left Side
			sharedIndices.push_back(pIC);
			sharedIndices.push_back(px0_index);
			sharedIndices.push_back(px1_index);
			MeshTriangle t1(pIC, px0_index, px1_index);
			//rightTrianglesOut.push_back(t1);
			leftTrianglesOut.push_back(t1);

			// Right side
			sharedIndices.push_back(pIB);
			sharedIndices.push_back(pIA);
			sharedIndices.push_back(px0_index);
			sharedIndices.push_back(pIA);
			sharedIndices.push_back(px0_index);
			sharedIndices.push_back(px1_index);
			MeshTriangle t2(pIB, pIA, px0_index);
			MeshTriangle t3(pIA, px0_index, px1_index);
			//leftTrianglesOut.push_back(t2);
			//leftTrianglesOut.push_back(t3);
			rightTrianglesOut.push_back(t2);
			rightTrianglesOut.push_back(t3);

			return 2;
		}

		return 3;
	}

	TriangleMeshData MeshTools::assembleMeshFromSharedData(	const std::vector<btVector3>& sharedVert,
															const std::vector<MeshTriangle>& triangles)
	{
		TriangleMeshData resultMesh;
		
		// Initialize the Marker Vector
		std::vector<unsigned int> marker;
		marker.resize(sharedVert.size());
		for (unsigned int i = 0; i < sharedVert.size(); i++)
		{
			marker[i] = -1; 
		}

		//Traverse Triangles
		for (unsigned int i = 0; i < triangles.size(); i++)
		{
			const MeshTriangle& currentTriangle = triangles[i];
			assemblyProcessTriangleVertex(currentTriangle.a(), sharedVert, marker, resultMesh);
			assemblyProcessTriangleVertex(currentTriangle.b(), sharedVert, marker, resultMesh);
			assemblyProcessTriangleVertex(currentTriangle.c(), sharedVert, marker, resultMesh);
		}

		assert(resultMesh.indices.size() / 3 == triangles.size());
		return resultMesh;
	}

	void MeshTools::assemblyProcessTriangleVertex(const unsigned int& index, const std::vector<btVector3>& sharedVert, const std::vector<unsigned int>& marker, TriangleMeshData& resultMesh)
	{
		if (marker[index] == -1)
		{
			// We havent moved this vertex over yet!
			const btVector3& vertex = sharedVert[index];
			const unsigned int vertIndex = resultMesh.vertices.size();
			resultMesh.vertices.push_back(vertex);
			resultMesh.indices.push_back(vertIndex);
		}
		else
		{
			resultMesh.indices.push_back(marker[index]);
		}
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

} // MeshTools namespace