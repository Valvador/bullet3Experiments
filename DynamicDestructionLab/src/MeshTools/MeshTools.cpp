//#include "stdafx.h"
#include "MeshTools.h"
#include <assert.h>


namespace MeshTools
{
	Bridge2dTo3dPoint::Bridge2dTo3dPoint(const std::vector<Edge>& edges, const std::vector<btVector3>& sharedVert)
	{
		// HACK AS FACK but will generally get the idea without me having to
		// grab code earlier.
		for (unsigned int i = 0; i < edges.size(); i++)
		{
			const btVector3& vector = sharedVert[edges[i].p0_i];
			if (vector.x() == 0 && vector.y() != 0 && vector.z() != 0)
			{
				flatDimension = ZeroDim_X;
				break;
			}
			if (vector.y() == 0 && vector.x() != 0 && vector.z() != 0)
			{
				flatDimension = ZeroDim_Y;
				break;
			}
			if (vector.z() == 0 && vector.x() != 0 && vector.y() != 0)
			{
				flatDimension = ZeroDim_Z;
				break;
			}
		}
	}

	p2t::Point* Bridge2dTo3dPoint::convertVec3ToPoint(const btVector3& vec) const
	{
		switch (flatDimension)
		{
		case ZeroDim_X:
			return new p2t::Point(vec.y(), vec.z());
		case ZeroDim_Y:
			return new p2t::Point(vec.x(), vec.z());
		case ZeroDim_Z:
			return new p2t::Point(vec.x(), vec.y());
		default:
			NULL;
		}
	}

	btVector3 Bridge2dTo3dPoint::convertPointToVec3(p2t::Point* pt) const
	{
		if (pt)
		{
			switch (flatDimension)
			{
			case ZeroDim_X:
				return btVector3(0, pt->x, pt->y);
			case ZeroDim_Y:
				return btVector3(pt->x, 0, pt->y);
			case ZeroDim_Z:
				return btVector3(pt->x, pt->y, 0);
			default:;
			}
		}
		return btVector3();
	}

	SplitMeshResult MeshTools::SplitMeshSlow(	const TriangleMeshData& originalTriangleMesh,
												const btVector3& planeNormal,
												const btVector3& pointOnPlane)
	{
		std::vector<btVector3> sharedVertices = originalTriangleMesh.vertices;
		std::vector<unsigned int> sharedIndices = originalTriangleMesh.indices;
		std::vector<MeshTriangle> leftTriangles;
		std::vector<MeshTriangle> rightTriangles;
		EdgeMap newEdgesFront;
		EdgeMap newEdgesBack;
		assert(sharedIndices.size() % 3 == 0);

		// Split Triangles, create new Vertices and Indices
		for (unsigned int i = 0; i < originalTriangleMesh.indices.size(); i+=3)
		{
			MeshTriangle currentTriangle(	originalTriangleMesh.indices[i], 
											originalTriangleMesh.indices[i + 1], 
											originalTriangleMesh.indices[i + 2]);

			ClipTriangle(	currentTriangle, planeNormal, pointOnPlane, 
							sharedVertices, sharedIndices,
							leftTriangles, rightTriangles,
							newEdgesFront, newEdgesBack);
		}

		// TODO: Close Mesh here!!!
		CloseClippedMesh(	sharedVertices, sharedIndices, leftTriangles, 
							rightTriangles, newEdgesFront, newEdgesBack);

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
								std::vector<MeshTriangle>& rightTrianglesOut,
								EdgeMap& newEdgesFront,        
								EdgeMap& newEdgesBack)
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
			
			// Add Edge to newly created edges Maps
			Edge newEdge;
			std::string px0K = convertVecToStrKey(px0);
			std::string px1K = convertVecToStrKey(px1);
			newEdge.p0_i = px0_index;
			newEdge.p1_i = px1_index;
			newEdgesFront.insert(std::pair<std::string, Edge>(px0K, newEdge));
			newEdgesBack.insert(std::pair<std::string, Edge>(px1K, newEdge));

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

			// Add Edge to newly created edges Maps
			Edge newEdge;
			std::string px0K = convertVecToStrKey(px0);
			std::string px1K = convertVecToStrKey(px1);
			newEdge.p0_i = px0_index;
			newEdge.p1_i = px1_index;
			newEdgesFront.insert(std::pair<std::string, Edge>(px0K, newEdge));
			newEdgesBack.insert(std::pair<std::string, Edge>(px1K, newEdge));

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


	void MeshTools::CloseClippedMesh(	std::vector<btVector3>& sharedVertices, std::vector<unsigned int>& sharedIndices,
										std::vector<MeshTriangle>& leftTrianglesOut, std::vector<MeshTriangle>& rightTrianglesOut,
										EdgeMap& newEdgesFront, EdgeMap& newEdgesBack)
	{
		// We first need to figure out which of the edges combine into a loop together.
		std::vector<std::vector<Edge>> edgeLoops;
		while (newEdgesFront.size())
		{
			std::vector<Edge> closedLoop;
			EdgeMap::iterator firstEdgeIt = newEdgesFront.begin();
			Edge firstEdge = firstEdgeIt->second;
			newEdgesFront.erase(firstEdgeIt);
			EdgeMap::iterator firstEdgeBackIt = newEdgesBack.find(convertVecToStrKey(sharedVertices[firstEdge.p1_i]));
			while (firstEdgeBackIt->second != firstEdge)
			{
				firstEdgeBackIt++;
			}
			newEdgesBack.erase(firstEdgeBackIt);
			Edge currentEdge = firstEdge;
			closedLoop.push_back(firstEdge);
			bool lastEdgeFlipped = false;

			while (iterateToNextEdge(sharedVertices, lastEdgeFlipped, newEdgesFront, newEdgesBack, currentEdge)
				&& !compareEdgeVertices(currentEdge, firstEdge, sharedVertices))
			{
				Edge toPush;
				// Flip edges so that they go in a circle
				if (lastEdgeFlipped)
				{
					toPush.p0_i = currentEdge.p1_i;
					toPush.p1_i = currentEdge.p0_i;
				}
				else
				{
					toPush = currentEdge;
				}
				closedLoop.push_back(toPush);
			}

			if (closedLoop.size() > 2 && edgesShareVertex(currentEdge, firstEdge, sharedVertices))
			{
				edgeLoops.push_back(closedLoop);
			}
		}

		// Now we have all the closed loops, go through them with Triangulation code.
		for (unsigned int i = 0; i < edgeLoops.size(); i++)
		{
			Bridge2dTo3dPoint converter(edgeLoops[i], sharedVertices);
			std::vector<p2t::Point*> polyLine = getPolyLineFromEdges(edgeLoops[i], sharedVertices, converter);
			p2t::CDT* triangulator = new p2t::CDT(polyLine);
			triangulator->Triangulate();
			addP2tTrianglesToMeshes(triangulator->GetTriangles(), sharedVertices, sharedIndices,
									leftTrianglesOut, rightTrianglesOut, converter);
			delete triangulator;
			for (unsigned int j = 0; j < polyLine.size(); j++)
			{
				delete polyLine[j];
			}
		}
	}

	bool MeshTools::iterateToNextEdge(std::vector<btVector3>& sharedVertices, bool& lastEdgeFlipped,
									EdgeMap& newEdgesFront, EdgeMap& newEdgesBack, Edge& out)
	{
		btVector3 lastVertex;
		if (lastEdgeFlipped)
		{
			lastVertex = sharedVertices[out.p0_i];
		}
		else
		{
			lastVertex = sharedVertices[out.p1_i];
		}
		EdgeMap::iterator edgeIt = newEdgesFront.find(convertVecToStrKey(lastVertex));
		if (edgeIt != newEdgesFront.end())
		{
			out = edgeIt->second;
			newEdgesFront.erase(edgeIt);
			EdgeMap::iterator edgeBackIt = newEdgesBack.find(convertVecToStrKey(sharedVertices[out.p1_i]));
			while (edgeBackIt->second != out)
			{
				edgeBackIt++;
			}
			newEdgesBack.erase(edgeBackIt);
			lastEdgeFlipped = false;
			return true;
		}

		edgeIt = newEdgesBack.find(convertVecToStrKey(lastVertex));
		if (edgeIt != newEdgesBack.end())
		{
			out = edgeIt->second;
			newEdgesBack.erase(edgeIt);
			EdgeMap::iterator edgeFrontIt = newEdgesFront.find(convertVecToStrKey(sharedVertices[out.p0_i]));
			while (edgeFrontIt->second != out)
			{
				edgeFrontIt++;
			}
			newEdgesFront.erase(edgeFrontIt);
			lastEdgeFlipped = true;
			return true;
		}

		return false;
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


	//UTILS
	std::string MeshTools::convertVecToStrKey(const btVector3& vector)
	{
		int a = vector.x() * 1000;
		int b = vector.y() * 1000;
		int c = vector.z() * 1000;
		return std::to_string(a) + std::to_string(b) + std::to_string(c);
	}

	bool MeshTools::compareEdgeVertices(Edge& e0, Edge& e1, const std::vector<btVector3>& sharedVertices)
	{
		const btVector3& e0_v0 = sharedVertices[e0.p0_i];
		const btVector3& e0_v1 = sharedVertices[e0.p1_i];
		const btVector3& e1_v0 = sharedVertices[e1.p0_i];
		const btVector3& e1_v1 = sharedVertices[e1.p1_i];

		bool v0s_equal = (e0_v0 - e1_v0).length2() < btScalar(1e-4);
		bool v1s_equal = (e0_v1 - e1_v1).length2() < btScalar(1e-4);

		bool v0s_cross_equal = (e0_v0 - e1_v1).length2() < btScalar(1e-4);
		bool v1s_cross_equal = (e0_v1 - e1_v0).length2() < btScalar(1e-4);

		return (v0s_equal && v1s_equal) || (v0s_cross_equal && v1s_cross_equal);
	}

	bool MeshTools::edgesShareVertex(Edge& e0, Edge& e1, const std::vector<btVector3>& sharedVertices)
	{
		const btVector3& e0_v0 = sharedVertices[e0.p0_i];
		const btVector3& e0_v1 = sharedVertices[e0.p1_i];
		const btVector3& e1_v0 = sharedVertices[e1.p0_i];
		const btVector3& e1_v1 = sharedVertices[e1.p1_i];
		bool v0s_equal = (e0_v0 - e1_v0).length2() < btScalar(1e-4);
		bool v1s_equal = (e0_v1 - e1_v1).length2() < btScalar(1e-4);

		bool v0s_cross_equal = (e0_v0 - e1_v1).length2() < btScalar(1e-4);
		bool v1s_cross_equal = (e0_v1 - e1_v0).length2() < btScalar(1e-4);

		return (v0s_equal || v1s_equal || v0s_cross_equal || v1s_cross_equal);
	}

	std::vector<p2t::Point*> MeshTools::getPolyLineFromEdges(std::vector<Edge> edges, const std::vector<btVector3>& sharedVertices, const Bridge2dTo3dPoint& converter)
	{
		std::vector<p2t::Point*> result;
		for (unsigned int i = 0; i < edges.size(); i++)
		{
			const btVector3& frontVertex = sharedVertices[edges[i].p0_i];
			result.push_back(converter.convertVec3ToPoint(frontVertex));
		}
		return result;
	}

	void MeshTools::addP2tTrianglesToMeshes(std::vector<p2t::Triangle*>& triangles, std::vector<btVector3>& sharedVertices, std::vector<unsigned int>& sharedIndices,
											std::vector<MeshTriangle>& leftTrianglesOut, std::vector<MeshTriangle>& rightTrianglesOut, const Bridge2dTo3dPoint& converter)
	{
		for (unsigned int i = 0; i < triangles.size(); i++)
		{
			// TODO, MAY WANT TO RE-ARRANGE THE ORDER OF TRIANGLES FOR DIFFERENT 
			// HALVES OF THE OBJECTS!
			btVector3 p0 = converter.convertPointToVec3(triangles[i]->GetPoint(0));
			btVector3 p1 = converter.convertPointToVec3(triangles[i]->GetPoint(1));
			btVector3 p2 = converter.convertPointToVec3(triangles[i]->GetPoint(2));
			unsigned int p0_index = sharedVertices.size();
			unsigned int p1_index = p0_index + 1;
			unsigned int p2_index = p0_index + 2;

			sharedVertices.push_back(p0);
			sharedVertices.push_back(p1);
			sharedVertices.push_back(p2);

			MeshTriangle leftTriangle(p0_index, p1_index, p2_index);
			MeshTriangle rightTriangle(p2_index, p1_index, p0_index);
			leftTrianglesOut.push_back(leftTriangle);
			rightTrianglesOut.push_back(rightTriangle);
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