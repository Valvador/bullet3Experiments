#pragma once

#include "LinearMath/btVector3.h"
#include "../Poly2Tri/poly2tri.h"
#include <string>
#include <vector>
#include <map>
#include <functional>


namespace MeshTools
{
	class Edge;
}

typedef std::vector<btVector3> Triangle;
typedef std::vector<unsigned int> TriangleIndices;
typedef std::vector<Triangle> Triangles;
typedef std::multimap<std::string, MeshTools::Edge> EdgeMap;

namespace MeshTools
{

	class Bridge2dTo3dPoint
	{
		enum ZeroDimension
		{
			ZeroDim_X,
			ZeroDim_Y,
			ZeroDim_Z
		};

		ZeroDimension flatDimension;

	public:
		Bridge2dTo3dPoint(const std::vector<Edge>& edges, const std::vector<btVector3>& sharedVert);

		p2t::Point* convertVec3ToPoint(const btVector3& vec) const;
		btVector3 convertPointToVec3(p2t::Point* pt) const;
	};

	class Edge
	{
	public:
		unsigned int p0_i;
		unsigned int p1_i;

		bool operator==(Edge& b) { return (p0_i == b.p0_i && p1_i == b.p1_i); };
		bool operator!=(Edge& b) { return (p0_i != b.p0_i || p1_i != b.p1_i); };
	};


	class tIndex
	{
	public:
		unsigned int vI;  // vertex Index
		btVector3 normal; // Normal for the Vertex at that Index
		float uv_x;       // no btVector2, so UV's in Indices. Lame I know.
		float uv_y;
	};

	class TriangleMeshData
	{
	public:
		std::vector<btVector3> vertices;
		std::vector<unsigned int> indices;

		void addVertex(btVector3& newVert) { vertices.push_back(newVert); };
		const btVector3& getVertex(unsigned int index) { return vertices[index]; };

		void addIndex(unsigned int newIndex) { indices.push_back(newIndex); };
		const unsigned int getIndex(unsigned int index) { return indices[index]; };

		TriangleMeshData(const std::vector<btVector3>& vert, const std::vector<unsigned int>& indi) :
			vertices(vert),
			indices(indi)
		{};
		
		TriangleMeshData() :
			vertices(),
			indices()
		{};


	};

	class SplitVerticesResult
	{
	public:
		std::vector<btVector3> verticesLeft;
		std::vector<btVector3> verticesRight;
		SplitVerticesResult(const std::vector<btVector3>& left, const std::vector<btVector3>& right) :
			verticesLeft(left),
			verticesRight(right)
		{};
	};

	class SplitMeshResult
	{
	public:
		TriangleMeshData leftMesh;
		TriangleMeshData rightMesh;
		SplitMeshResult(const TriangleMeshData& left, const TriangleMeshData& right) :
			leftMesh(left),
			rightMesh(right)
		{};
	};

	class MeshTriangle
	{
	private:
		unsigned int a_index;
		unsigned int b_index;
		unsigned int c_index;

	public:
		unsigned int a() const { return a_index; };
		unsigned int b() const { return b_index; };
		unsigned int c() const { return c_index; };

		MeshTriangle(unsigned int a, unsigned int b, unsigned int c) :
			a_index(a),
			b_index(b),
			c_index(c)
		{};
		MeshTriangle() {};

	};


	class MeshTools
	{
	public:
		static SplitMeshResult SplitMeshConvexNoClose(	const TriangleMeshData& originalTriangleMesh,
															const btVector3& planeNormal,
															const btVector3& pointOnPlane);
		static SplitMeshResult SplitMeshSlow(	const TriangleMeshData& originalTriangleMesh,
												const btVector3& planeNormal,
												const btVector3& pointOnPlane);

		static int ClipTriangle(const MeshTriangle& triangle,						
								const btVector3& planeNormal,
								const btVector3& pointOnPlane,
								std::vector<btVector3>& sharedVertices,				// Container of Vertices to modify when we add new Triangles (shared between left and right)
								std::vector<unsigned int>& sharedIndices,           // Container of Indices to modify when we add new Triangles
								std::vector<MeshTriangle>& leftTrianglesOut,        // Triangles split to the left
								std::vector<MeshTriangle>& rightTrianglesOut,       // Triangles split to the right of plane
								EdgeMap& newEdgesFront,           // Adding new Edges to shape to later seal them with a face.
								EdgeMap& newEdgesBack);           // Have to also store them with Back as key in order to find them no matter the orientation

		static void CloseClippedMesh(	std::vector<btVector3>& sharedVertices,				// Container of Vertices to modify when we add new Triangles (shared between left and right)
										std::vector<unsigned int>& sharedIndices,           // Container of Indices to modify when we add new Triangles
										std::vector<MeshTriangle>& leftTrianglesOut,        // Triangles split to the left
										std::vector<MeshTriangle>& rightTrianglesOut,       // Triangles split to the right of plane
										EdgeMap& newEdgesFront,           // Adding new Edges to shape to later seal them with a face.
										EdgeMap& newEdgesBack);           // Have to also store them with Back as key in order to find them no matter the orientation)

		static bool iterateToNextEdge(	std::vector<btVector3>& sharedVertices, bool& lastEdgeFlipped,
										EdgeMap& newEdgesFront, EdgeMap& newEdgesBack, Edge& out);

		static TriangleMeshData assembleMeshFromSharedData(	const std::vector<btVector3>& sharedVert,
															const std::vector<MeshTriangle>& triangles);

		static void assemblyProcessTriangleVertex(const unsigned int& index, const std::vector<btVector3>& sharedVert, std::vector<unsigned int>& marker, TriangleMeshData& resultMesh);

		// UTILS
		static bool insertEdgeIfUnique(Edge& edge, EdgeMap& edgeMapFront, EdgeMap& edgeMapBack, const std::vector<btVector3>& sharedVertices);
		static bool flippedNormals(btVector3& oA, btVector3& oB, btVector3& oC, btVector3& pA, btVector3& pB, btVector3& pC);
		static bool uniqueSlopes(p2t::Point* origin, p2t::Point* pA, p2t::Point* pB);
		static std::string convertVecToStrKey(const btVector3& vector);
		static bool edgeIsSinglePoint(Edge& edge, const std::vector<btVector3>& sharedVertices);
		static bool compareEdgeVertices(Edge& e0, Edge& e1, const std::vector<btVector3>& sharedVertices);
		static bool edgesHasThisVertex(const btVector3& vertex, Edge& edge, const std::vector<btVector3>& sharedVertices, bool& equalIfFlipped);
		static bool edgesShareVertex(Edge& e0, Edge& e1, const std::vector<btVector3>& sharedVertices);
		static std::vector<p2t::Point*> getPolyLineFromEdges(std::vector<Edge> edges, const std::vector<btVector3>& sharedVertices, const Bridge2dTo3dPoint& converter);
		static void moveAndScalePointsToOrigin(std::vector<p2t::Point*>& points, btVector3& movedBy, float& scaledBy);
		static void undoScaleAndOriginMove(std::vector<p2t::Triangle*>& triangles, const btVector3& movedBy, const float& scaledBy);
		static void addP2tTrianglesToMeshes(std::vector<p2t::Triangle*>& triangles,
											std::vector<btVector3>& sharedVertices,				// Container of Vertices to modify when we add new Triangles (shared between left and right)
											std::vector<unsigned int>& sharedIndices,           // Container of Indices to modify when we add new Triangles
											std::vector<MeshTriangle>& leftTrianglesOut,        // Triangles split to the left
											std::vector<MeshTriangle>& rightTrianglesOut,
											const Bridge2dTo3dPoint& converter);

		static int ClipTriangle(const Triangle& triangle, const btVector3& planeNormal, const btVector3& pointOnPlane, Triangles& leftTrianglesOut, Triangles& rightTrianglesOut);
		static float distRayPlane(const btVector3& vStart, const btVector3& vEnd, const btVector3& vnPlaneNormal, const btVector3& vPointOnPlane);
		static btVector3 PointOnPlane(const btVector3& vStart, const btVector3& vEnd, const btVector3& vPlaneNormal, const btVector3& vPointOnPlane);
	};

}//namespace MeshTools
