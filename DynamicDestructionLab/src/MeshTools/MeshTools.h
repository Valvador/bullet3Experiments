#include "LinearMath/btVector3.h"
#include <vector>

typedef std::vector<btVector3> Triangle;
typedef std::vector<unsigned int> TriangleIndices;
typedef std::vector<Triangle> Triangles;

namespace MeshTools
{
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

	};


	class MeshTools
	{
	public:

		static SplitMeshResult SplitMeshSlow(	const TriangleMeshData& originalTriangleMesh,
												const btVector3& planeNormal,
												const btVector3& pointOnPlane);

		static int ClipTriangle(const MeshTriangle& triangle,						
								const btVector3& planeNormal,
								const btVector3& pointOnPlane,
								std::vector<btVector3>& sharedVertices,				// Container of Vertices to modify when we add new Triangles (shared between left and right)
								std::vector<unsigned int>& sharedIndices,           // Container of Indices to modify when we add new Triangles
								std::vector<MeshTriangle>& leftTrianglesOut,        // Triangles split to the left
								std::vector<MeshTriangle>& rightTrianglesOut);      // Triangles split to the right of plane

		static int ClipTriangle(const Triangle& triangle, const btVector3& planeNormal, const btVector3& pointOnPlane, Triangles& leftTrianglesOut, Triangles& rightTrianglesOut);
		static float distRayPlane(const btVector3& vStart, const btVector3& vEnd, const btVector3& vnPlaneNormal, const btVector3& vPointOnPlane);
		static btVector3 PointOnPlane(const btVector3& vStart, const btVector3& vEnd, const btVector3& vPlaneNormal, const btVector3& vPointOnPlane);
	};

}//namespace MeshTools