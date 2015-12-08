#include "LinearMath/btVector3.h"
#include <vector>

typedef std::vector<btVector3> Triangle;
typedef std::vector<unsigned int> TriangleIndices;
typedef std::vector<Triangle> Triangles;

class TriangleMeshData
{
public:
	unsigned int numVertices;
	btVector3* vertices;

	unsigned int numIndices;
	unsigned int* indices;

	TriangleMeshData() :
		numVertices(0),
		vertices(NULL),
		numIndices(0),
		indices(NULL)
	{};


};


class MeshTools
{
public:

	struct SplitMeshResult
	{
		TriangleMeshData* left;
		TriangleMeshData* right;
		TriangleMeshData* mid;
	};

	static SplitMeshResult splitMeshZ(TriangleMeshData* oldMesh, float zCut, float zInsert, bool marginalTrisInLeft, bool marginalTrisInRight);
	static int ClipTriangle(const Triangle& triangle, const btVector3& planeNormal, const btVector3& pointOnPlane, Triangles& leftTrianglesOut, Triangles& rightTrianglesOut);
	static float distRayPlane(const btVector3& vStart, const btVector3& vEnd, const btVector3& vnPlaneNormal, const btVector3& vPointOnPlane);
	static btVector3 PointOnPlane(const btVector3& vStart, const btVector3& vEnd, const btVector3& vPlaneNormal, const btVector3& vPointOnPlane);
};