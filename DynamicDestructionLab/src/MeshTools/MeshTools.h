#include "LinearMath/btVector3.h"

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
};