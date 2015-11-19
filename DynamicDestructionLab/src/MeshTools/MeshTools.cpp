//#include "stdafx.h"
#include "MeshTools.h"

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
}