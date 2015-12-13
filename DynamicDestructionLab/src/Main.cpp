#include "MeshTools\MeshTools.h"
#include "LinearMath\btVector3.h"

int main()
{
	// TEST MESH SPLITTING SIMPLE
	//   B______D
	//   /\    /\
	//  /  \  /  \
	//A/    \/    \E
	// ¯¯¯¯¯¯C¯¯¯¯¯
	std::vector<btVector3> meshVert = { btVector3(-1, 0, 0),
										btVector3(-.5, 0, 1),
										btVector3(0, 0, 0),
										btVector3(.5, 0, 1),
										btVector3(1, 0, 0) };

	std::vector<unsigned int> meshInd = {	0, 1, 2,
											1, 2, 3,
											2, 3, 4 };

	const MeshTools::TriangleMeshData mesh(meshVert, meshInd);
	btVector3 plane(-1, 0, 0);
	btVector3 pointOnPlane(-0.8, 0, 0);

	MeshTools::SplitMeshResult result = MeshTools::MeshTools::SplitMeshSlow(mesh, plane, pointOnPlane);
	printf("Left Mesh Triangles: \n");
	std::vector<btVector3> vertices = result.leftMesh.vertices;
	std::vector<unsigned int> indices = result.leftMesh.indices;
	for (unsigned int i = 0; i < indices.size(); i += 3)
	{
		printf("%f, %f, %f \n", vertices[indices[i]].x(), vertices[indices[i]].y(), vertices[indices[i]].z());
		printf("%f, %f, %f \n", vertices[indices[i+1]].x(), vertices[indices[i+1]].y(), vertices[indices[i+1]].z());
		printf("%f, %f, %f \n", vertices[indices[i+2]].x(), vertices[indices[i+2]].y(), vertices[indices[i+2]].z());
	}

	printf("\n \n \nRightMesh Mesh Triangles: \n");
	vertices = result.rightMesh.vertices;
	indices = result.rightMesh.indices;
	for (unsigned int i = 0; i < indices.size(); i += 3)
	{
		printf("%f, %f, %f \n", vertices[indices[i]].x(), vertices[indices[i]].y(), vertices[indices[i]].z());
		printf("%f, %f, %f \n", vertices[indices[i + 1]].x(), vertices[indices[i + 1]].y(), vertices[indices[i + 1]].z());
		printf("%f, %f, %f \n", vertices[indices[i + 2]].x(), vertices[indices[i + 2]].y(), vertices[indices[i + 2]].z());
	}

	return 0;
}
