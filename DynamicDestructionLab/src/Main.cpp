#include "MeshTools\MeshTools.h"

int main()
{
	Triangle testTriangle;
	testTriangle.push_back(btVector3(-1.0, 0, -1.0));
	testTriangle.push_back(btVector3(1.0, 0, -1.0));
	testTriangle.push_back(btVector3(0.0, 0, 1.0));

	btVector3 plane(-1.0, 0, 0);
	btVector3 pointOnPlane(-0.1, 0, 0);

	Triangles leftTriangles;
	Triangles rightTriangles;
	MeshTools::ClipTriangle(testTriangle, plane, pointOnPlane, leftTriangles, rightTriangles);

	printf("Left Triangle Count: %d \n", leftTriangles.size());
	printf("Right Triangle Count: %d \n", rightTriangles.size());
}