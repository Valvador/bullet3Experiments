#include <assert.h>
#include <limits>

#include "GJK.h"
#include "GJKConvex.h"

const static int maxGJKIterations = 1000;
const static float epaTolerance = 0.0001f;
const static int epaMaxNumFace = 64;
const static int epaMaxNumLooseEdges = 32;
const static int epaMaxNumIterations = 64;
/// THIS CODE USES https://github.com/kevinmoran/GJK/blob/master/GJK.h AS REFERENCE
/// 

namespace ProjGJK
{
	void GJKSimplex::updateSimplexVertex(int i, const btVector3& vertex)
	{
		assert(i < 4);
		vertices[i] = vertex;
	}

	void GJKSimplex::updateSimplex3(int &simpDim, btVector3& searchDir)
	{
		/* Required winding order:
		//  b
		//  | \
		//  |   \
		//  |    a
		//  |   /
		//  | /
		//  c
		*/
		btVector3 bSubA = vertices[1] - vertices[0];
		btVector3 cSubA = vertices[2] - vertices[0];
		btVector3 n = (bSubA).cross(cSubA); // Triangle Normal
		btVector3 A0 = -vertices[0];												// Direction to Origin

		simpDim = 2;
		if (bSubA.cross(n).dot(A0) > 0.0f) //Closest to edge AB
		{
			vertices[2] = vertices[0];
			searchDir = bSubA.cross(A0).cross(bSubA);
			return;
		}

		if (n.cross(cSubA).dot(A0) > 0.0f) //Closest to edge AC
		{
			vertices[1] = vertices[0];
			searchDir = cSubA.cross(A0).cross(cSubA);
		}

		simpDim = 3;
		if (n.dot(A0) > 0.0f) // Above Triangle
		{
			vertices[3] = vertices[2];
			vertices[2] = vertices[1];
			vertices[1] = vertices[0];
			searchDir = n;
		}

		// Below Triangle
		vertices[3] = vertices[1];
		vertices[1] = vertices[0];
		searchDir = -n;
	}

	bool GJKSimplex::updateSimplex4CheckInside(int &simpDim, btVector3& searchDir)
	{
		// Normals of 3 Faces
		btVector3 n012 = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
		btVector3 n023 = (vertices[2] - vertices[0]).cross(vertices[3] - vertices[0]);
		btVector3 n031 = (vertices[3] - vertices[0]).cross(vertices[1] - vertices[0]);

		btVector3 A0 = -vertices[0];
		simpDim = 3;

		if (n012.dot(A0) > 0.0f)
		{
			vertices[3] = vertices[2];
			vertices[2] = vertices[1];
			vertices[1] = vertices[0];
			searchDir = n012;
			return false;
		}
		if (n023.dot(A0) > 0.0f)	
		{
			vertices[1] = vertices[0];
			searchDir = n023;
			return false;
		}
		if (n031.dot(A0) > 0.0f)
		{
			vertices[2] = vertices[3];
			vertices[3] = vertices[1];
			vertices[1] = vertices[0];
			searchDir = n031;
			return false;
		}

		// POINT INSIDE SIMPLEX
		return true;
	}

	GJKContactResult GJKSimplex::doEPAForSimplexContact(GJKConvex* shape0, GJKConvex* shape1)
	{
		GJKContactResult result;
		
		btVector3 faces[epaMaxNumFace][3];
		btVector3 faceNorms[epaMaxNumFace];
		faces[0][0] = vertices[0];
		faces[0][1] = vertices[1];
		faces[0][2] = vertices[2];
		faceNorms[0] = ((vertices[1] - vertices[0]).cross(vertices[2] - vertices[0])).normalized();
		faces[1][0] = vertices[0];
		faces[1][1] = vertices[2];
		faces[1][2] = vertices[3];
		faceNorms[1] = ((vertices[2] = vertices[0]).cross(vertices[3] - vertices[0])).normalized();
		faces[2][0] = vertices[0];
		faces[2][1] = vertices[3];
		faces[2][2] = vertices[1];
		faceNorms[2] = ((vertices[3] = vertices[0]).cross(vertices[1] - vertices[0])).normalized();
		faces[3][0] = vertices[1];
		faces[3][1] = vertices[3];
		faces[3][2] = vertices[2];
		faceNorms[3] = ((vertices[3] = vertices[1]).cross(vertices[2] - vertices[1])).normalized();

		int numFaces = 4;
		int closestFace;
		btVector3 closestSupport1;
		btVector3 closestSupport0;

		//EPA ITERATIONS
		for (int iterations = 0; iterations < epaMaxNumIterations; iterations++)
		{
			float minDist = faces[0][0].dot(faceNorms[0]);
			closestFace = 0;

			for (int i = 1; i < numFaces; i++)
			{
				float dist = faces[i][0].dot(faceNorms[i]);
				if (dist < minDist)
				{
					minDist = dist;
					closestFace = i;
				}
			}

			btVector3 searchDirection = faceNorms[closestFace];
			btVector3 closestSupport1 = shape1->getSupport(searchDirection);
			btVector3 closestSupport0 = shape0->getSupport(-searchDirection);
			btVector3 point = shape1->getSupport(searchDirection) - shape0->getSupport(-searchDirection);

			if (point.dot(searchDirection) - minDist < epaTolerance)
			{
				// Convergence, (Close to Origin)
				result.depth = minDist;
				result.normal = searchDirection;
				result.position = closestSupport1;
				return result;
			}

			// TODO: May optimize Below with Unordered Set?
			btVector3 looseEdges[epaMaxNumLooseEdges][2]; // Keep Track of edges we need to deal with after removing Faces
			int numLooseEdges = 0;

			// Find All Triangles facing Point
			for (int i = 0; i < numFaces; i++)
			{
				if (faceNorms[i].dot(point - faces[i][0]) > 0.0f)
				{
					// Add removed triangles edges to loose edge list
					// If already in list, remove.
					for (int j = 0; j < 3; j++) // Three Edges per Face/Tri
					{
						btVector3 currentEdge[2] = { faces[i][j], faces[i][(j + 1) % 3] };
						bool foundEdge = false;
						for (int k = 0; k < numLooseEdges; k++)
						{
							if (looseEdges[k][1] == currentEdge[0] && looseEdges[k][0] == currentEdge[1])
							{
								// Edge in List, remove.
								// Assumes Edges can only be shared by 2 Triangles.
								// Assumes edge will be reversed.
								// Requires triangles wound CCW
								looseEdges[k][0] = looseEdges[numLooseEdges - 1][0];
								looseEdges[k][1] = looseEdges[numLooseEdges - 1][1];
								numLooseEdges--;
								foundEdge = true;
								k = numLooseEdges;
							}
						}

						if (!foundEdge) // Only add to Edge if not already found.
						{
							if (numLooseEdges >= epaMaxNumIterations)
								break;

							looseEdges[numLooseEdges][0] = currentEdge[0];
							looseEdges[numLooseEdges][1] = currentEdge[1];
							numLooseEdges++;
						}
					}

					//Remove triangle i from list
					faces[i][0] = faces[numFaces - 1][0];
					faces[i][1] = faces[numFaces - 1][1];
					faces[i][2] = faces[numFaces - 1][2];
					faceNorms[i] = faceNorms[numFaces - 1];
					numFaces--;
					i--;
				}
			}

			//Reconstruct Polytope with Point added
			for (int i = 0; i < numLooseEdges; i++)
			{
				if (numFaces > epaMaxNumFace)
					break;

				faces[numFaces][0] = looseEdges[i][0];
				faces[numFaces][1] = looseEdges[i][1];
				faces[numFaces][2] = point;
				faceNorms[numFaces] = ((faces[numFaces][0] - faces[numFaces][1]).cross(faces[numFaces][0] - point)).normalized();

				// Check for wrong CCW winding
				float bias = 0.0000001f;
				if ((faces[numFaces][0].dot(faceNorms[numFaces]) + bias) < 0.0f) 
				{
					btVector3 temp = faces[numFaces][0];
					faces[numFaces][0] = faces[numFaces][1];
					faces[numFaces][1] = temp;
					faceNorms[numFaces] = -faceNorms[numFaces];
				}
				numFaces++;
			}
		}

		assert(false);
		result.position = closestSupport1;
		result.normal = faceNorms[closestFace];
		result.depth = faces[closestFace][0].dot(faceNorms[closestFace]);
		return result;
	}
	
	bool GJKPair::doGJK(GJKContactResult* resultOut)
	{
		// Search Direction - Can Cache this for Optimized
		btVector3 searchDir = obj0->getTransform().getOrigin() - obj1->getTransform().getOrigin();

		// Get Initial Simplex Points
		GJKSimplex simplex;
		simplex.updateSimplexVertex(2, (obj1->getSupport(searchDir) - obj0->getSupport(-searchDir)));
		searchDir = -simplex.vertices[2];
		simplex.updateSimplexVertex(1, (obj1->getSupport(searchDir) - obj0->getSupport(-searchDir)));

		if (simplex.vertices[1].dot(searchDir) < 0)
		{
			return false; // Didn't reach Origin, won't enclose
		}

		// Search Perpendicular to line segment towards origin
		searchDir = (simplex.vertices[2] - simplex.vertices[1]).cross(-simplex.vertices[1]).cross(simplex.vertices[2] - simplex.vertices[1]);
		if (searchDir.length2() < std::numeric_limits<float>::epsilon()) // Normal is almost on Line Segment
		{
			searchDir = (simplex.vertices[2] - simplex.vertices[1]).cross(btVector3(1.0f, 0.0f, 0.0f));			//Normal with X
			if (searchDir.length2() < std::numeric_limits<float>::epsilon())
			{
				searchDir = (simplex.vertices[2] - simplex.vertices[1]).cross(btVector3(0.0f, 0.0f, -1.0f));	//Normal with -Z
			}
		}

		int simpDim = 2; // Current Dimension of Simplex
		for (int i = 0; i < maxGJKIterations; i++)
		{
			simplex.vertices[0] = obj1->getSupport(searchDir) - obj0->getSupport(-searchDir);
			if (simplex.vertices[0].dot(searchDir) < 0.0f)
			{
				return false;
			}

			simpDim++;
			if (simpDim == 3)
			{
				simplex.updateSimplex3(simpDim, searchDir);
			}
			else if (simplex.updateSimplex4CheckInside(simpDim, searchDir))
			{
				if (resultOut)
				{
					// INSERT EPA HERE
					*resultOut = simplex.doEPAForSimplexContact(obj0, obj1);
				}
				return true;
			}
		}

		return false;
	}
};