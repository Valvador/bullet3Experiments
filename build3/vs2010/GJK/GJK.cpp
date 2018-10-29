#include <assert.h>
#include <limits>

#include "GJK.h"
#include "GJKConvex.h"

int maxGJKIterations = 1000;
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
	
	bool GJKPair::doGJK()
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
				// INSERT EPA HERE
				return true;
			}
		}

		return false;
	}
};