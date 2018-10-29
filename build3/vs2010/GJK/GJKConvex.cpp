#include "GJKConvex.h"
#include "btQuaternion.h"

namespace ProjGJK
{
	const btVector3 GJKConvexHull::getSupport(const btVector3& direction)
	{
		//TODO: Optimize this to use EDGES to find best point. For now, we use brute force O(n) method.
		const btVector3 localDirection = quatRotate(getTransform().getRotation().inverse(), direction);

		btVector3 vecOut = vertices[0];
		float maxDot = vecOut.dot(localDirection);

		for (int i = 1; i < vertices.size(); i++)
		{
			const btVector3& testVec = vertices[i];
			float dot = testVec.dot(localDirection);
			if (dot > maxDot)
			{
				maxDot = dot;
				vecOut = testVec;
			}
		}

		return getTransform() * vecOut;
	}
};