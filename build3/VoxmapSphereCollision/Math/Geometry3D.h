#pragma once
#include "Vector3.h"
#include <limits>
#include <cmath>

namespace VSC
{
class Geometry3D
{
public:
	struct Plane
	{
	public:
		Vector3 normal;
		Vector3 point;
	};

	static bool planeAABBIntersect(const Plane& p, const Vector3& aabbMin, const Vector3& aabbMax)
	{
		// TODO: IMPLEMENT THIS
		return true;
	}

	static bool triangleAABBIntersect(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& aabbMin, const Vector3& aabbMax)
	{
		Vector3 aabbCenter = (aabbMin + aabbMax) * 0.5f;
		Vector3 extents = (aabbMax - aabbMin) * 0.5f;

		// Translate coordinates to AABB origin
		Vector3 v0t = v0 - aabbCenter;
		Vector3 v1t = v1 - aabbCenter;
		Vector3 v2t = v2 - aabbCenter;

		// Edge slopes
		Vector3 s01 = v1t - v0t;
		Vector3 s12 = v2t - v1t;
		Vector3 s02 = v2t - v0t;

		// Test axis X, Y, Z
		float p0 = v0t.z * v1t.y - v0t.y*v1t.z;
		float p2 = v2t.z * (v1t.y - v0t.y) - v2t.y * (v1t.z - v0t.z);
		float r = extents.y * std::fabs(s01.z) + extents.z * std::fabs(s01.y);
		if (std::fmaxf(-std::fmaxf(p0, p2), std::fminf(p0, p2)) > r)
			return false;

		// TODO: 9 MORE AXES

		return true;
	}
};

};
