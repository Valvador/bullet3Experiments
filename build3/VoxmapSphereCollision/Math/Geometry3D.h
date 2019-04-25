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
		float distance;
	};

	static float max3(float f0, float f1, float f2)
	{
		return std::fmaxf(std::fmaxf(f0, f1), f2);
	}

	static float min3(float f0, float f1, float f2)
	{
		return std::fminf(std::fminf(f0, f1), f2);
	}

	static bool planeAABBIntersect(const Plane& p, const Vector3& aabbMin, const Vector3& aabbMax)
	{
		// AABB center and extents
		Vector3 aabbCenter = (aabbMin + aabbMax) * 0.5f;
		Vector3 extents = (aabbMax - aabbCenter);

		// Projection Interval
		Vector3 planeNormalAbs = Vector3(std::fabs(p.normal.x), std::fabs(p.normal.y), std::fabs(p.normal.z));
		float r = extents.dot(planeNormalAbs);
		// Distance of AABB center from plane
		float distFromPlane = p.normal.dot(aabbCenter) - p.distance;

		return std::fabs(distFromPlane) <= r;
	}

	static bool triangleAABBIntersect(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& aabbMin, const Vector3& aabbMax)
	{
		Vector3 aabbCenter = (aabbMin + aabbMax) * 0.5f;
		Vector3 extents = (aabbMax - aabbMin) * 0.5f;

		// Translate coordinates to AABB origin
		Vector3 v0t = v0 - aabbCenter;
		Vector3 v1t = v1 - aabbCenter;
		Vector3 v2t = v2 - aabbCenter;

		// Test 3 axes corresponding to face norms of AABB.
		if (max3(v0t.x, v1t.x, v2t.x) < -extents.x || min3(v0t.x, v1t.x, v2t.x) > extents.x)
			return false;
		if (max3(v0t.y, v1t.y, v2t.y) < -extents.y || min3(v0t.y, v1t.y, v2t.y) > extents.y)
			return false;
		if (max3(v0t.z, v1t.z, v2t.z) < -extents.z || min3(v0t.z, v1t.z, v2t.z) > extents.z)
			return false;

		// Test 9 axes of forall[basisAxes x edgeSlope]
		// u0 = (1, 0, 0), u1 = (0, 1, 0), u2 = (0, 0, 1)
		// -------------------------------
		// a00 = u0 x s0 = (0, -edge[0].z, edge[0].y), p0 = v0.dot(a00), p1 = v1.dot(a00), p2 = v2.dot(a00)
		// a01 = u0 x s1 = (0, -edge[1].z, edge[1].y)
		// ...
		// a10 = u1 x s0 = (edge[0].z, 0, -edge[0].x)
		// ...
		// a22 = u2 x s2 = (-edge[2].y, edge[2].x, 0)
		static const Vector3 u[3] = { Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1) };
		Vector3 edge[3] = { v1t - v0t, v2t - v1t, v2t - v0t };

#ifdef TRIANGLE_AABB_USE_GENERIC
		// THIS IS GENERIC FORMULATION OF THE BELOW 9 AXES TESTS.
		// THIS CODE DOES A LOT OF OPERATION ON AXES THAT ARE ESSENTIALLY ZERO
		// DUE TO THIS WE HAVE TO WRITE UGLY CODE THAT HAS WAY FEWER OPERATIONS.
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Vector3 axis = u[i].cross(edge[j]);
				float r = extents.dot(axis.abs());
				float p0 = v0t.dot(axis);
				float p1 = v1t.dot(axis);
				float p2 = v2t.dot(axis);
				if (std::fmaxf(-(max3(p0, p2, p1)), min3(p0, p2, p1)) > r)
					return false;
			}
		}
#else
		{
			// Convert this problem to 2D space, and then to 1D distance space.
			Vector2 vProj[3]; // Triangle vertices projected onto plane.
			Vector2 sProj[3]; // Separating axis projected.
			Vector2 extProj; // Extents projected
			for (int i = 0; i < 3; i++)
			{
				if (i == 0)
				{
					vProj[0] = v0t.yz(); sProj[0] = Vector2(-edge[0].z, edge[0].y);
					vProj[1] = v1t.yz(); sProj[1] = Vector2(-edge[1].z, edge[1].y);
					vProj[2] = v2t.yz(); sProj[2] = Vector2(-edge[2].z, edge[2].y);
					extProj = extents.yz();
				}
				else if (i == 1)
				{
					vProj[0] = v0t.xz(); sProj[0] = Vector2(edge[0].z, -edge[0].x);
					vProj[1] = v1t.xz(); sProj[1] = Vector2(edge[1].z, -edge[1].x);
					vProj[2] = v2t.xz(); sProj[2] = Vector2(edge[2].z, -edge[2].x);
					extProj = extents.xz();
				}
				else
				{
					vProj[0] = v0t.xy(); sProj[0] = Vector2(-edge[0].y, edge[0].x);
					vProj[1] = v1t.xy(); sProj[1] = Vector2(-edge[1].y, edge[1].x);
					vProj[2] = v2t.xy(); sProj[2] = Vector2(-edge[2].y, edge[2].x);
					extProj = extents.xy();
				}

				for (int j = 0; j < 3; j++)
				{
					// If you look at the GENERIC commented out implementation, we actually have 3 projections:
					// p0, p1, p2. Due to the AABB being axis aligned, dot products with crosses
					// we end up with one of the p values being a duplicate. We reduce this to speed up the calc.
					// every j (edge) we have 2 different pN's cancelling out.
					// edge[0], v1 - v0, p0 == p1, p0 and p2 is all we need
					// edge[1], v2 - v1, p2 == p1, p0 and p2 is all we need
					// edge[2], v2 - v0, p2 == p0, swap p0 value for p1.

					float p0 = j != 2 ? vProj[0].dot(sProj[j]) : vProj[1].dot(sProj[j]);
					float p2 = vProj[2].dot(sProj[j]);
					float r = extProj.dot(sProj[j].abs());
					if (std::fmaxf(-std::fmaxf(p0, p2), std::fminf(p0, p2)) > r)
						return false;
				}
			}
		}
#endif

		// Final, Triangle Face Normal test
		Plane tFacePlane;
		tFacePlane.normal = (edge[0].cross(edge[1])).normalized();
		tFacePlane.distance = tFacePlane.normal.dot(v0t);
		// this plane is already assuming the center of the aabb is our origin,
		// we have to adjust aabbMin and aabbMax to get correct planeAABBIntersect results.
		return planeAABBIntersect(tFacePlane, aabbMin - aabbCenter, aabbMax - aabbCenter);
	}
};

};
