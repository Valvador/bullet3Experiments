#pragma once
#include "Vector3.h"
#include <cmath>
#include <limits>
#include <vector>

namespace VSC
{
class Geometry2D
{
public:
	struct LineSegment2D
	{
	public:
		Vector2 a;
		Vector2 b;

		LineSegment2D(const Vector2& _a, const Vector2& _b) : a(_a), b(_b) {};

		Vector2 slope() const { return b - a; }
	};

	static bool lineSegmentIntersect(const LineSegment2D& seg0, const LineSegment2D& seg1, Vector2& result)
	{
		Vector2 slope0 = seg0.slope();
		Vector2 slope1 = seg1.slope();
		float angle = slope0.cross(slope1);

		if (std::fabs(angle) < std::numeric_limits<float>::epsilon())
			return false;

		Vector2 aToA = seg1.a - seg0.a;
		float t = aToA.cross(slope1) / angle;
		if (t < 0 || t > 1.0)
		{
			return false;
		}

		float u = aToA.cross(slope0) / angle;
		if (u < 0 || u > 1)
		{
			return false;
		}

		result = seg0.a + slope0 * t;
		return true;
	}

	static bool pointInsideAABB(const Vector2& pt, const Vector2& aabbMin, const Vector2& aabbMax)
	{
		return (pt.x >= aabbMin.x && pt.x <= aabbMax.x && pt.y >= aabbMin.y && pt.y <= aabbMax.y);
	}

	static bool lineSegmentAABBIntersect(const LineSegment2D& seg, const Vector2& aabbMin, const Vector2& aabbMax)
	{
		// Check if either of segments points are in the AABB
		if (pointInsideAABB(seg.a, aabbMin, aabbMax) || pointInsideAABB(seg.b, aabbMin, aabbMax))
			return true;


		LineSegment2D left(aabbMin, Vector2(aabbMin.x, aabbMax.y)); // Edge from bottom left, to top left
		LineSegment2D bottom(aabbMin, Vector2(aabbMax.x, aabbMin.y)); // Edge from bottom left, to bottom right
		LineSegment2D right(Vector2(aabbMax.x, aabbMin.y), aabbMax); // Edge from bottom right, to top right
		LineSegment2D top(Vector2(aabbMin.x, aabbMax.y), aabbMax); // Edge from top left, to top right

		Vector2 placeholder;
		return	lineSegmentIntersect(seg, left, placeholder) ||
				lineSegmentIntersect(seg, bottom, placeholder) ||
				lineSegmentIntersect(seg, right, placeholder) ||
				lineSegmentIntersect(seg, top, placeholder);
	}

	static bool lineSegmentsAABBIntersect(LineSegment2D* segments, int32_t numSegments, const Vector2& aabbMin, const Vector2& aabbMax)
	{
		for (int32_t i = 0; i < numSegments; i++)
		{
			if (lineSegmentAABBIntersect(segments[i], aabbMin, aabbMax))
			{
				return true;
			}
		}

		return false;
	}
};

};