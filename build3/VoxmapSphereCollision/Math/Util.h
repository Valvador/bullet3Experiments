#pragma once

namespace VSC
{
class MathUtil
{
public:
	static const double pi()
	{
		return 3.141592653589793238462643383279502884;
	}

	static float degreesToRadians(float degrees)
	{
		return degrees * float(pi()) / 180.0f;
	}

	static float radiansToDegrees(float radians)
	{
		return radians * 180 / float(pi());
	}
};

};