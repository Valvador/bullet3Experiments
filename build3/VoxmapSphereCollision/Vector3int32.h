#pragma once

#include <cstdlib>

namespace VSC
{
class Vector3int32
{
public:
	int x;
	int y;
	int z;

	Vector3int32() : x(0), y(0), z(0) {}
	Vector3int32(int v) : x(v), y(v), z(v) {}
	Vector3int32(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}

	const int& operator[] (int i) const
	{
		return ((int*)this)[i];
	}

	int& operator[] (int i)
	{
		return ((int*)this)[i];
	}
};
};
