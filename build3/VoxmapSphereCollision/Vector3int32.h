#pragma once

#include <stdint.h>

namespace VSC
{
class Vector3int32
{
public:
	int32_t x;
	int32_t y;
	int32_t z;

	Vector3int32() : x(0), y(0), z(0) {}
	Vector3int32(int32_t v) : x(v), y(v), z(v) {}
	Vector3int32(int32_t _x, int32_t _y, int32_t _z) : x(_x), y(_y), z(_z) {}

	const int32_t& operator[] (int i) const
	{
		return ((int32_t*)this)[i];
	}

	int32_t& operator[] (int i)
	{
		return ((int32_t*)this)[i];
	}

	bool operator==(const Vector3int32& other)
	{
		return x == other.x && y == other.y && z == other.z;
	}
};
};
