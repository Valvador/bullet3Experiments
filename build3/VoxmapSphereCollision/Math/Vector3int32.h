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

	struct Hash
	{
		size_t operator()(const Vector3int32& k) const
		{
			/*
			See: http://graphics.ethz.ch/Downloads/Publications/Papers/2003/tes03/p_tes03.pdf
			"Optimized Spatial Hashing for Collision Detection of Deformable Objects"

			hash functions. It gets three values, describing
			vertex position (x, y, z), and returns a hash
			hash(x,y,z) = ( x p1 xor y p2 xor z p3) mod
			where p1, p2, p3 are large prime numbers,
			our case 73856093, 19349663, 83492791, respectively.
			The value n is the hash table size.
			*/
			uint32_t xProd = uint32_t(k.x) * 73856093;
			uint32_t yProd = uint32_t(k.y) * 19349663;
			uint32_t zProd = uint32_t(k.z) * 83492791;
			return size_t(xProd ^ yProd ^ zProd);
		}
	};

	const int32_t& operator[] (int i) const
	{
		return ((int32_t*)this)[i];
	}

	int32_t& operator[] (int i)
	{
		return ((int32_t*)this)[i];
	}

	bool operator==(const Vector3int32& other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}
};
};
