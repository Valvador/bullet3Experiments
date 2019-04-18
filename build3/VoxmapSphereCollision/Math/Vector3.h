#pragma once

namespace VSC
{
class Vector3
{
public:
	Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {};
	Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {};
	Vector3(float v) : x(v), y(v), z(v) {};
	Vector3() {}; // Uninitialized

	float x, y, z;
    // Operators
	const float& operator[](int i) const
	{
		return ((float*)(this))[i];
	}

	float& operator[](int i)
	{
		return ((float*)(this))[i];
	}

	Vector3 operator*(const Vector3& other) const
	{
		return Vector3(x*other.x, y*other.y, z*other.z);
	}

	Vector3 operator*(float scale) const
	{
		return Vector3(x*scale, y*scale, z*scale);
	}

	Vector3 operator+(const Vector3& other) const
	{
		return Vector3(x + other.x, y + other.y, z + other.z);
	}

	Vector3 operator-(const Vector3& other) const
	{
		return Vector3(x - other.x, y - other.y, z - other.z);
	}

	void operator*=(float scale)
	{
		x *= scale;
		y *= scale;
		z *= scale;
	}

	void operator*=(const Vector3& other)
	{
		x *= other.x;
		y *= other.y;
		z *= other.z;
	}

	// Operations
	Vector3 cross(const Vector3& other) const
	{
		return Vector3(	y * other.z - z * other.y,
						z * other.x - x * other.z,
						x * other.y - y * other.x);
	}

	float dot(const Vector3& other) const
	{
		return x * other.x + y * other.y + z * other.z;
	}

	void setMinAxis(const Vector3& other)
	{
		if (other[0] < x)
		{
			x = other[0];
		}
		if (other[1] < y)
		{
			y = other[1];
		}
		if (other[2] < z)
		{
			z = other[2];
		}
	}

	void setMaxAxis(const Vector3& other)
	{
		if (other[0] > x)
		{
			x = other[0];
		}
		if (other[1] > y)
		{
			y = other[1];
		}
		if (other[2] > z)
		{
			z = other[2];
		}
	}
};
};