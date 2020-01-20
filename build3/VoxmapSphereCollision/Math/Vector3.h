#pragma once
#include <limits>
#include <cmath>


namespace VSC
{
class Vector2
{
public:
	Vector2(const Vector2& other) : x(other.x), y(other.y) {};
	Vector2(float _x, float _y) : x(_x), y(_y) {};
	Vector2(float v) : x(v), y(v) {};
	Vector2() {}; // Uninitialized

	float x, y;

	// Operators
	Vector2 operator*(const Vector2& other) const
	{
		return Vector2(x*other.x, y*other.y);
	}

	Vector2 operator*(float scale) const
	{
		return Vector2(x*scale, y*scale);
	}

	Vector2 operator+(const Vector2& other) const
	{
		return Vector2(x + other.x, y + other.y);
	}

	Vector2 operator-(const Vector2& other) const
	{
		return Vector2(x - other.x, y - other.y);
	}

	//Operations
	float cross(const Vector2& other) const
	{
		return x * other.y - y * other.x;
	}

	float dot(const Vector2& other) const
	{
		return x * other.x + y * other.y;
	}

	//Misc
	Vector2 abs() const
	{
		return Vector2(std::fabs(x), std::fabs(y));
	}

	float sqrMagnitude() const
	{
		return x*x + y*y;
	}

	bool fuzzyEquals(const Vector2& other, float eps = std::numeric_limits<float>::epsilon()) const
	{
		return (*this - other).sqrMagnitude() <= (eps * eps);
	}

	Vector2 normalized() const
	{
		return *this * (1.0f / (sqrtf(sqrMagnitude())));
	}
};

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

	void operator+=(const Vector3& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
	}

	bool operator==(const Vector3& other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}

	Vector2 xy() const
	{
		return Vector2(x, y);
	}

	Vector2 xz() const
	{
		return Vector2(x, z);
	}

	Vector2 yz() const
	{
		return Vector2(y, z);
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

	// Misc
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

	Vector3 abs()
	{
		return Vector3(std::fabs(x), std::fabs(y), std::fabs(z));
	}

	float sqrMagnitude() const
	{
		return x*x + y*y + z*z;
	}

	bool fuzzyEquals(const Vector3& other, float eps = std::numeric_limits<float>::epsilon()) const
	{
		return (*this - other).sqrMagnitude() <= (eps * eps);
	}

	Vector3 normalized() const
	{
		return *this * (1.0f / (sqrtf(sqrMagnitude())));
	}

	float maxValue() const
	{
		return std::fmaxf(std::fmaxf(x, y), z);
	}
};
};