#pragma once
#include "Vector3.h"
#include "Util.h"

namespace VSC
{
class Quaternion
{
private:
	float w;
	Vector3 v;

public:
	// Getters
	float getScalarW() const { return w; }
	const Vector3& getVectorIJK() const { return v; }

	Quaternion() {};
	Quaternion(float _w, const Vector3& _v) :
		w(_w),
		v(_v)
	{}

	// Operators
	void operator+=(const Quaternion& q)
	{
		w += q.w;
		v += q.v;
	}

	Quaternion operator+(const Quaternion& q) const
	{
		return Quaternion(w + q.w, v + q.v);
	}

	void operator-=(const Quaternion& q)
	{
		w -= q.w;
		v -= q.v;
	}

	Quaternion operator-(const Quaternion& q) const
	{
		return Quaternion(w - q.w, v - q.v);
	}

	Quaternion operator*(const Quaternion& q) const
	{
		return Quaternion(
			w * q.w - v.dot(q.v), 
			q.v * w + v * q.w + v.cross(q.v)
		);
	}

	Quaternion operator*=(const Quaternion& q)
	{
		(*this) = (*this) * q;
	}

	void operator*=(const float value)
	{
		w *= value;
		v *= value;
	}

	Quaternion operator*(const float value) const
	{
		return Quaternion(w * value, v * value);
	}

	// Util Funcs
	float norm() const
	{
		return sqrtf(w * w + v.dot(v));
	}

	void normalize()
	{
		float n = norm();
		if (n != 0)
		{
			float nInverse = 1 / n;
			(*this) *= nInverse;
		}
	}

	void convertToUnitRotQuaternion()
	{
		float angle = MathUtil::degreesToRadians(w);
		v.normalize();
		w = cosf(angle * 0.5);
		v = v * sinf(angle * 0.5);
	}

	Quaternion conjugate() const
	{
		return Quaternion(w, v * (-1));
	}

	Quaternion inverse() const
	{
		float divide = norm();
		divide *= divide;
		divide = 1 / divide;

		return conjugate() * divide;
	}

	// Operation on Vector
	Vector3 rotateVector(const Vector3& v) const
	{
		Quaternion temp(0, v);
		const Quaternion& q = (*this);
		return (q * temp * q.inverse()).v;
	}
};
} // namespace VSC