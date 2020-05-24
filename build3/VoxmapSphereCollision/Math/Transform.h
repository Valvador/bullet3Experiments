#pragma once

#include "Vector3.h"
#include "Quaternion.h"

namespace VSC
{
class Transform
{
private:
	Vector3 position;
	Quaternion rotation;

public:
	// Getters
	const Vector3& getPosition() const { return position; }
	const Quaternion& getRotation() const { return rotation; }

	Transform() :
		position(0.0f),
		rotation(1.00, Vector3(0))
	{};

	Transform(const Vector3& p) :
		position(p),
		rotation(1.00, Vector3(0))
	{};

	Transform(const Vector3& p, const Quaternion& q) :
		position(p),
		rotation(q)
	{};

	Transform inverse() const
	{
		return Transform(position * (-1), rotation.inverse());
	}

	// Transform Multiplications
	Transform operator*(const Transform& t) const
	{
		return Transform(position + t.position, rotation * t.rotation);
	}

	// Point Operations
	Vector3 pointToWorldSpace(const Vector3& localSpace) const
	{
		return (rotation.inverse().rotateVector(localSpace) + position);
	}

	Vector3 pointToLocalSpace(const Vector3& worldSpace) const
	{
		return rotation.rotateVector(worldSpace - position);
	}
};
};