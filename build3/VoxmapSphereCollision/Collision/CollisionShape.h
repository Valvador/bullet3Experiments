#pragma once
#include "Math/Vector3.h"
#include "Math/Transform.h"

namespace VSC
{
struct AxisAlignedBoundingBox
{
	Vector3 min;
	Vector3 max;

	AxisAlignedBoundingBox(const Vector3& _min, const Vector3& _max) : min(_min), max(_max) {};
	AxisAlignedBoundingBox(){};
};

enum CollisionShapeType
{
	DistanceGridSphereTree,
	None
};

class CollisionShape
{
private:
	Transform transform;
	AxisAlignedBoundingBox localBox;

public:
	const Transform& getTransform() { return transform; }
	void setTransform(const Transform& tForm) { transform = tForm; }

	const AxisAlignedBoundingBox& getLocalBoundingBox() { return localBox; }
	void setLocalBoundingBox(const AxisAlignedBoundingBox& box) { localBox = box; }

	virtual CollisionShapeType getShapeType() const = 0;
};
} // namespace VSC