#include "Constraint_c.h"
#include "RigidBody_c.h"

#ifndef COLLISIONCONSTRAINT_H
#define COLLISIONCONSTRAINT_H


public class CollisionConstraint : public Constraint_c
{
	XMFLOAT3 worldPositionA;
	XMFLOAT3 worldPositionB;
	XMFLOAT3 localPositionA;
	XMFLOAT3 localPositionB;

	CollisionConstraint(RigidBody_c* body0, RigidBody_c* body1);
	~CollisionConstraint();
};

#endif