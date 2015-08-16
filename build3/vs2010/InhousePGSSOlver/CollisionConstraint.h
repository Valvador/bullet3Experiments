#ifndef COLLISIONCONSTRAINT_H
#define COLLISIONCONSTRAINT_H

#include "Constraint_c.h"
#include "DMatrix.h"
#include "RigidBody_c.h"


namespace PGSSOlver {

	class CollisionConstraint : public Constraint_c
	{
		XMFLOAT3 worldPositionA;
		XMFLOAT3 worldPositionB;
		XMFLOAT3 localPositionA;
		XMFLOAT3 localPositionB;
		XMFLOAT3 contactNormal;

		RigidBody_c* bodyA;
		RigidBody_c* bodyB;

		float coeffElasticity;
		float coeffFriction;
		float depth;
	public:
		CollisionConstraint(RigidBody_c* body0, RigidBody_c* body1, XMFLOAT3& localContactPos0, XMFLOAT3& localContactPos1, XMFLOAT3& normal, float elasticity = 0.0f, float friction = 0.0f, float distance = 0.0f);
		~CollisionConstraint();

		DMatrix GetJacobian(const RigidBody_c* rb)							override;
		DMatrix GetPenalty()												override;
		DMatrix GetRestitution()											override;
		int     GetDimension()										  const override;
	};

}
#endif