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

		XMFLOAT3 normPlaneU1;
		XMFLOAT3 normPlaneU2;

		RigidBody_c* bodyA;
		RigidBody_c* bodyB;

		float coeffElasticity;
		float coeffFriction;
		float depth;

		void GetContactJacobian(DMatrix& fullJacobian, XMFLOAT3& contactNormal, XMVECTOR& localContactPos);
		void GetFrictionJacobian(DMatrix& fullJacobian, XMVECTOR& localContactPos);
	public:
		CollisionConstraint(RigidBody_c* body0, RigidBody_c* body1, XMFLOAT3& localContactPos0, XMFLOAT3& localContactPos1, XMFLOAT3& normal, float elasticity = 0.0f, float friction = 0.0f, float distance = 0.0f);
		~CollisionConstraint();

		DMatrix GetJacobian(const RigidBody_c* rb)							override;
		DMatrix GetLowerLimits(const RigidBody_c* rb)						override;
		DMatrix GetUpperLimits(const RigidBody_c* rb)						override;
		DMatrix GetPenalty()												override;
		DMatrix GetRestitution()											override;
		int     GetDimension()										  const override;
	};

}
#endif