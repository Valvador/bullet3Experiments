#include "Stdafx.h"
#include "CollisionConstraint.h"

using namespace PGSSOlver;
const static float baumgarteStabilizationMaxLength = 0.3f;


CollisionConstraint::CollisionConstraint(RigidBody_c* body0, RigidBody_c* body1, XMFLOAT3& localContactPos0, XMFLOAT3& localContactPos1, XMFLOAT3& normal, float elasticity, float friction, float distance)
{
	bodyA			= body0;
	bodyB			= body1;
	worldPositionA	= body0->m_position;
	worldPositionB	= body1->m_position;
	localPositionA	= localContactPos0;
	localPositionB	= localContactPos1;
	contactNormal	= normal;

	coeffElasticity = elasticity;
	coeffFriction	= friction;
	depth			= distance;
}

CollisionConstraint::~CollisionConstraint()
{

}

int CollisionConstraint::GetDimension() const
{
	return 3;
}


void CollisionConstraint::GetContactJacobian(DMatrix& fullJacobian, XMFLOAT3& contactNormal, XMVECTOR& localContactPos)
{
	assert(fullJacobian.GetNumRows() >= 3);
	assert(fullJacobian.GetNumCols() == 6);
	// Get the linear component of the Jacobi matrix for the body
	// Diagonal matrix of the contactNormal vector components
	//		[  Nx	 0	   0  ]
	//		[  0 	 Ny	   0  ]
	//		[  0     0     Nz ]
	fullJacobian.Set(0, 0) = contactNormal.x;
	fullJacobian.Set(1, 1) = contactNormal.y;
	fullJacobian.Set(2, 2) = contactNormal.z;

	// Angular component, Skew-Symmetric matrix
	// Ca = ath component of the cross product (r x n)
	//		[  0	 Cz	   -Cy ]
	//		[ -Cz	 0		Cx ]
	//		[  Cy   -Cx     0  ]
	XMVECTOR normal = XMLoadFloat3(&contactNormal);
	XMVECTOR crossResults = XMVector3Cross(localContactPos, normal);
	XMFLOAT3 crossResultsV3;
	XMStoreFloat3(&crossResultsV3, crossResults);
	fullJacobian.Set(0, 4) = crossResultsV3.z;
	fullJacobian.Set(0, 5) = -crossResultsV3.y;
	fullJacobian.Set(1, 3) = -crossResultsV3.z;
	fullJacobian.Set(1, 5) = crossResultsV3.x;
	fullJacobian.Set(2, 3) = crossResultsV3.y;
	fullJacobian.Set(2, 4) = -crossResultsV3.x;
}

void CollisionConstraint::GetFrictionJacobian(DMatrix& fullJacobian, RigidBody_c* rb1, RigidBody_c* rb2, XMVECTOR& localContactPos)
{
	assert(fullJacobian.GetNumRows() >= 9);  // Assumes that Collision constraint has at least 3 rows
	assert(fullJacobian.GetNumCols() == 6);

	//Linear Component of Friction Constraint
	//		[  U1x	 0	   0  ]
	//		[  0 	 U1y   0  ]
	//		[  0     0     U1z]
	//		[  U2x	 0	   0  ]
	//		[  0 	 U2y   0  ]
	//		[  0     0     U2z]
	fullJacobian.Set(3 + 0, 0) = rb1->m_linearVelocity.x;
	fullJacobian.Set(3 + 1, 1) = rb1->m_linearVelocity.y;
	fullJacobian.Set(3 + 2, 2) = rb1->m_linearVelocity.z;
	fullJacobian.Set(6 + 0, 0) = rb2->m_linearVelocity.x;
	fullJacobian.Set(6 + 1, 1) = rb2->m_linearVelocity.y;
	fullJacobian.Set(6 + 2, 2) = rb2->m_linearVelocity.z;
		 
	//Angular Component of Friction Constraint
	//V1a = ath component of the cross product (r x u1)
	//		[  0	 V1z   -V1y]
	//		[-V1z	 0		V1x]
	//		[ V1y   -V1x    0  ]
	//		[  0	 V2z   -V2y]
	//		[ -V2z	 0		V2x]
	//		[  V2y  -V2x    0  ]
	XMVECTOR u1 = XMLoadFloat3(&rb1->m_linearVelocity);
	XMVECTOR u2 = XMLoadFloat3(&rb2->m_linearVelocity);
	XMVECTOR rCrossU1 = XMVector3Cross(localContactPos, u1);
	XMVECTOR rCrossU2 = XMVector3Cross(localContactPos, u2);
	XMFLOAT3 V1, V2;
	XMStoreFloat3(&V1, rCrossU1);
	XMStoreFloat3(&V2, rCrossU2);
	fullJacobian.Set(3 + 0, 3 + 1) =  V1.z;
	fullJacobian.Set(3 + 0, 3 + 2) = -V1.y;
	fullJacobian.Set(3 + 1, 3 + 0) = -V1.z;
	fullJacobian.Set(3 + 1, 3 + 2) =  V1.x;
	fullJacobian.Set(3 + 2, 3 + 0) =  V1.y;
	fullJacobian.Set(3 + 2, 3 + 1) = -V1.x;
	fullJacobian.Set(6 + 0, 3 + 1) =  V2.z;
	fullJacobian.Set(6 + 0, 3 + 2) = -V2.y;
	fullJacobian.Set(6 + 1, 3 + 0) = -V2.z;
	fullJacobian.Set(6 + 1, 3 + 2) = -V2.x;
	fullJacobian.Set(6 + 2, 3 + 0) =  V2.y;
	fullJacobian.Set(6 + 2, 3 + 1) = -V2.x;


}

DMatrix CollisionConstraint::GetJacobian(const RigidBody_c* rb)
{
	// We are getting the Jacobian section that has to do with the restrictions on a SINGLE rigid body.
	// C' = [(v1 + w1 x r1) - (v0 + w0 x r0)] dot n1 + [(p1 + r1) - (p0 + r0)] dot (w x n1) >= 0
	//
	// For a single body the Jacobian is (-/+ for bodyA, bodyB respectively) 
	// (COLLISIONS ONLY)
	// C(body)' = (-/+)(v + w x r) dot n1 + (-/+)(p x r) dot (w x n1) 
	// (FRICTION CONSTRAINT)
	// C(body)' = (-/+)(u1   r x u1)
	//            (-/+)(u2   r x u2)

	RigidBody_c* rigidBody = NULL;
	XMVECTOR localContactPos;
	XMVECTOR worldPosition;
	int jacobianMult = 0;
	if (rb == bodyA)
	{
		rigidBody = bodyA;
		localContactPos = XMLoadFloat3(&localPositionA);
		worldPosition = XMLoadFloat3(&bodyA->m_position);
		jacobianMult = -1;
	}
	else if (rb == bodyB)
	{
		rigidBody = bodyB;
		localContactPos = XMLoadFloat3(&localPositionB);
		worldPosition = XMLoadFloat3(&bodyB->m_position);
		jacobianMult = 1;
	}

	if (rigidBody)
	{
		XMVECTOR normal = XMLoadFloat3(&contactNormal);
		DMatrix fullJacob(3, 6);

		GetContactJacobian(fullJacob, contactNormal, localContactPos);
		
		return fullJacob * jacobianMult;
	}
	return DMatrix(0, 0);
}

DMatrix CollisionConstraint::GetConstraintLimits()
{
	// TODO: Implement
	return DMatrix(0, 0);
}

DMatrix CollisionConstraint::GetPenalty()
{
	// TODO: Verify validity of these this "Stabilization Matrix"
	DMatrix positionalCorrection(3, 1);

	if (-1*depth > Config::positionCorrectionDepthThreshold)
	{
		XMVECTOR worldNormalOnB = XMLoadFloat3(&contactNormal);
		XMVECTOR depthSplat = XMLoadFloat3(&XMFLOAT3(depth, depth, depth));
		XMVECTOR depenetrationVector = XMVectorMultiply(worldNormalOnB, depthSplat);
		XMFLOAT3 depenetration;

		XMStoreFloat3(&depenetration, depenetrationVector);
		positionalCorrection.Set(0, 0) = depenetration.x;
		positionalCorrection.Set(1, 0) = depenetration.y;
		positionalCorrection.Set(2, 0) = depenetration.z;
	}

	return positionalCorrection;
}

DMatrix CollisionConstraint::GetRestitution()
{
	DMatrix restitution(3, 3);
	restitution.Set(0, 0) = restitution.Set(1, 1) = restitution.Set(2, 2) = 1.0f + coeffElasticity;
	return restitution;
}