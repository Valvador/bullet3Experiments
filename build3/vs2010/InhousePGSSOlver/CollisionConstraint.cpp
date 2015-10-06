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

	XMVECTOR normalVec = XMLoadFloat3(&contactNormal);
	XMFLOAT3 basisX = XMFLOAT3(1, 0, 0);
	XMFLOAT3 basisY = XMFLOAT3(0, 1, 0);
	XMVECTOR basisXVec = XMLoadFloat3(&basisX);
	XMVECTOR basisYVec = XMLoadFloat3(&basisY);
	XMVECTOR u1;
	if (std::abs(XMVector3Dot(normalVec, basisXVec).m128_f32[0]) < 0.9f)
	{
		u1 = XMVector3Cross(normalVec, basisXVec);
	}
	else
	{
		u1 = XMVector3Cross(normalVec, basisYVec);
	}
	XMVECTOR u2 = XMVector3Cross(normalVec, u1);
	XMStoreFloat3(&normPlaneU1, u1);
	XMStoreFloat3(&normPlaneU2, u2);
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
	assert(fullJacobian.GetNumRows() >= 1);
	assert(fullJacobian.GetNumCols() == 6);
	// Contact Jacobian
	// [ n, r x n ]

	fullJacobian.Set(0, 0) = contactNormal.x;
	fullJacobian.Set(0, 1) = contactNormal.y;
	fullJacobian.Set(0, 2) = contactNormal.z;


	XMVECTOR normal = XMLoadFloat3(&contactNormal);
	XMVECTOR crossResults = XMVector3Cross(localContactPos, normal);
	XMFLOAT3 crossResultsV3;
	XMStoreFloat3(&crossResultsV3, crossResults);

	fullJacobian.Set(0, 3) = crossResultsV3.x;
	fullJacobian.Set(0, 4) = crossResultsV3.y;
	fullJacobian.Set(0, 5) = crossResultsV3.z;
}

void CollisionConstraint::GetFrictionJacobian(DMatrix& fullJacobian, XMVECTOR& localContactPos)
{
	assert(fullJacobian.GetNumRows() >= 3);  // Assumes that Collision constraint has at least 3 rows
	assert(fullJacobian.GetNumCols() == 6);

	
	XMVECTOR u1 = XMLoadFloat3(&normPlaneU1);
	XMVECTOR u2 = XMLoadFloat3(&normPlaneU2);
	XMVECTOR rCrossU1 = XMVector3Cross(localContactPos, u1);
	XMVECTOR rCrossU2 = XMVector3Cross(localContactPos, u2);
	XMFLOAT3 u1F3;
	XMFLOAT3 u2F3;
	XMStoreFloat3(&u1F3, u1);
	XMStoreFloat3(&u2F3, u2);

	// Friction Constraint
	// C1 = [ u1 , r x u1 ]
	// C2 = [ u2 , r x u2 ]
	fullJacobian.Set(1, 0) = u1F3.x;
	fullJacobian.Set(1, 1) = u1F3.y;
	fullJacobian.Set(1, 2) = u1F3.z;
	fullJacobian.Set(2, 0) = u2F3.x;
	fullJacobian.Set(2, 1) = u2F3.y;
	fullJacobian.Set(2, 2) = u2F3.z;		

	XMFLOAT3 V1, V2;
	XMStoreFloat3(&V1, rCrossU1);
	XMStoreFloat3(&V2, rCrossU2);

	fullJacobian.Set(1, 3) = V1.x;
	fullJacobian.Set(1, 4) = V1.y;
	fullJacobian.Set(1, 5) = V1.z;
	fullJacobian.Set(2, 3) = V2.x;
	fullJacobian.Set(2, 4) = V2.y;
	fullJacobian.Set(2, 5) = V2.z;

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
		DMatrix fullJacob(GetDimension(), 6);
		XMFLOAT3 localContact;
		XMStoreFloat3(&localContact, localContactPos);
		GetContactJacobian(fullJacob, contactNormal, localContactPos);
		GetFrictionJacobian(fullJacob, localContactPos);
		
		return fullJacob * jacobianMult;
	}
	return DMatrix(0, 0);
}

DMatrix CollisionConstraint::GetLowerLimits(const RigidBody_c* rb)
{
	DMatrix lowerLimit(GetDimension(), 1);
	lowerLimit.Set(0, 0) = 0;// COLLISIONS CANNOT PULL OBJECTS
	
	if (rb->m_invMass != 0.0f)
	{
		// -mU*m*Fex (TODO: CURRENTLY GRAVITY ONLY -> NEED TO ABSTRACT TO ALL FORCES)
		lowerLimit.Set(1, 0) = -abs((coeffFriction / rb->m_invMass) * rb->m_force.y);
		lowerLimit.Set(2, 0) = -abs((coeffFriction / rb->m_invMass) * rb->m_force.y);
	}

	return lowerLimit;
}

DMatrix CollisionConstraint::GetUpperLimits(const RigidBody_c* rb)
{
	DMatrix upperLimit(GetDimension(), 1);
	upperLimit.Set(0, 0) = FLT_MAX;

	if (rb->m_invMass != 0.0f)
	{
		// mU*m*Fex (TODO: CURRENTLY GRAVITY ONLY -> NEED TO ABSTRACT TO ALL FORCES)
		upperLimit.Set(1, 0) = abs((coeffFriction / rb->m_invMass) * rb->m_force.y);
		upperLimit.Set(2, 0) = abs((coeffFriction / rb->m_invMass) * rb->m_force.y);
	}

	return upperLimit;
}

DMatrix CollisionConstraint::GetPenalty()
{
	// TODO: Verify validity of these this "Stabilization Matrix"
	DMatrix positionalCorrection(GetDimension(), 1);

	if (-1*depth > Config::positionCorrectionDepthThreshold)
	{
		positionalCorrection.Set(0, 0) = -depth;
	}

	return positionalCorrection;
}

DMatrix CollisionConstraint::GetRestitution()
{
	DMatrix restitution(1, 1);
	restitution.Set(0, 0) = 1.0f + coeffElasticity;
	return restitution;
}