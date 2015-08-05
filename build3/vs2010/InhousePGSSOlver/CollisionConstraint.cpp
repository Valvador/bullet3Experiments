#include "Stdafx.h"
#include "CollisionConstraint.h"

using namespace PGSSOlver;
const static float baumgarteStabilizationMaxLength = 0.3f;


CollisionConstraint::CollisionConstraint(RigidBody_c* body0, RigidBody_c* body1, XMFLOAT3& localContactPos0, XMFLOAT3& localContactPos1, XMFLOAT3& normal)
{
	bodyA			= body0;
	bodyB			= body1;
	worldPositionA	= body0->m_position;
	worldPositionB	= body1->m_position;
	localPositionA	= localContactPos0;
	localPositionB	= localContactPos1;
	contactNormal	= normal;
}

CollisionConstraint::~CollisionConstraint()
{

}

int CollisionConstraint::GetDimension() const
{
	return 3;
}

DMatrix CollisionConstraint::GetJacobian(const RigidBody_c* rb)
{
	// We are getting the Jacobian section that has to do with the restrictions on a SINGLE rigid body.
	// C' = [(v1 + w1 x r1) - (v0 + w0 x r0)] dot n1 + [(p1 + r1) - (p0 + r0)] dot (w x n1) >= 0
	//
	// For a single body the Jacobian is (-/+ for bodyA, bodyB respectively)
	// C(body)' = (-/+)(v + w x r) dot n1 + (-/+)(p x r) dot (w x n1) 
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

		// Get the linear component of the Jacobi matrix for the body
		// Diagonal matrix of the contactNormal vector components
		//		[  Nx	 0	   0  ]
		//		[  0 	 Ny	   0  ]
		//		[  0     0     Nz ]
		DMatrix linearJacob(3, 3);
		linearJacob.Set(0, 0) = contactNormal.x;
		linearJacob.Set(1, 1) = contactNormal.y;
		linearJacob.Set(2, 2) = contactNormal.z;
		
		// Angular component, Skew-Symmetric matrix
		// Ca = ath component of the cross product (r x n)
		//		[  0	 Cz	   -Cy ]
		//		[ -Cz	 0		Cx ]
		//		[  Cy   -Cx     0  ]
		DMatrix angularJacob(3, 3);
		XMVECTOR crossResults = XMVector3Cross(localContactPos, normal);
		XMFLOAT3 crossResultsV3;
		XMStoreFloat3(&crossResultsV3, crossResults);
		angularJacob.Set(0, 1) = crossResultsV3.z;
		angularJacob.Set(0, 2) = -crossResultsV3.y;
		angularJacob.Set(1, 0) = -crossResultsV3.z;
		angularJacob.Set(1, 2) = crossResultsV3.x;
		angularJacob.Set(2, 0) = crossResultsV3.y;
		angularJacob.Set(2, 1) = -crossResultsV3.x;

		// Combine the linear and angular components together
		fullJacob.SetSubMatrix(0, 0, linearJacob );
		fullJacob.SetSubMatrix(0, 3, angularJacob);
		return fullJacob * jacobianMult;
	}
	return DMatrix(0, 0);
}

DMatrix CollisionConstraint::GetPenalty()
{
	// TODO: Verify validity of these this "Stabilization Matrix"
	DMatrix stabilizationError(3, 1);
	XMVECTOR worldPosA = XMLoadFloat3(&worldPositionA);
	XMVECTOR worldPosB = XMLoadFloat3(&worldPositionB);
	XMVECTOR maxDeltaP = XMVector3ClampLength(worldPosA - worldPosB, 0.0f, baumgarteStabilizationMaxLength);
	XMFLOAT3 errorMatrix;

	XMStoreFloat3(&errorMatrix, maxDeltaP);
	stabilizationError.Set(0, 0) = errorMatrix.x;
	stabilizationError.Set(1, 0) = errorMatrix.y;
	stabilizationError.Set(2, 0) = errorMatrix.z;

	return stabilizationError;
}