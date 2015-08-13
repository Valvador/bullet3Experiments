// This is the main DLL file.

#include "stdafx.h"
//#include <DirectXMath.h>
#include <assert.h>

#include "InhousePGSSOlver.h"
#include "CollisionConstraint.h"
#include <DirectXMath.h>

using namespace DirectX;
using namespace PGSSOlver;


Solver::Solver()
{
	assert(m_constraints.size() == 0);
	assert(m_rigidBodies.size() == 0);
}

Solver::~Solver()
{
	m_constraints.clear();
	m_rigidBodies.clear();
}

void Solver::clearConstraints()
{
	for (int i = 0; i < m_constraints.size(); i++)
	{
		delete m_constraints[i];
	}

	m_constraints.clear();
}

void Solver::clearRigidBodies()
{
	for (int i = 0; i < m_rigidBodies.size(); i++)
	{
		delete m_rigidBodies[i];
	}

	m_rigidBodies.clear();
}

void Solver::GaussSeidelLCP(DMatrix& a, DMatrix& b, DMatrix* x, const DMatrix* lo, const DMatrix* hi)
{
	int maxIterations = 10; // Test Max value
	x->SetToZero(); // Clear our matrix to start with (slow, only for debug)
	const int n = x->GetNumRows();

	float sum = 0.0f;
	while (maxIterations--)
	{
		for (int i = 0; i < n; i++)
		{
			sum = b.Get(i);
			for (int j = 0; j < n; j++)
			{
				if (i != j)
				{
					sum = sum - (a.Get(i, j) * x->Get(j));
				}
			}
			// If a.Get(i,i) is zero – you have a bad matrix!
			assert(a.Get(i, i) != 0.0f);
			x->Set(i) = sum / a.Get(i, i);
			// Only do condition to check if we have them
		}
		// If we have boundary conditions, e.g. >= or <=, then we modify our basic Ax=b,
		// solver to apply constraint conditions
		// Optional - only if bounds
		if (lo || hi)
			for (int i = 0; i<n; i++)
			{
				if (lo)
				{
					assert(lo->GetNumCols() == 1); // Sanity Checks
					assert(lo->GetNumRows() == n);
					if (x->Get(i) < lo->Get(i)) x->Set(i) = lo->Get(i);
				}
				if (hi)
				{
					assert(hi->GetNumCols() == 1); // Sanity Checks
					assert(hi->GetNumRows() == n);
					if (x->Get(i) > hi->Get(i)) x->Set(i) = hi->Get(i);
				}
			}
	}
	// We've solved x!
}void Solver::ComputeJointConstraints(float dt){
	// Magic Formula
	//
	// J * M^-1 * J^t * lamba = -1.0 * J * (1/dt*V + M^-1 * Fext)
	//
	// A x = b
	//
	// where
	//
	// A = J * M^-1 * J^t
	// x = lambda
	// b = -J * (1/dt*V + M^-1 * Fext)
	//
	const int numBodies			= m_rigidBodies.size();
	const int numConstraints	= m_constraints.size();
	if (numBodies == 0 || numConstraints == 0) return;
	//-------------------------------------------------------------------------
	// 1st - build our matrices - very bad to build them each frame, but
	//-------------------------------------------------------------------------
	// simpler to explain and implement this way
	DMatrix s(numBodies * 7, 1);							// pos & qrot
	DMatrix s_corr(numBodies * 7, 1);
	DMatrix u(numBodies * 6, 1);							// vel & rotvel
	DMatrix s_next(numBodies * 7, 1);						// pos & qrot after timestep
	DMatrix u_next(numBodies * 6, 1);						// vel & rotvel after timestep
	DMatrix S(numBodies * 7, numBodies * 6);
	DMatrix MInverse(numBodies * 6, numBodies * 6);
	DMatrix Fext(numBodies * 6, 1);
	for (int i = 0; i<numBodies; i++)
	{
		const RigidBody_c* rb = m_rigidBodies[i];
		s.Set(i * 7 + 0) = rb->m_position.x;
		s.Set(i * 7 + 1) = rb->m_position.y;
		s.Set(i * 7 + 2) = rb->m_position.z;
		s.Set(i * 7 + 3) = rb->m_orientation.w;
		s.Set(i * 7 + 4) = rb->m_orientation.x;
		s.Set(i * 7 + 5) = rb->m_orientation.y;
		s.Set(i * 7 + 6) = rb->m_orientation.z;
		u.Set(i * 6 + 0) = rb->m_linearVelocity.x;
		u.Set(i * 6 + 1) = rb->m_linearVelocity.y;
		u.Set(i * 6 + 2) = rb->m_linearVelocity.z;
		u.Set(i * 6 + 3) = rb->m_angularVelocity.x;
		u.Set(i * 6 + 4) = rb->m_angularVelocity.y;
		u.Set(i * 6 + 5) = rb->m_angularVelocity.z;
		const XMFLOAT4& q = rb->m_orientation;
		DMatrix Q(4, 3);
		Q.Set(0, 0) = -q.x; 
		Q.Set(0, 1) = -q.y; 
		Q.Set(0, 2) = -q.z;
		Q.Set(1, 0) = q.w; 
		Q.Set(1, 1) = q.z; 
		Q.Set(1, 2) = -q.y;
		Q.Set(2, 0) = -q.z; 
		Q.Set(2, 1) = q.w; 
		Q.Set(2, 2) = q.x;
		Q.Set(3, 0) = q.y; 
		Q.Set(3, 1) = -q.x; 
		Q.Set(3, 2) = q.w;
		Q = Q *  0.5f;
		DMatrix Idenity(3, 3);
		Idenity.SetToZero();
		Idenity.Set(0, 0) = Idenity.Set(1, 1) = Idenity.Set(2, 2) = 1.0f;
		S.SetSubMatrix(i * 7 + 0, i * 6 + 0, Idenity);
		S.SetSubMatrix(i * 7 + 3, i * 6 + 3, Q);
		DMatrix M(3, 3);
		M.Set(0, 0) = M.Set(1, 1) = M.Set(2, 2) = rb->m_invMass;
		const XMFLOAT3X3& dxm = rb->CreateWorldII();
		DMatrix I(3, 3);
		I.Set(0, 0) = dxm._11; I.Set(1, 0) = dxm._12; I.Set(2, 0) = dxm._13;
		I.Set(0, 1) = dxm._21; I.Set(1, 1) = dxm._22; I.Set(2, 1) = dxm._23;
		I.Set(0, 2) = dxm._31; I.Set(1, 2) = dxm._32; I.Set(2, 2) = dxm._33;
		MInverse.SetSubMatrix(i * 6, i * 6, M);
		MInverse.SetSubMatrix(i * 6 + 3, i * 6 + 3, I);
		DMatrix F(3, 1);
		F.Set(0, 0) = rb->m_force.x;
		F.Set(1, 0) = rb->m_force.y;
		F.Set(2, 0) = rb->m_force.z;
		XMFLOAT3 rF = rb->m_torque;
		DMatrix T(3, 1);
		T.Set(0, 0) = rF.x;
		T.Set(1, 0) = rF.y;
		T.Set(2, 0) = rF.z;
		Fext.SetSubMatrix(i * 6, 0, F);
		Fext.SetSubMatrix(i * 6 + 3, 0, T);
	}
	//-------------------------------------------------------------------------
	// 2nd - apply constraints
	//-------------------------------------------------------------------------
	// Determine the size of our jacobian matrix
	int numRows = 0;
	for (int i = 0; i<numConstraints; i++)
	{
		const Constraint_c* constraint = m_constraints[i];
		assert(constraint);
		numRows += constraint->GetDimension();
	}
	// Allocate it, and fill it
	DMatrix J(numRows, 6 * numBodies);
	DMatrix rst(numRows, numRows);			// Restitution
	int constraintRow = 0;
	for (int c = 0; c<numConstraints; c++)
	{
		Constraint_c* constraint = m_constraints[c];
		assert(constraint);
		for (int r = 0; r<numBodies; r++)
		{
			const RigidBody_c* rigidBody = m_rigidBodies[r];
			assert(rigidBody);
			DMatrix JMat = constraint->GetJacobian(rigidBody);
			if (JMat.GetNumCols() == 0 && JMat.GetNumRows() == 0)
				continue;
			assert(JMat.GetNumCols() != 0);
			assert(JMat.GetNumRows() != 0);
			J.SetSubMatrix(constraintRow, r * 6, JMat);

			// Delta position needed to depenetrate bodies 
			// (Unfortunately doesn't use constraints yet) (MAY NEED FIX)
			DMatrix adjMat = constraint->GetPositionalCorrection(rigidBody);
			s_corr.AddSubMatrix(r * 7, 0, adjMat);
		}
		rst.AddSubMatrix(constraintRow, constraintRow, constraint->GetRestitution());
		constraintRow += constraint->GetDimension();
	}
	DMatrix Jt = DMatrix::Transpose(J);
	DMatrix A = J*MInverse*Jt;
	DMatrix b = rst * J*(u)+J*(dt*MInverse*Fext);          // HIGHLY UNOPTIMIZED! NEEDS WORK!
	DMatrix x(A.GetNumRows(), b.GetNumCols());
	DMatrix* lo = NULL; // Don’t set any min/max boundaries for this demo/sample
	DMatrix* hi = NULL;
	// Solve for x
	GaussSeidelLCP(A, b, &x, lo, hi);

	//dprintf( A.Print() );
	u_next = u - MInverse*Jt*x + dt*MInverse*Fext;
	s_next = s + dt*S*u_next   - s_corr * Config::positionalCorrectionFactor;
	// Basic integration without - euler integration standalone
	// u_next = u + dt*MInverse*Fext;
	// s_next = s + dt*S*u_next;
	//-------------------------------------------------------------------------
	// 3rd – re-inject solved values back into the simulator
	//-------------------------------------------------------------------------
	for (int i = 0; i<numBodies; i++)
	{
		RigidBody_c* rb = m_rigidBodies[i];
		rb->m_position.x = s_next.Get(i * 7 + 0);
		rb->m_position.y = s_next.Get(i * 7 + 1);
		rb->m_position.z = s_next.Get(i * 7 + 2);
		rb->m_orientation.w = s_next.Get(i * 7 + 3);
		rb->m_orientation.x = s_next.Get(i * 7 + 4);
		rb->m_orientation.y = s_next.Get(i * 7 + 5);
		rb->m_orientation.z = s_next.Get(i * 7 + 6);
		rb->m_linearVelocity.x = u_next.Get(i * 6 + 0);
		rb->m_linearVelocity.y = u_next.Get(i * 6 + 1);
		rb->m_linearVelocity.z = u_next.Get(i * 6 + 2);
		rb->m_angularVelocity.x = u_next.Get(i * 6 + 3);
		rb->m_angularVelocity.y = u_next.Get(i * 6 + 4);
		rb->m_angularVelocity.z = u_next.Get(i * 6 + 5);
		rb->m_force = XMFLOAT3(0, 0, 0);
		rb->m_torque = XMFLOAT3(0, 0, 0);
		// Just incase we get drifting in our quaternion orientation
		XMVECTOR normalQuaternion = XMQuaternionNormalize(XMLoadFloat4(&rb->m_orientation));
		XMStoreFloat4(&rb->m_orientation, normalQuaternion);
	}
}void Solver::Update(float dt){	ComputeJointConstraints(dt);}