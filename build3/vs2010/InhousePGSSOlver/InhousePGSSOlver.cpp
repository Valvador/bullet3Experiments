// This is the main DLL file.

#include "stdafx.h"
//#include <DirectXMath.h>
#include <assert.h>

#include "InhousePGSSOlver.h"
#include "CollisionConstraint.h"
#include <DirectXMath.h>

using namespace DirectX;
using namespace PGSSOlver;

static const bool UseJVSolverOpt = true;

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

void Solver::GaussSeidelLCP(const DMatrix& J, const DMatrix& W_Jt, const DMatrix& b, DMatrix& V, DMatrix& x, const DMatrix* lo, const DMatrix* hi)
{
	int maxIterations = 20;
	x.SetToZero();
	const int n = x.GetNumRows();

	DMatrix aDiag = J.diagonalProduct(W_Jt);
	float xi_prime;
	float xi;
	float delta_xi;

	DMatrix A = J * W_Jt; // REMOVE DEBUGGING
	while (maxIterations--)
	{
		for (int i = 0; i < n; i++)
		{
			// xi' = xi + 1/Aii (bi - Ai*x)
			// xi' = xi + 1/Aii (bi - J*Vi)
			xi = x.Get(i);
			xi_prime = xi;
			assert(aDiag.Get(i) != 0.0f);
#ifdef _DEBUG
			float Ai_x = A.rowProduct(x, i).Get(0);
			float J_Vi = J.rowProduct(V, i).Get(0);
			assert(Ai_x == J_Vi); // REMOVE DEBUGGING
#endif
			xi_prime += 1 / aDiag.Get(i) * (b.Get(i) - A.rowProduct(x, i).Get(0));// J.rowProduct(V, i).Get(0));

			if (lo && (xi_prime < lo->Get(i)))
			{
				xi_prime = lo->Get(i);
			}
			if (hi && xi_prime > hi->Get(i))
			{
				xi_prime = hi->Get(i);
			}
			// Update X-component with new result
			x.Set(i) = xi_prime;

			// Fix V component with latest X result by using delta between current
			// and previous result for the X component.
			// V' = V + W*Jt * delta_x
			float delta_xi = xi_prime - xi;
			const DMatrix& V_adjust = W_Jt.colProduct(x, i);
			V.SetSubMatrix(0, i, V_adjust);
		}
	}
}

void Solver::GaussSeidelLCP(DMatrix& a, DMatrix& b, DMatrix* x, const DMatrix* lo, const DMatrix* hi)
{
	int maxIterations = 20; // Test Max value
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
					float a_ij = a.Get(i, j);
					if (a_ij)
					{
						sum = sum - (a_ij * x->Get(j)); // This is where we differ from Jacobi, by using 'x', which is our answer.
					}
				}
			}
			// If a.Get(i,i) is zero – you have a bad matrix!
			assert(a.Get(i, i) != 0.0f);
			x->Set(i) = sum / a.Get(i, i);
			// Only do condition to check if we have them
			if (lo && (x->Get(i) < lo->Get(i)))
			{
				x->Set(i) = lo->Get(i);
			}
			if (hi && x->Get(i) > hi->Get(i))
			{
				x->Set(i) = hi->Get(i);
			}
			/*
			if (abs(x->Get(i, 0)) > 2000.0f)
			{
				printf("All Print - Object[%d] Impulse: %4.7f, too small, Min: %4.7f \n", i, x->Get(i), lo->Get(i));
			}
			*/
		}
		// If we have boundary conditions, e.g. >= or <=, then we modify our basic Ax=b,
		// solver to apply constraint conditions
		// Optional - only if bounds
		/*
		if (lo || hi)
			for (int i = 0; i<n; i++)
			{
				if (lo)
				{
					assert(lo->GetNumCols() == 1); // Sanity Checks
					assert(lo->GetNumRows() == n);
					if (x->Get(i) < lo->Get(i))
					{
						if (x->Get(i, 0) < 2000.0f)
						{
							printf("Object[%d] Impulse: %4.7f, too small, Min: %4.7f \n", i, x->Get(i), lo->Get(i));
						}
						x->Set(i) = lo->Get(i);
						if (x->Get(i, 0) < 2000.0f)
						{
							printf("Updated - Object[%d] Impulse: %4.7f, too small, Min: %4.7f \n", i, x->Get(i), lo->Get(i));
						}
					}
				}
				if (hi)
				{
					assert(hi->GetNumCols() == 1); // Sanity Checks
					assert(hi->GetNumRows() == n);
					if (x->Get(i) > hi->Get(i))
					{
						if (x->Get(i, 0) > 2000.0f)
						{
							printf("Object[%d] Impulse: %4.7f, too large, Max: %4.7f \n", i, x->Get(i), hi->Get(i));
						}
						x->Set(i) = hi->Get(i);
						if (x->Get(i, 0) > 2000.0f)
						{
							printf("Updated - Object[%d] Impulse: %4.7f, too small, Min: %4.7f \n", i, x->Get(i), lo->Get(i));
						}
					}
				}
			}
			*/
	}
	// We've solved x!
}

void Solver::ComputeFreeFall(float dt)
{
	const int numBodies = m_rigidBodies.size();
	const int numConstraints = m_constraints.size();
	if (numConstraints != 0 || numBodies == 0) return;

	DMatrix s(numBodies * 7, 1);							// pos & qrot
	DMatrix u(numBodies * 6, 1);							// vel & rotvel
	DMatrix s_next(numBodies * 7, 1);						// pos & qrot after timestep
	DMatrix u_next(numBodies * 6, 1);						// vel & rotvel after timestep
	DMatrix S(numBodies * 7, numBodies * 6);
	DMatrix MInverse(numBodies * 6, numBodies * 6);
	DMatrix Fext(numBodies * 6, 1);
	setUpBodyMatricies(s, u, s_next, u_next, S, MInverse, Fext);

	u_next = u + dt*MInverse*Fext;
	s_next = s + dt*S*u_next;
	applyIntegrationOnRigidBodies(s_next, u_next);
}

void Solver::setUpBodyMatricies(DMatrix& s, DMatrix& u, DMatrix& s_next, DMatrix& u_next, DMatrix& S, DMatrix& MInverse, DMatrix& Fext)
{
	for (int i = 0; i<m_rigidBodies.size(); i++)
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
		Q.Set(0, 0) = -q.x; Q.Set(0, 1) = -q.y; Q.Set(0, 2) = -q.z;
		Q.Set(1, 0) = q.w;	Q.Set(1, 1) = q.z;	Q.Set(1, 2) = -q.y;
		Q.Set(2, 0) = -q.z;	Q.Set(2, 1) = q.w;	Q.Set(2, 2) = q.x;
		Q.Set(3, 0) = q.y;	Q.Set(3, 1) = -q.x;	Q.Set(3, 2) = q.w;
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

		/*if (rb->m_invMass == 0.0f)
		{
			M.Set(0, 0) = M.Set(1, 1) = M.Set(2, 2) = 0.01f;
			I.Set(0, 0) = 0.01f;   I.Set(1, 0) = dxm._12; I.Set(2, 0) = dxm._13;
			I.Set(0, 1) = dxm._21; I.Set(1, 1) = 0.01f  ; I.Set(2, 1) = dxm._23;
			I.Set(0, 2) = dxm._31; I.Set(1, 2) = dxm._32; I.Set(2, 2) = 0.01f;
		}*/
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
}

void Solver::applyIntegrationOnRigidBodies(DMatrix& s_next, DMatrix& u_next)
{
	for (int i = 0; i< m_rigidBodies.size(); i++)
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
		
		XMVECTOR normalQuaternion = XMQuaternionNormalize(XMLoadFloat4(&rb->m_orientation));
		XMStoreFloat4(&rb->m_orientation, normalQuaternion);
	}
}

void Solver::ComputeJointConstraints(float dt)
{
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
	DMatrix u(numBodies * 6, 1);							// vel & rotvel
	DMatrix s_next(numBodies * 7, 1);						// pos & qrot after timestep
	DMatrix u_next(numBodies * 6, 1);						// vel & rotvel after timestep
	DMatrix S(numBodies * 7, numBodies * 6);
	DMatrix& MInverse = m_MInverseBuffer;
	MInverse.Resize(numBodies * 6, numBodies * 6, true);	// ToDo - This may be mostly static!
	DMatrix Fext(numBodies * 6, 1);
	setUpBodyMatricies(s, u, s_next, u_next, S, MInverse, Fext);
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
	DMatrix minForces(numRows, 1);
	DMatrix maxForces(numRows, 1);
	DMatrix& J = m_jacobianBuffer;
	J.Resize(numRows, 6 * numBodies, /*memReset*/true);
	DMatrix rst(numRows, numRows);			// Restitution
	DMatrix& s_err = m_cBuffer;
	s_err.Resize(numRows, 1, true);
	int constraintRow = 0;
	for (int c = 0; c<numConstraints; c++)
	{
		Constraint_c* constraint = m_constraints[c];
		assert(constraint);
		DMatrix minLimit(constraint->GetDimension(), 1);
		DMatrix maxLimit(constraint->GetDimension(), 1);

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
			minLimit.AddSubMatrix(0, 0, constraint->GetLowerLimits(rigidBody));
			maxLimit.AddSubMatrix(0, 0, constraint->GetUpperLimits(rigidBody));

			// Delta position needed to depenetrate bodies 
			// (Unfortunately doesn't use constraints yet) (MAY NEED FIX)
		}
		minForces.AddSubMatrix(constraintRow, 0, minLimit);
		maxForces.AddSubMatrix(constraintRow, 0, maxLimit);
		rst.AddSubMatrix(constraintRow, constraintRow, constraint->GetRestitution());		// elasticity
		s_err.AddSubMatrix(constraintRow, 0, constraint->GetPenalty());						// positional correction

		constraintRow += constraint->GetDimension();
	}

	if (UseJVSolverOpt)
	{
		DMatrix& W_Jt = m_MInverseJtBuffer;
		DMatrix& V = m_virtualDisplacementBuffer;
		DMatrix& b = m_bBuffer;
		DMatrix& x = m_resultImpulseBuffer;
		W_Jt = MInverse * DMatrix::Transpose(J);
		b = rst*J*(u)+J*(dt*MInverse*Fext);
		// TODO - To implement Caching/Warmstarting we have to pre-set x and V with previous Data!
		x.Resize(J.GetNumRows(), b.GetNumCols(), /*memReset*/ true);
		V.Resize(W_Jt.GetNumRows(), x.GetNumRows(), /*memReset*/ true);

		// Solve for x [Velocity Stage]
		GaussSeidelLCP(J, W_Jt, b, V, x, &minForces, &maxForces);

		DMatrix& y = m_resultPositionCorrectionBuffer;
		DMatrix& Z = m_virtualCorrectionBuffer;
		y.Resize(J.GetNumRows(), b.GetNumCols(), /*memReset*/ true);
		Z.Resize(W_Jt.GetNumRows(), y.GetNumRows(), /*memReset*/ true);

		// Solve for y [Position Stage]
		GaussSeidelLCP(J, W_Jt, s_err, Z, y, nullptr, nullptr);

		u_next = u - W_Jt*x + dt*MInverse*Fext;
		s_next = s + dt*S*u_next - S*W_Jt*y * Config::positionalCorrectionFactor;
	}
	else
	{
		// Velocity Gauss Seidel
		DMatrix Jt = DMatrix::Transpose(J);
		DMatrix A = J*MInverse*Jt;
		DMatrix b = rst*J*(u)+J*(dt*MInverse*Fext);          
		DMatrix x(A.GetNumRows(), b.GetNumCols());

		DMatrix* lo = NULL; // Don’t set any min/max boundaries for this demo/sample
		DMatrix* hi = NULL;

		// Solve for x
		//GaussSeidelLCP(A, b, &x, lo, hi);
		GaussSeidelLCP(A, b, &x, &minForces, &maxForces);

		// Positional Correction Gauss Seidel
		DMatrix c = s_err;
		DMatrix y(A.GetNumRows(), b.GetNumCols());
		GaussSeidelLCP(A, c, &y, lo, hi);

		u_next = u - MInverse*Jt*x + dt*MInverse*Fext;
		s_next = s + dt*S*u_next - S*MInverse*Jt*y * Config::positionalCorrectionFactor;
	}

	// Basic integration without - euler integration standalone
	// u_next = u + dt*MInverse*Fext;
	// s_next = s + dt*S*u_next;
	//-------------------------------------------------------------------------
	// 3rd – re-inject solved values back into the simulator
	//-------------------------------------------------------------------------
	applyIntegrationOnRigidBodies(s_next, u_next);
}

void Solver::Update(float dt)
{
	ComputeFreeFall(dt); 

	ComputeJointConstraints(dt);
}
