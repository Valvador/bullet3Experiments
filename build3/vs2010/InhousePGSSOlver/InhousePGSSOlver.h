// InhousePGSSOlver.h
#include <vector>
#include <assert.h>

#ifndef INHOUSE_PGS_H
#define INHOUSE_PGS_H

#include "../../../src/LinearMath/btAlignedObjectArray.h"

#include "DMatrix.h"
#include "RigidBody_c.h"
#include "Constraint_c.h"
#include "SolverConfig.h"

#pragma once

namespace PGSSOlver {

	class Solver
	{
	public:
		Solver();
		~Solver();

		static void GaussSeidelLCP(DMatrix& a, DMatrix& b, DMatrix* x, const DMatrix* lo, const DMatrix* hi);
		static void GaussSeidelLCP(	const DMatrix& J,	// Jacobian - Constraint Info
			                        const DMatrix& W_Jt,// Mass Inverse * J Transpose
									const DMatrix& b,	// b side of the A * x = b equation
									DMatrix& V,			// M_inverse * Jt * x, J*V replace the A * x side of the equation.
									DMatrix& x, 
									const DMatrix* lo,	// low clamp of x
									const DMatrix* hi);	// high clamp of x

		void Update(float dt = 0.0166666f);
		void ComputeForces(float dt = 0.0166666f);
		void ComputeJointConstraints(float dt = 0.0166666f);
		void ComputeFreeFall(float dt = 0.0166666f);

		void setRigidBodies(btAlignedObjectArray<RigidBody_c*>& bodies)			{ assert(m_rigidBodies.size() == 0); m_rigidBodies = bodies; }
		void setConstraints(std::vector<Constraint_c*>& constraints)			{ assert(m_constraints.size() == 0); m_constraints = constraints; }

		void addRigidBody(RigidBody_c* body)					{ m_rigidBodies.push_back(body); }
		void addConstraint(Constraint_c* constraint)			{ m_constraints.push_back(constraint); }

		void clearRigidBodies();
		void clearConstraints();
	protected:
		btAlignedObjectArray<RigidBody_c*>		m_rigidBodies;
		std::vector<Constraint_c*>				m_constraints;

		// Computational Buffers
		DMatrix									m_jacobianBuffer;  //J 
		DMatrix									m_MInverseBuffer; //W or M_inverse
		DMatrix									m_MInverseJtBuffer; // W * Jt
		// Velocity Stage
		DMatrix	                                m_resultImpulseBuffer; //x
		DMatrix									m_bBuffer;
		DMatrix									m_virtualDisplacementBuffer; // M_inverse * Jt * x

		// Position Stage
		DMatrix									m_resultPositionCorrectionBuffer; //y
		DMatrix									m_cBuffer;
		DMatrix									m_virtualCorrectionBuffer; // M_inverse * Jt * y

		// UTIL
		void setUpBodyMatricies(DMatrix& s, DMatrix& u, DMatrix& s_next, DMatrix& u_next, DMatrix& S, DMatrix& MInverse, DMatrix& Fext);
		void applyIntegrationOnRigidBodies(DMatrix& s_next, DMatrix& u_next);
	};

}

#endif