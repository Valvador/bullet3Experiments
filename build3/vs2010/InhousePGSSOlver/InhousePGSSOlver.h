// InhousePGSSOlver.h
#include <vector>

#include "DMatrix.h"
#include "RigidBody_c.h"
#include "Constraint_c.h"

#pragma once

namespace PGSSOlver {

	public class Solver
	{
		// TODO: Add your methods for this class here.

	public:
		Solver();
		~Solver();

		static void GaussSeidelLCP(DMatrix& a, DMatrix& b, DMatrix* x, const DMatrix* lo, const DMatrix* hi);
		void Update(float dt = 0.0166666f);
		void ComputeForces(float dt = 0.0166666f);
		void ComputeJointConstraints(float dt = 0.0166666f);
	private:
		std::vector<RigidBody_c*>		m_rigidBodies;
		std::vector<Constraint_c*>		m_constraints;
	};

}
