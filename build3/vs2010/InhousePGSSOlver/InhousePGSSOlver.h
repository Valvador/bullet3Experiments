// InhousePGSSOlver.h
#include <vector>

#include "DMatrix.h"
#include "RigidBody_c.h"
#include "Constraint_c.h"

#pragma once

using namespace System;

namespace PGSSOlver {

	public class Solver
	{
		// TODO: Add your methods for this class here.

	public:
		Solver();
		~Solver();

		static void GaussSeidelLCP(DMatrix& a, DMatrix& b, DMatrix* x, const DMatrix* lo, const DMatrix* hi);
		void ComputeJointConstraints(float dt = 0.0166666f);
	private:
		std::vector<RigidBody_c*>		m_rigidBodies;
		std::vector<Constraint_c*>		m_constraints;
	};

}
