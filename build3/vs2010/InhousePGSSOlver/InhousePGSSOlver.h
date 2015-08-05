// InhousePGSSOlver.h
#include <vector>
#include <assert.h>

#ifndef INHOUSE_PGS_H
#define INHOUSE_PGS_H

#include "DMatrix.h"
#include "RigidBody_c.h"
#include "Constraint_c.h"

#pragma once

namespace PGSSOlver {

	class Solver
	{
		// TODO: Add your methods for this class here.

	public:
		Solver();
		~Solver();

		static void GaussSeidelLCP(DMatrix& a, DMatrix& b, DMatrix* x, const DMatrix* lo, const DMatrix* hi);
		void Update(float dt = 0.0166666f);
		void ComputeForces(float dt = 0.0166666f);
		void ComputeJointConstraints(float dt = 0.0166666f);

		void setRigidBodies(std::vector<RigidBody_c*>& bodies)			{ assert(m_rigidBodies.size() == 0); m_rigidBodies = bodies; }
		void setConstraints(std::vector<Constraint_c*>& constraints)	{ assert(m_constraints.size() == 0); m_constraints = constraints; }

		void addRigidBody(RigidBody_c* body)			{ m_rigidBodies.push_back(body); }
		void addConstraint(Constraint_c* constraint)	{ m_constraints.push_back(constraint); }

		void clearRigidBodies() { m_rigidBodies.clear(); }
		void clearConstraints() { m_constraints.clear(); }
	private:
		std::vector<RigidBody_c*>		m_rigidBodies;
		std::vector<Constraint_c*>		m_constraints;
	};

}

#endif