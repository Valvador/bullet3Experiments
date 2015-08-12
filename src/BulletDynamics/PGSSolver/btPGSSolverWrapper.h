#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"

#include "InhousePGSSOlver.h"
#include <DirectXMath.h>
using namespace DirectX;




class btPGSSolverWrapper: public PGSSOlver::Solver, public btConstraintSolver
{
protected:
	PGSSOlver::Solver* solver;
	int m_fixedBodyId;

public:
	btPGSSolverWrapper()	{ m_fixedBodyId = -1;  solver = new PGSSOlver::Solver(); }
	~btPGSSolverWrapper()	{ delete solver; }
	// overrides
	btScalar solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher) override;
	void reset() override;
	btConstraintSolverType getSolverType() const override { return BT_SEQUENTIAL_IMPULSE_SOLVER; }

	int getOrInitSolverBody(btCollisionObject& body, btScalar timestep);
	void initSolverBody(PGSSOlver::RigidBody_c* solverBody, btCollisionObject* collisionObj, btScalar timeStep);
	void convertManifoldPtsToConstraints(btPersistentManifold** manifold, int numManifolds, btScalar timeStep);
	void updateBodiesWithNewVelocitiesAndForces(btCollisionObject** bodies, int numBodies, btScalar timeStep);


};