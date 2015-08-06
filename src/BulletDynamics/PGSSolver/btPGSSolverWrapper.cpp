#include "btPGSSolverWrapper.h"
#include <DirectXMath.h>

using namespace DirectX;

btScalar btPGSSolverWrapper::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher)
{
	// TODO -> NEED TO SET UP BODIES, AND SET UP CONSTRAINTS USING THE MANIFOLD
	for (int i = 0; i < numBodies; i++)
	{
		bodies[i]->setCompanionId(-1);
	}

	m_rigidBodies.reserve(numBodies + 1);
	m_rigidBodies.resize(0);

	for (int i = 0; i < numBodies; i++)
	{
		int bodyId = getOrInitSolverBody(*bodies[i], 0.01666f);
	}

	// TODO -> ADD CONSTRAINTS
	return 0.0f;
}

int btPGSSolverWrapper::getOrInitSolverBody(btCollisionObject& body, btScalar timeStep)
{
	int solverBodyIdA = -1;

	if (body.getCompanionId() >= 0)
	{
		//body has already been converted
		solverBodyIdA = body.getCompanionId();
		btAssert(solverBodyIdA < m_rigidBodies.size());
	}
	else
	{
		btRigidBody* rb = btRigidBody::upcast(&body);
		//convert both active and kinematic objects (for their velocity)
		if (rb && (rb->getInvMass() || rb->isKinematicObject()))
		{
			solverBodyIdA = m_rigidBodies.size();
			PGSSOlver::RigidBody_c* solverBody = new PGSSOlver::RigidBody_c();
			m_rigidBodies.push_back(solverBody);
			initSolverBody(solverBody, &body, timeStep);
			body.setCompanionId(solverBodyIdA);
		}
			//			return 0;//assume first one is a fixed solver body
	}

	return solverBodyIdA;
}

void btPGSSolverWrapper::initSolverBody(PGSSOlver::RigidBody_c* solverBody, btCollisionObject* collisionObject, btScalar timeStep)
{
	btRigidBody* rb = collisionObject ? btRigidBody::upcast(collisionObject) : 0;

	if (rb)
	{
		solverBody->m_position			= XMFLOAT3(rb->getWorldTransform().getOrigin().x(), 
													rb->getWorldTransform().getOrigin().y(), 
													rb->getWorldTransform().getOrigin().z());
		solverBody->m_orientation		= XMFLOAT4(rb->getWorldTransform().getRotation().x(),
													rb->getWorldTransform().getRotation().y(),
													rb->getWorldTransform().getRotation().z(),
													rb->getWorldTransform().getRotation().w());
		solverBody->m_force				= XMFLOAT3(rb->getTotalForce().x(), rb->getTotalForce().y(), rb->getTotalForce().z());
		solverBody->m_invMass			= (float) rb->getInvMass();
		btMatrix3x3 iIT					= rb->getInvInertiaTensorWorld();
		solverBody->m_invInertia		= XMFLOAT3X3(iIT.getRow(0).x(), iIT.getRow(0).y(), iIT.getRow(0).z(),
													iIT.getRow(1).x(), iIT.getRow(1).y(), iIT.getRow(1).z(),
													iIT.getRow(2).x(), iIT.getRow(2).y(), iIT.getRow(2).z());
		solverBody->m_torque			= XMFLOAT3(rb->getTotalTorque().x(), rb->getTotalTorque().y(), rb->getTotalTorque().z());
		solverBody->m_linearVelocity	= XMFLOAT3(rb->getLinearVelocity().x(), rb->getLinearVelocity().y(), rb->getLinearVelocity().z());
		solverBody->m_angularVelocity	= XMFLOAT3(rb->getAngularVelocity().x(), rb->getAngularVelocity().y(), rb->getAngularVelocity().z());
	}
}