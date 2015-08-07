#include "btPGSSolverWrapper.h"
#include "CollisionConstraint.h"
#include "BulletCollision\NarrowPhaseCollision\btPersistentManifold.h"
#include <DirectXMath.h>

using namespace DirectX;

btScalar btPGSSolverWrapper::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher)
{
	//
	// SETTING UP BODIES
	//
	for (int i = 0; i < numBodies; i++)
	{
		bodies[i]->setCompanionId(-1);
	}

	m_rigidBodies.reserve(numBodies + 1);
	m_rigidBodies.resize(0);

	for (int i = 0; i < numBodies; i++)
	{
		int bodyId = getOrInitSolverBody(*bodies[i], info.m_timeStep);
	}

	convertManifoldPtsToConstraints(manifold, numManifolds, info.m_timeStep);

	ComputeJointConstraints(info.m_timeStep);

	updateBodiesWithNewVelocitiesAndForces(bodies, numBodies, info.m_timeStep);

	for (int i = 0; i < m_constraints.size(); i++)
	{
		delete m_constraints[i];
	}
	clearConstraints();

	// TODO -> ADD CONSTRAINTS
	return 0.0f;
}

void btPGSSolverWrapper::reset()
{
	clearRigidBodies();
	clearConstraints();
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

void btPGSSolverWrapper::convertManifoldPtsToConstraints(btPersistentManifold** manifold, int numManifolds, btScalar timeStep)
{
	for (int i = 0; i < numManifolds; i++)
	{
		btCollisionObject *colObj0 = 0, *colObj1 = 0;
		colObj0 = (btCollisionObject*)manifold[i]->getBody0();
		colObj1 = (btCollisionObject*)manifold[i]->getBody1();

		int solverBodyIdA = getOrInitSolverBody(*colObj0, timeStep);
		int solverBodyIdB = getOrInitSolverBody(*colObj1, timeStep);

		PGSSOlver::RigidBody_c* rbA = m_rigidBodies[solverBodyIdA];
		PGSSOlver::RigidBody_c* rbB = m_rigidBodies[solverBodyIdB];

		for (int j = 0; j < manifold[i]->getNumContacts(); j++)
		{
			btManifoldPoint& cp = manifold[i]->getContactPoint(j);
			if (cp.getDistance() <= manifold[i]->getContactProcessingThreshold())
			{
				btVector3 rel_posA = cp.getPositionWorldOnA() - colObj0->getWorldTransform().getOrigin();
				btVector3 rel_posB = cp.getPositionWorldOnB() - colObj1->getWorldTransform().getOrigin();
				XMFLOAT3 relativePositionA = XMFLOAT3(rel_posA.x(), rel_posA.y(), rel_posA.z());
				XMFLOAT3 relativePositionB = XMFLOAT3(rel_posB.x(), rel_posB.y(), rel_posB.z());
				XMFLOAT3 normal            = XMFLOAT3(cp.m_normalWorldOnB.x(), cp.m_normalWorldOnB.y(), cp.m_normalWorldOnB.z());
				PGSSOlver::CollisionConstraint* contactConstraint = new PGSSOlver::CollisionConstraint(rbA, rbB, relativePositionA, relativePositionB, normal);

				// Need to optimize, Push_back is too expensive
				m_constraints.push_back(contactConstraint);
			}
		}
	}
}

void btPGSSolverWrapper::updateBodiesWithNewVelocitiesAndForces(btCollisionObject** bodies, int numBodies, btScalar timeStep)
{
	for (int i = 0; i < numBodies; i++)
	{
		btRigidBody* btRB = bodies[i] ? btRigidBody::upcast(bodies[i]) : 0;
		int bodyId = getOrInitSolverBody(*bodies[i], timeStep);
		PGSSOlver::RigidBody_c* rb = m_rigidBodies[bodyId];

		btRB->getWorldTransform().setOrigin(btVector3(rb->m_position.x, rb->m_position.y, rb->m_position.z));
		btRB->getWorldTransform().setRotation(btQuaternion(rb->m_orientation.x, rb->m_orientation.y, rb->m_orientation.z, rb->m_orientation.z));
		btRB->setLinearVelocity(btVector3(rb->m_linearVelocity.x, rb->m_linearVelocity.y, rb->m_linearVelocity.z));
		btRB->setAngularVelocity(btVector3(rb->m_angularVelocity.x, rb->m_angularVelocity.y, rb->m_angularVelocity.z));
	}
}