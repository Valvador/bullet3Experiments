/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "BasicExampleCustomSolver.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


struct BasicExampleCustomSolver : public CommonRigidBodyBase
{
	BasicExampleCustomSolver(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExampleCustomSolver(){}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 41;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};

void BasicExampleCustomSolver::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld(true);
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	//btCylinderShape* groundShape = new btCylinderShape(btVector3(30, 5, 30));
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(40.),btScalar(3.),btScalar(40.)));
	

	//groundShape->initializePolyhedralFeatures();
    //btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	btTransform groundTransform2;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0,0));
	btQuaternion groundQ = groundTransform.getRotation();
	groundQ.setEuler(0, .30, 0);
	groundTransform.setRotation(groundQ);
	groundTransform2.setIdentity();
	groundTransform2.setOrigin(btVector3(0, 0, +70));
	btQuaternion groundQ2 = groundTransform2.getRotation();
	groundQ2.setEuler(0,-.30, 0);
	groundTransform2.setRotation(groundQ2);

	{
		btScalar mass(0.);
		btRigidBody* body	= createRigidBody(mass, groundTransform,  groundShape, btVector4(0,0,1,1));
		btRigidBody* body2	= createRigidBody(mass, groundTransform2, groundShape, btVector4(0, 0, 1, 1));
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(1,1,1));
		btSphereShape* sphereShape = new btSphereShape(1.0);
		//btSphereShape* colShape = new btSphereShape(1.0);
		

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);


		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										btScalar(2.0*i),
										btScalar(10+2.0*k),
										btScalar(2.0*j)));

					btQuaternion blockQQ = startTransform.getRotation();
					//blockQQ.setEuler(0, .20, 0);
					startTransform.setRotation(blockQQ);
			
					btRigidBody* body = createRigidBody(mass,startTransform,colShape);
					

				}
			}
		}

		for (int k = 0; k<ARRAY_SIZE_Y; k++)
		{
			for (int i = 0; i<ARRAY_SIZE_X; i++)
			{
				for (int j = 0; j<ARRAY_SIZE_Z; j++)
				{
					startTransform.setOrigin(btVector3(
						btScalar(2.0*i),
						btScalar(15 + 2.0*k),
						btScalar(2.0*j)));

					btQuaternion blockQQ = startTransform.getRotation();
					//blockQQ.setEuler(0, .20, 0);
					startTransform.setRotation(blockQQ);

					btRigidBody* body = createRigidBody(mass, startTransform, sphereShape);


				}
			}
		}

	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void BasicExampleCustomSolver::renderScene()
{
	CommonRigidBodyBase::renderScene();
	
}


CommonExampleInterface*    BasicExampleCustomSolverCreateFunc(CommonExampleOptions& options)
{
	return new BasicExampleCustomSolver(options.m_guiHelper);
}



