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


#include <vector>
#include "MeshTools.h"
#include "VHACDDemo.h"
#include "cd_wavefront.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision\CollisionShapes\btBvhTriangleMeshShape.h"
#include "BulletCollision\CollisionShapes\btTriangleMesh.h"
#include "BulletCollision\CollisionShapes\btCompoundShape.h"
#include "BulletCollision\CollisionShapes\btConvexHullShape.h"

#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


struct VHACDDemo : public CommonRigidBodyBase
{
	VHACDDemo(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	btCompoundShape* createCompoundFromObj(ConvexDecomposition::WavefrontObj& meshObj);
	std::vector<btBvhTriangleMeshShape*> splitMeshesFromObj(ConvexDecomposition::WavefrontObj& meshObj, int splitTimes);
	virtual ~VHACDDemo(){}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 41;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3] = { 0, 0.46, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
};


std::vector<btBvhTriangleMeshShape*> VHACDDemo::splitMeshesFromObj(ConvexDecomposition::WavefrontObj& meshObj, int splitTimes)
{
	std::vector<btBvhTriangleMeshShape*> newMeshes;
	// Set Up Mesh from OBJ
	std::vector<btVector3> origVert;
	std::vector<unsigned int> origIndex;
	for (int i = 0; i < meshObj.mVertexCount; i++)
	{
		origVert.push_back(btVector3(meshObj.mVertices[3 * i + 0],
			meshObj.mVertices[3 * i + 1],
			meshObj.mVertices[3 * i + 2]));
	}
	for (int i = 0; i < meshObj.mTriCount; i++)
	{
		origIndex.push_back(meshObj.mIndices[3 * i + 0]);
		origIndex.push_back(meshObj.mIndices[3 * i + 1]);
		origIndex.push_back(meshObj.mIndices[3 * i + 2]);
	}
	MeshTools::TriangleMeshData originalMesh(origVert, origIndex);

	// Split the Mesh Data
	std::vector<MeshTools::TriangleMeshData> meshesToSplit = { originalMesh };
	std::vector<MeshTools::TriangleMeshData> splitMeshes;
	for (int i = 0; i < splitTimes; i++)
	{
		btVector3 splittingPlane;
		switch (i % 3)
		{
		case 0: splittingPlane = btVector3(1, 0, 0); break;
		case 1: splittingPlane = btVector3(0, 1, 0); break;
		case 2: splittingPlane = btVector3(0, 0, 1); break;
		default:;
		}
		
		btVector3 splittingPoint = btVector3(0, 0, 0);
		splitMeshes.clear();
		for (unsigned int i = 0; i < meshesToSplit.size(); i++)
		{
			MeshTools::SplitMeshResult resultMeshes = MeshTools::MeshTools::SplitMeshSlow(meshesToSplit[i],
				splittingPlane,
				splittingPoint);
			if (resultMeshes.leftMesh.indices.size())
				splitMeshes.push_back(resultMeshes.leftMesh);
			if (resultMeshes.rightMesh.indices.size())
				splitMeshes.push_back(resultMeshes.rightMesh);
		}
		meshesToSplit = splitMeshes;
	}

	// Convert to btTriangleMesh
	for (unsigned int i = 0; i < splitMeshes.size(); i++)
	{
		btTriangleMesh* newMesh = new btTriangleMesh();
		std::vector<unsigned int>& indices = splitMeshes[i].indices;
		std::vector<btVector3>& vertices = splitMeshes[i].vertices;
		
		//skip empty meshes.
		if (!vertices.size() || !indices.size())
			continue;

		for (unsigned int i = 0; i < indices.size(); i += 3)
		{
			newMesh->addTriangle(vertices[indices[i + 0]],
				vertices[indices[i + 1]],
				vertices[indices[i + 2]],
				true); /* remove duplicate vertices */
		}

		newMeshes.push_back(new btBvhTriangleMeshShape(newMesh, true, true));
	}

	return newMeshes;
}

btCompoundShape* VHACDDemo::createCompoundFromObj(ConvexDecomposition::WavefrontObj& meshObj)
{
	std::vector<float> vert;
	std::vector<int> index;
	vert.resize(meshObj.mVertexCount * 3);
	index.resize(meshObj.mTriCount * 3);
	for (int i = 0; i < meshObj.mVertexCount; i++)
	{
		vert[3 * i + 0] = meshObj.mVertices[3 * i + 0];
		vert[3 * i + 1] = meshObj.mVertices[3 * i + 1];
		vert[3 * i + 2] = meshObj.mVertices[3 * i + 2];
	}

	for (int i = 0; i < meshObj.mTriCount; i++)
	{
		index[3 * i + 0] = meshObj.mIndices[3 * i + 0];
		index[3 * i + 1] = meshObj.mIndices[3 * i + 1];
		index[3 * i + 2] = meshObj.mIndices[3 * i + 2];
	}

	VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
	VHACD::VHACD::Parameters params;
	params.Init();
	params.m_concavity = .01;
	params.m_resolution = 100000;
	interfaceVHACD->Compute(&vert[0], 3, (unsigned int)vert.size() / 3, &index[0], 3, (unsigned int)index.size() / 3, params);
	int numConvex = interfaceVHACD->GetNConvexHulls();

	btCompoundShape* compoundShape = new btCompoundShape();

	for (int i = 0; i < numConvex; i++)
	{
		btConvexHullShape* convexHullShape = new btConvexHullShape();
		VHACD::VHACD::ConvexHull convexHull;
		interfaceVHACD->GetConvexHull(i, convexHull);
		for (int j = 0; j < convexHull.m_nPoints; j++)
		{
			bool recalcAABB = false;
			if (j == convexHull.m_nPoints - 1)
			{
				recalcAABB = true;
			}
			btScalar vertx = convexHull.m_points[3 * j + 0];
			btScalar verty = convexHull.m_points[3 * j + 1];
			btScalar vertz = convexHull.m_points[3 * j + 2];
			btVector3 vert(vertx, verty, vertz);
			convexHullShape->addPoint(vert, recalcAABB);
		}
		btTransform compTrans;
		compTrans.setIdentity();
		compTrans.setOrigin(btVector3(0, 0, 0));
		compoundShape->addChildShape(compTrans, convexHullShape);
	}
	interfaceVHACD->Release();
	return compoundShape;
}

void VHACDDemo::initPhysics()
{
	// Load Mesh Triangle Mesh
	const char* fileName = "../../data/cylinder.objobj";
	ConvexDecomposition::WavefrontObj mesh_obj;
	mesh_obj.loadObj(fileName);
	btTriangleMesh* mesh = new btTriangleMesh();
	for (int i = 0; i < mesh_obj.mTriCount; i++)
	{
		btScalar vert1x = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i] + 0];
		btScalar vert1y = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i] + 1];
		btScalar vert1z = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i] + 2];
		btVector3 vert1(vert1x, vert1y, vert1z);
		btScalar vert2x = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i + 1] + 0];
		btScalar vert2y = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i + 1] + 1];
		btScalar vert2z = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i + 1] + 2];
		btVector3 vert2(vert2x, vert2y, vert2z);
		btScalar vert3x = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i + 2] + 0];
		btScalar vert3y = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i + 2] + 1];
		btScalar vert3z = mesh_obj.mVertices[3 * mesh_obj.mIndices[3 * i + 2] + 2];
		btVector3 vert3(vert3x, vert3y, vert3z);
		mesh->addTriangle(vert1, vert2, vert3, true);
	}
	btBvhTriangleMeshShape* baseMesh = new btBvhTriangleMeshShape(mesh, true, true);
	btCompoundShape* compoundShape = createCompoundFromObj(mesh_obj);
	std::vector<btBvhTriangleMeshShape*> splitMeshes = splitMeshesFromObj(mesh_obj, 3);
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));


	//groundShape->initializePolyhedralFeatures();
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);
	m_collisionShapes.push_back(baseMesh);
	m_collisionShapes.push_back(compoundShape);
	for (unsigned int i = 0; i < splitMeshes.size(); i++)
	{
		m_collisionShapes.push_back(splitMeshes[i]);
	}

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));

	{
		btScalar mass(0.);
		btRigidBody* body = createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	// put split meshes into World
	for (unsigned int i = 0; i < splitMeshes.size(); i++)
	{
		btTransform meshTransform;
		meshTransform.setIdentity();
		meshTransform.setOrigin(btVector3(60 - 1 * i, 20 - 1 * i, 0));
		createRigidBody(0., meshTransform, splitMeshes[i]);
	}

	// Put mesh into World
	btTransform meshTransform;
	meshTransform.setIdentity();
	meshTransform.setOrigin(btVector3(20, 20, 0));
	createRigidBody(0., meshTransform, baseMesh);


	// Put VHACD into World
	btTransform meshDecompTransform;
	meshDecompTransform.setIdentity();
	meshDecompTransform.setOrigin(btVector3(-20, 40, 0));
	createRigidBody(20., meshDecompTransform, compoundShape);


	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void VHACDDemo::renderScene()
{
	CommonRigidBodyBase::renderScene();

}







CommonExampleInterface*    VHACDDemoCreateFunc(CommonExampleOptions& options)
{
	return new VHACDDemo(options.m_guiHelper);
}



