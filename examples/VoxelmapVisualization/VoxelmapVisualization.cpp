#include "../build3/VoxmapSphereCollision/VoxelGridFactory.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"


struct VoxelmapVisualization : public CommonRigidBodyBase
{
	VoxelmapVisualization(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{}
	virtual ~VoxelmapVisualization() {}
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

void VoxelmapVisualization::initPhysics()
{
	// Default Stuff?
	m_guiHelper->setUpAxis(1);
	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	
	// Make Voxel Grid
	using namespace VSC;
	std::vector<float> boxVert;
	std::vector<size_t> boxInd;
	VoxelGridFactory::debug_MakeBoxVertexIndices(Vector3(1.2f), Vector3(0.0f), boxVert, boxInd);
	float voxelWidth = 0.2f; // With box size 1.2f, we should have center voxel empty, but immediately surrounded voxels full.
	VoxelGrid* resultGrid = VoxelGridFactory::generateVoxelGridFromMesh((const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth);

	const Vector3int32& min = resultGrid->getGridDescConst().min;
	const Vector3int32& max = resultGrid->getGridDescConst().max;
	btScalar mass(0.f);

	btVector3 size = btVector3(1, 1, 1);
	btBoxShape* colShape = createBoxShape(size);
	const Vector3int32& delta = max - min;


	// Make 5 different shapes for 
	// -2, -1, 0,  1,  2
	btVector3 offset[5];
	for (int i = 0; i < 5; i++)
	{
		int indexOffset = i - 2;
		offset[i] = btVector3(delta.x * indexOffset * 4.0, 0, 0);
	}
	
	btTransform startTransform;
	startTransform.setIdentity();
	for (int32_t x = min.x; x <= max.x; x++)
	{
		for (int32_t y = min.y; y <= max.y; y++)
		{
			for (int32_t z = min.z; z <= max.z; z++)
			{
				if (const int32_t* value = resultGrid->getVoxel(Vector3int32(x, y, z)))
				{
					for (int i = -2; i <= 2; i++)
					{
						int indexOffset = i + 2;
						if (*value == i)
						{
							startTransform.setOrigin(btVector3(x, y, z) * size * 2.0 + offset[indexOffset]);
							btRigidBody* body = createRigidBody(mass, startTransform, colShape);
						}
					}
				}
			}
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void VoxelmapVisualization::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface*    VoxelmapVisualizationCreateFunc(CommonExampleOptions& options)
{
	return new VoxelmapVisualization(options.m_guiHelper);
}