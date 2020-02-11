#include "../build3/VoxmapSphereCollision/VoxelGridFactory.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include <string>
#include <algorithm>


struct VoxelmapVisualization : public CommonRigidBodyBase
{
	VoxelmapVisualization(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{}
	virtual ~VoxelmapVisualization() {}
	virtual void initPhysics();
	virtual void renderScene();
	virtual void physicsDebugDraw(int debugFlags);
	void resetCamera()
	{
		float dist = 41;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3] = { 0, 0.46, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}

	float voxelWidth;
	VSC::VoxelGrid* resultGrid;
	VSC::SparseGrid<VSC::Vector3> gridGradient;
	VSC::SparseGrid<VSC::Vector3> surfaceProjection;
	VSC::SphereTree* sphereTree;
};

void drawNthLayerSphereTreeNodes(int layer, GUIHelperInterface* guiHelper, VSC::SphereTreeNode<VSC::SphereTree::SphereTreeNodeMax>* treeNode, int currentLayer = 0)
{
	if (layer != currentLayer && treeNode->getNumChildren())
	{
		for (int i = 0; i < treeNode->getNumChildren(); i++)
		{
			drawNthLayerSphereTreeNodes(layer, guiHelper, treeNode->getChild(i), currentLayer + 1);
		}
	}
	else
	{
		// This is going to leak a lot of memory
		btSphereShape* sphere = new btSphereShape(std::max(treeNode->getRadius(), 0.02f));
		guiHelper->createCollisionShapeGraphicsObject(sphere);
		btCollisionObject* sphereObject = new btCollisionObject();
		sphereObject->setCollisionShape(sphere);
		btTransform tr;
		tr.setIdentity();
		VSC::Vector3 origin = treeNode->getPosition();
		tr.setOrigin(btVector3(origin.x, origin.y, origin.z));
		btVector3 color(0.0, 0.0, 0.5);
		sphereObject->setWorldTransform(tr);
		guiHelper->createCollisionObjectGraphicsObject(sphereObject, color);
	}
}

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
	voxelWidth = 0.2f; // With box size 1.2f, we should have center voxel empty, but immediately surrounded voxels full.
	resultGrid = VoxelGridFactory::generateVoxelGridFromMesh((const float*)&boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth);
	gridGradient = VoxelGridFactory::getVoxelGridGradient(resultGrid);
	surfaceProjection = VoxelGridFactory::getSurfaceProjection(gridGradient, (const float*)& boxVert[0], boxVert.size() / 3, &boxInd[0], boxInd.size() / 3, voxelWidth, resultGrid);
	sphereTree = VoxelGridFactory::generateSphereTreeFromSurfaceProjections(surfaceProjection);

	drawNthLayerSphereTreeNodes(3, m_guiHelper, sphereTree->getRootNode());
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void getRGB(float& r, float& g, float& b, float a)
{
	btAssert(a <= 1.0f && a >= 0.f);
	btVector3 red = btVector3(1, 0, 0);
	btVector3 green = btVector3(0, 1, 0);
	btVector3 blue = btVector3(0, 0, 1);

	btVector3 layer0 = red * a + green * (1 - a);
	btVector3 layer1 = green * a + blue * (1 - a);
	btVector3 finalColor = layer0 * a  + layer1 * (1 - a);

	r = finalColor.x();
	g = finalColor.y();
	b = finalColor.z();
}

static void drawCircleFromLines(const VSC::Vector3& position, float radius)
{
	// Draw lines to form a circle here.
}

void VoxelmapVisualization::physicsDebugDraw(int debugFlags)
{
	btTransform startTransform;
	startTransform.setIdentity();
	using namespace VSC;
	const Vector3int32& min = resultGrid->getGridDescConst().min;
	const Vector3int32& max = resultGrid->getGridDescConst().max;

	// Render Grid
	const VSC::GridDesc& gridDesc = resultGrid->getGridDescConst();
	for (int32_t x = min.x; x <= max.x; x++)
	{
		for (int32_t y = min.y; y <= max.y; y++)
		{
			for (int32_t z = min.z; z <= max.z; z++)
			{
				if (const int32_t * value = resultGrid->getVoxel(Vector3int32(x, y, z)))
				{
					for (int i = -2; i <= 2; i++)
					{
						int indexOffset = i + 2;
						if (*value == i)
						{
							VSC::Vector3 position = gridDesc.gridCenterToCoord(Vector3int32(x, y, z));;
							float r, g, b;
							getRGB(r, g, b, (i + 2) / 5.0f);
							m_guiHelper->drawText3D(std::to_string(i).c_str(), position.x, position.y, position.z, 1.0f, r,g,b);

							// Gradient
							if (const VSC::Vector3 * value = gridGradient.getAt(Vector3int32(x, y, z)))
							{
								VSC::Vector3 gradient = *value * voxelWidth * 2.0f;
								VSC::Vector3 startPos = position + gradient * 0.5f;
								VSC::Vector3 endPos = position - gradient * 0.5f;
								m_guiHelper->drawLine3D(startPos.x, startPos.y, startPos.z, endPos.x, endPos.y, endPos.z, r, g, b, 1.0f, 1.5f);
							}

							// Surface Projection
							if (const VSC::Vector3 * surfacePt = surfaceProjection.getAt(Vector3int32(x, y, z)))
							{
								m_guiHelper->drawPoint3D(surfacePt->x, surfacePt->y, surfacePt->z, r, g, b, 1.0f, 10.0f);
							}
						}
					}
				}
			}
		}
	}
}

void VoxelmapVisualization::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface*    VoxelmapVisualizationCreateFunc(CommonExampleOptions& options)
{
	return new VoxelmapVisualization(options.m_guiHelper);
}