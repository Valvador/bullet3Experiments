#include "VoxelGrid.h"
#include "SphereTree.h"
#include "DistanceGridSphereTreeShape.h"
#include "CollisionAlgorithm.h"

const int numContactDistanceGridSphereTree = 4;

namespace VSC
{
class DistanceGridSphereTreeCollisionAlgorithm : public CollisionAlgorithm
{
private:
	int numCollisions = 0;
	ContactPoint collisions[numContactDistanceGridSphereTree];

public:
	virtual int getMaxContacts() override { return numContactDistanceGridSphereTree; }
	virtual int getNumContacts() override { return numCollisions; }
	virtual const ContactPoint& getContactPoint(int i) override { assert(i < numCollisions); return collisions[i]; }
	virtual bool doCollide() override;
};
}; //namespace VSC