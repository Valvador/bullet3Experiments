#include "VoxelGrid.h"
#include "SphereTree.h"
#include "CollisionShape.h"

namespace VSC
{
	class DistanceGridSphereTreeShape : public CollisionShape
	{
	private:
		VoxelGrid* roughGrid = nullptr;
		VoxelGridDistanceField* fineGrid = nullptr;
		SphereTree* sphereTree = nullptr;
		
	public:
		DistanceGridSphereTreeShape(VoxelGrid* vGrid, VoxelGridDistanceField* distanceField, SphereTree* sTree)
			:roughGrid(vGrid)
			,fineGrid(distanceField)
			,sphereTree(sTree)
		{}

		DistanceGridSphereTreeShape() 
		{}
	};
}