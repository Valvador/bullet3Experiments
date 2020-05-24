#include "VoxelGrid.h"
#include "SphereTree.h"

namespace VSC
{
	class DistanceGridSphereTreeShape
	{
	private:
		VoxelGrid* roughGrid;
		VoxelGridDistanceField* fineGrid;
		SphereTree* sphereTree;
		// todo, add SphereTree
	public:
		DistanceGridSphereTreeShape() {};
	};
}