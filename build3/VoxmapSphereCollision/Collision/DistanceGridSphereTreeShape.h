#include "VoxelGrid.h"

namespace VSC
{
	class DistanceGridSphereTreeShape
	{
	private:
		VoxelGrid* roughGrid;
		VoxelGridDistanceField* fineGrid;
		// todo, add SphereTree
	public:
		DistanceGridSphereTreeShape() {};
	};
}