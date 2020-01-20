#include "stdafx.h"
#include "VoxelGrid.h"
#include "Math/Geometry2D.h"
#include "Math/Geometry3D.h"
#include <math.h>
#include <vector>
#include <assert.h>

namespace VSC
{
void VoxelGrid::setVoxel(const Vector3int32& pos, int32_t state)
{
	const int32_t* currentState = getVoxel(pos);
	if (currentState && *currentState == 0)
	{
		assert(surfaceCount);
		surfaceCount--;
	}
	if (state == 0)
	{
		surfaceCount++;
	}

	insertAt(pos, state);
}

}; //namespace VSC