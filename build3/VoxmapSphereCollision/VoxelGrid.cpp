#include "stdafx.h"
#include "VoxelGrid.h"

namespace VSC
{
void VoxelGrid::setVoxel(const Vector3int32& pos, int32_t state)
{
	insertAt(pos, state);
}

const int32_t* VoxelGrid::getVoxel(const Vector3int32& pos) const
{
	return getAt(pos);
}

void VoxelGrid::clearVoxel(const Vector3int32& pos)
{
	eraseAt(pos);
}

}; //namespace VSC