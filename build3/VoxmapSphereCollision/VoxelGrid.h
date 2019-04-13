#pragma once
#include "Grid.h"

namespace VSC
{
class VoxelGrid : public SparseGrid<int32_t>
{
public:
	// Base API
	void setVoxel(const Vector3int32& pos, int32_t state);
	const int32_t* getVoxel(const Vector3int32& pos) const;
	void clearVoxel(const Vector3int32& pos);
};
}; //namespace VSC