#pragma once
#include "Grid.h"
#include "Math/Vector3.h"

namespace VSC
{
struct MeshToGrid
{
public:
	// Defines Width of 1 Voxel
	float voxWidth;
	Vector3 min;
	Vector3 max;
};

class VoxelGrid : public SparseGrid<int32_t>
{
public:
	// Base API
	void setVoxel(const Vector3int32& pos, int32_t state);
	const int32_t* getVoxel(const Vector3int32& pos) const;
	void clearVoxel(const Vector3int32& pos);

	VoxelGrid() {};
};
}; //namespace VSC