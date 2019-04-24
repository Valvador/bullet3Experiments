#pragma once
#include "Grid.h"
#include "Math/Vector3.h"

namespace VSC
{
struct VoxelGridDesc
{
public:
	// Defines Width of 1 Voxel
	float voxWidth;
	Vector3 min;
	Vector3 max;

	VoxelGridDesc(float _voxWidth, const Vector3* verts, size_t numVerticess);

	Vector3int32 coordToGrid(const Vector3& coord);
	void minMaxCoordsOfGrid(const Vector3int32& gridId, Vector3& min, Vector3& max);
};

class VoxelGrid : public SparseGrid<int32_t>
{
public:
	// Base API
	void setVoxel(const Vector3int32& pos, int32_t state);
	const int32_t* getVoxel(const Vector3int32& pos) const;
	void clearVoxel(const Vector3int32& pos);

	VoxelGrid(VoxelGridDesc desc)
	: gridDesc(desc)
	{};

	// Used during Grid Generation
	void fillGridWithTriangleSurfaceVoxels(const Vector3& v0, const Vector3& v1, const Vector3& v2);

protected:

	VoxelGridDesc gridDesc;
};
}; //namespace VSC