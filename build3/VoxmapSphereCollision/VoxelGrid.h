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
	Vector3int32 min;
	Vector3int32 max;

	VoxelGridDesc(float _voxWidth, const Vector3* verts, size_t numVerticess);
	void expandGridBy(const Vector3int32& expandBy);

	bool withinGrid(const Vector3int32& id) const;
	Vector3int32 coordToGrid(const Vector3& coord) const;
	Vector3 gridCenterToCoord(const Vector3int32& grid) const;
	void minMaxCoordsOfGrid(const Vector3int32& gridId, Vector3& min, Vector3& max) const;
};

class VoxelGrid : public SparseGrid<int32_t>
{
	friend class VoxelGridFactory;
public:
	// Base API
	void setVoxel(const Vector3int32& pos, int32_t state);
	const int32_t* getVoxel(const Vector3int32& pos) const;
	void clearVoxel(const Vector3int32& pos);

	VoxelGrid(VoxelGridDesc desc)
	: gridDesc(desc)
	, surfaceCount(0)
	{};

	const VoxelGridDesc& getGridDescConst() { return gridDesc; }
	size_t surfaceCount;

	size_t countSurfaceVoxels() { return surfaceCount; }

protected:
	VoxelGridDesc gridDesc;
};
}; //namespace VSC