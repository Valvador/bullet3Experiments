#pragma once
#include "Grid.h"
#include "Math/Vector3.h"

namespace VSC
{
// VoxelGridBase
template <class T>
class VoxelGridBase : protected SparseGrid<T>
{
public:
	void setVoxel(const Vector3int32& pos, T state);
	const T* getVoxel(const Vector3int32& pos) const;
	void clearVoxel(const Vector3int32& pos);
};

template <class T>
void VoxelGridBase<T>::setVoxel(const Vector3int32& pos, T state)
{
	insertAt(pos, state);
}

template <class T>
const T* VoxelGridBase<T>::getVoxel(const Vector3int32& pos) const
{
	return getAt(pos);
}

template <class T>
void VoxelGridBase<T>::clearVoxel(const Vector3int32& pos)
{
	eraseAt(pos);
}
// VoxelGridBase End

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
	void minMaxCoordsOfGridVoxel(const Vector3int32& gridId, Vector3& min, Vector3& max) const;
};

class VoxelGrid : public VoxelGridBase<int32_t>
{
	friend class VoxelGridFactory;
protected:
	size_t surfaceCount;
	VoxelGridDesc gridDesc;
public:
	// Base API
	void setVoxel(const Vector3int32& pos, int32_t state);

	VoxelGrid(VoxelGridDesc desc)
	: gridDesc(desc)
	, surfaceCount(0)
	{};

	const VoxelGridDesc& getGridDescConst() const { return gridDesc; }
	size_t countSurfaceVoxels() { return surfaceCount; }
};

class VoxelGridDistanceField : public VoxelGridBase<float>
{
	friend class VoxelGridFactory;
public:
	VoxelGridDistanceField() {};
};

}; //namespace VSC