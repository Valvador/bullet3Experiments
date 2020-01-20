#pragma once
#include "Base/Grid.h"
#include "Math/Vector3.h"
#include "SpatialGrid.h"

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

class VoxelGrid : public VoxelGridBase<int32_t>
{
	friend class VoxelGridFactory;
protected:
	size_t surfaceCount;
	GridDesc gridDesc;
public:
	// Base API
	void setVoxel(const Vector3int32& pos, int32_t state);

	VoxelGrid(GridDesc desc)
	: gridDesc(desc)
	, surfaceCount(0)
	{};

	const GridDesc& getGridDescConst() const { return gridDesc; }
	size_t countSurfaceVoxels() { return surfaceCount; }
};

class VoxelGridDistanceField : public VoxelGridBase<float>
{
	friend class VoxelGridFactory;
public:
	VoxelGridDistanceField() {};
};

}; //namespace VSC