#include "stdafx.h"
#include "Grid.h"

namespace VSC
{
	template<class T>
	void SparseGrid<T>::insertAt(const Vector3int32& pos, const T& t)
	{
		gridMap[pos] = t;
	}

	template<class T>
	const T* SparseGrid<T>::getAt(const Vector3int32& pos) const
	{
		if (gridMap.count(pos))
		{
			return &gridMap[pos];
		}
		return nullptr;
	}

	template <class T>
	size_t SparseGrid<T>::eraseAt(const Vector3int32& pos)
	{
		gridMap.erase(pos);
	}

	template <class T>
	T& SparseGrid<T>::operator[](const Vector3int32& pos)
	{
		return gridMap[pos];
	}
}; //namespace VSC