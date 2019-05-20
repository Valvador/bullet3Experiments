#pragma once
#include "Math/Vector3int32.h"
#include <unordered_map>

namespace VSC
{
template <class T>
class SparseGrid
{
public:
	// public API
	void insertAt(const Vector3int32& pos, const T& t);
	const T* getAt(const Vector3int32& pos) const;
	size_t eraseAt(const Vector3int32& pos);
	size_t countVoxels();

	// Operators
	T& operator[](const Vector3int32& pos);

	// Iterator
	typename std::unordered_map<Vector3int32, T, Vector3int32::Hash>::iterator begin();
	typename std::unordered_map<Vector3int32, T, Vector3int32::Hash>::iterator end();
	typename std::unordered_map<Vector3int32, T, Vector3int32::Hash>::const_iterator cbegin() const;
	typename std::unordered_map<Vector3int32, T, Vector3int32::Hash>::const_iterator cend() const;

private:
	// Sparse Grid storage
	std::unordered_map<Vector3int32, T, Vector3int32::Hash> gridMap;
};

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
		return &gridMap.at(pos);
	}
	return nullptr;
}

template <class T>
size_t SparseGrid<T>::eraseAt(const Vector3int32& pos)
{
	return gridMap.erase(pos);
}

template <class T>
T& SparseGrid<T>::operator[](const Vector3int32& pos)
{
	return gridMap[pos];
}

template <class T>
size_t SparseGrid<T>::countVoxels()
{
	return gridMap.size();
}

template <class T>
typename std::unordered_map<Vector3int32, T, Vector3int32::Hash>::iterator SparseGrid<T>::begin()
{
	return gridMap.begin();
}

template <class T>
typename std::unordered_map<Vector3int32, T, Vector3int32::Hash>::iterator SparseGrid<T>::end()
{
	return gridMap.end();
}

template <class T>
typename std::unordered_map<Vector3int32, T, Vector3int32::Hash>::const_iterator SparseGrid<T>::cbegin() const
{
	return gridMap.cbegin();
}

template <class T>
typename std::unordered_map<Vector3int32, T, Vector3int32::Hash>::const_iterator SparseGrid<T>::cend() const
{
	return gridMap.cend();
}

}; //namespace VSC