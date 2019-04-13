#pragma once
#include "Vector3int32.h"
#include <unordered_map>

namespace VSC
{
template <class T>
class SparseGrid
{
protected:
	// public API
	void insertAt(const Vector3int32& pos, const T& t);
	const T* getAt(const Vector3int32& pos) const;
	size_t eraseAt(const Vector3int32& pos);

	// Operators
	T& operator[](const Vector3int32& pos);

private:
	// Sparse Grid storage
	std::unordered_map<Vector3int32, T> gridMap;
};
}; //namespace VSC