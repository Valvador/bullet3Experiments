#include "Base/Grid.h"
#include "Math/Vector3int32.h"
#include "Math/Vector3.h"

namespace VSC
{
struct GridDesc
{
public:
	// Defines Width of 1 Voxel
	float voxWidth;
	Vector3int32 min;
	Vector3int32 max;

	GridDesc(float _voxWidth, const Vector3* verts, size_t numVerticess);
	GridDesc(float _voxWidth);
	void expandGridBy(const Vector3int32& expandBy);

	bool withinGrid(const Vector3int32& id) const;
	Vector3int32 coordToGrid(const Vector3& coord) const;
	Vector3 gridCenterToCoord(const Vector3int32& grid) const;
	void minMaxCoordsOfGridVoxel(const Vector3int32& gridId, Vector3& min, Vector3& max) const;
};

// class for storing T in buckets in a Spatial Grid
template <class T>
class SpatialSparseGrid : public SparseGrid<std::vector<T*>>
{
private:
	GridDesc desc;

	// Define this wherever this class is used.
	Vector3int32 getGridId(const T& t) const;
public:
	SpatialSparseGrid(float _voxWidth) : desc(_voxWidth) {};

	const GridDesc& getGridDesc() { return desc; }
	void addToSpatialGrid(T* t);
	void gatherNClosestToTarget(const T& target, std::vector<T*>& result, size_t N);
};

template<class T>
void SpatialSparseGrid<T>::addToSpatialGrid(T* t)
{
	Vector3int32 id = getGridId(*t);
	std::vector<T*>& bucket = operator[](id);
	bucket.push_back(t);

	// Define min and max IDs as we fill
	desc.min.setMinAxis(id);
	desc.max.setMaxAxis(id);
}

template <class T>
void SpatialSparseGrid<T>::gatherNClosestToTarget(const T& target, std::vector<T*>& result, size_t N)
{
	Vector3int32 id = getGridId(target);

	bool outOfBounds = false;
	int distanceFromCenter = 0;
	while (result.size() < N && !outOfBounds)
	{
		int iterationMin = (0 - distanceFromCenter);
		int iterationMax = distanceFromCenter;
		outOfBounds = true;

		// There must be some way to unwrap this so that we don't have to search
		// through a cube each time, since we only care about the edges.
		// I think I have an implementation that does this somewhere else in the code.
		for (int x = iterationMin; x <= iterationMax; x++)
		{
			for (int y = iterationMin; y <= iterationMax; y++)
			{
				for (int z = iterationMin; z <= iterationMax; z++)
				{
					// We only care about the edges
					if (x == iterationMin || x == iterationMax || 
						y == iterationMin || y == iterationMax || 
						z == iterationMin || z == iterationMax)
					{
						Vector3int32 lookupId = id + Vector3int32(x, y, z);
						if (desc.withinGrid(lookupId))
						{
							outOfBounds = false;
							if (const std::vector<T*>* bucket = getAt(lookupId))
							{
								if (bucket->size())
								{
									for (auto& item : *bucket)
									{
										result.push_back(item);
									}
								}
							}
						}
					}
				}
			}
		}

		// We expand search grid and only look at the new edges
		distanceFromCenter++;
	}
}
}