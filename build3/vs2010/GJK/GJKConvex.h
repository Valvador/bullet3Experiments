#include "btVector3.h"
#include "btTransform.h"
#include "btAlignedObjectArray.h"

namespace ProjGJK
{
	class GJKConvex
	{
	private:
		bool initialized;
		btTransform transform;
	public:
		GJKConvex(const btTransform& tForm = btTransform(), bool init = false):
			initialized(init),
			transform(tForm)
		{};
		virtual const btVector3 getSupport(const btVector3& direction) = 0;
		const btTransform& getTransform() { return transform; }
	};

	class GJKConvexHull : GJKConvex
	{
	private: 		
		btAlignedObjectArray<btVector3> vertices;
		btAlignedObjectArray<int>		edges;
	public:
		const btVector3 getSupport(const btVector3& direction) override;

		GJKConvexHull() :
			GJKConvex()
		{};
		GJKConvexHull(const btTransform& transform, const btAlignedObjectArray<btVector3>& vertices, const btAlignedObjectArray<int>& edges) :
			GJKConvex(transform, true),
			vertices(vertices),
			edges(edges)
		{};
	};
}