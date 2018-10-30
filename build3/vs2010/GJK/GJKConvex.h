#include "btVector3.h"
#include "btTransform.h"
#include "btAlignedObjectArray.h"
#include <utility>

#pragma once

namespace ProjGJK
{
	__declspec(align(16)) class GJKConvex
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

		virtual ~GJKConvex() {};

		void* operator new(size_t i)
		{
			return _mm_malloc(i, 16);
		}

		void operator delete(void* p)
		{
			_mm_free(p);
		}
	};

	__declspec(align(16)) class GJKConvexHull : public GJKConvex
	{
	private: 		
		btAlignedObjectArray<btVector3>				vertices;
		btAlignedObjectArray<std::pair<int, int>>	edges;
	public:
		const btVector3 getSupport(const btVector3& direction) override;

		GJKConvexHull() :
			GJKConvex()
		{};
		GJKConvexHull(const btTransform& transform, const btAlignedObjectArray<btVector3>& vertices, const btAlignedObjectArray<std::pair<int, int>>& edges) :
			GJKConvex(transform, true),
			vertices(vertices),
			edges(edges)
		{};

		void* operator new(size_t i)
		{
			return _mm_malloc(i, 16);
		}

		void operator delete(void* p)
		{
			_mm_free(p);
		}
	};
}