#include "btVector3.h"

// GJK IMPLEMETNATION USING https://github.com/kevinmoran/GJK/blob/master/GJK.h AS REFERENCE
/// 

#pragma once

namespace ProjGJK
{
	class GJKConvex;

	struct GJKContactResult
	{
		btVector3 normal;
		btVector3 position;
		float depth;
	};

	struct GJKSimplex
	{
		btVector3 vertices[4];
		GJKSimplex() // Uninitialized
		{};
		GJKSimplex( const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& d) :
			vertices{ a,b,c,d }
		{};

		void updateSimplexVertex(int i, const btVector3& vertex);
		void updateSimplex3(int &simpDim, btVector3& searchDir);
		bool updateSimplex4CheckInside(int &simpDim, btVector3& searchDir);
		GJKContactResult doEPAForSimplexContact(GJKConvex* shape0, GJKConvex* shape1);
	};

	class GJKPair
	{
	private:
		GJKConvex* obj0;
		GJKConvex* obj1;
	public:
		GJKPair(GJKConvex* shape0, GJKConvex* shape1) :
			obj0(shape0),
			obj1(shape1)
		{};

		bool doGJK(GJKContactResult* resultOut = nullptr);
	};
}