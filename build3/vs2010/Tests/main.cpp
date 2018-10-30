#include "GJK.h"
#include "GJKConvex.h"
#include "btVector3.h"
#include "btTransform.h"
#include "btMatrix3x3.h"
#include "btAlignedObjectArray.h"

#include "GJKTest.h"
#include <assert.h>

int main()
{
	// GJKTest
	btVector3 partSize = btVector3(1.0f, 1.0f, 1.0f);
	btTransform originTForm = btTransform(btMatrix3x3(	1.0f, 0.0f, 0.0f, 
														0.0f, 1.0f, 0.0f,
														0.0f, 0.0f, 1.0f));
	btTransform collidingTForm = originTForm;
	collidingTForm.setOrigin(partSize / 2.0f);
	btTransform notCollidingTForm = originTForm;
	notCollidingTForm.setOrigin(partSize * 2.0f);
	ProjGJK::GJKConvexHull* obj0 = GJKTest::GJKTestHelper::newGJKConvexHullTetrahedron(partSize, btTransform(originTForm));
	ProjGJK::GJKConvexHull* obj1 = GJKTest::GJKTestHelper::newGJKConvexHullTetrahedron(partSize, btTransform(collidingTForm));
	ProjGJK::GJKConvexHull* obj2 = GJKTest::GJKTestHelper::newGJKConvexHullTetrahedron(partSize, btTransform(notCollidingTForm));

	ProjGJK::GJKPair collidingPair(obj0, obj1);
	ProjGJK::GJKPair nonCollidingPair(obj0, obj2);

	bool collidingGJK = collidingPair.doGJK();
	bool nonCollidingGJK = nonCollidingPair.doGJK();

	assert(collidingGJK);
	assert(!nonCollidingGJK);

	return 1;
}