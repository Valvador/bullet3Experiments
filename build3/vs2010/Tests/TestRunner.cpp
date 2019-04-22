#include "TestRunner.h"
#include "GJKTest.h"
#include "DMatrixTests.h"
#include "VoxelmapPointshellTests.h"

TestRunner::TestRunner()
{
	// REGISTER ALL TESTS HERE
	registerTest(new DMatrixTests());
	//registerTest(new GJKTest());
	registerTest(new Geometry2DTest());
	registerTest(new Geometry3DTest());
	registerTest(new VoxelmapTest());
}

void TestRunner::registerTest(TestRunnerTest* test)
{
	tests.push_back(TestWrapper(test));
}

bool TestRunner::runTests()
{
	bool result = true;
	for (auto test : tests)
	{
		test.success = test.test->runTest();
		if (!test.success)
		{
			result = false;
		}
	}

	return result;
}