#include "TestRunner.h"
#include "GJKTest.h"
#include "DMatrixTests.h"

TestRunner::TestRunner()
{
	// REGISTER ALL TESTS HERE
	registerTest(new DMatrixTests());
	registerTest(new GJKTest());
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