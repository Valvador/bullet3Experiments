#include <vector>
#pragma once

class TestRunnerTest
{
public: 
	bool virtual runTest() = 0;
};

class TestRunner
{
private:
	struct TestWrapper
	{
		TestRunnerTest* test;
		bool			success;

		TestWrapper(TestRunnerTest* t)
			: test(t)
			, success(false)
		{};
	};

	std::vector<TestWrapper> tests;

	void registerTest(TestRunnerTest* test);
public:
	TestRunner();
	bool runTests();
};
