#include "TestRunner.h"
#include "../InhousePGSSOlver/DMatrix.h"
#pragma once

class DMatrixTests : public TestRunnerTest
{
private:
	void generateNbyMRandMatrix(PGSSOlver::DMatrix& Matrix, int n, int m);

public: 
	bool runTest() override;
};
