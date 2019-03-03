#include "DMatrixTests.h"
#include <stdlib.h>

using namespace PGSSOlver;

float random(float max)
{
	return static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / max));
}

void DMatrixTests::generateNbyMRandMatrix(DMatrix& matrix, int n, int m)
{
	matrix.Resize(n, m);
	for (int i = 0; i < matrix.GetNumRows(); i++)
	{
		for (int j = 0; j < matrix.GetNumCols(); j++)
		{
			matrix.Set(i, j) = random(1000.0f);
		}
	}
}

bool DMatrixTests::runTest()
{
	std::srand(1);
	// Verify Multiplication vs Column Product
	bool multResultCol = false;
	DMatrix a;
	DMatrix b;
	generateNbyMRandMatrix(a, 6, 4);
	generateNbyMRandMatrix(b, 4, 2);
	DMatrix a_b = a*b;

	DMatrix a_b_colProj;
	a_b_colProj.Resize(a_b.GetNumRows(), a_b.GetNumCols(), true);
	for (int j = 0; j < b.GetNumRows(); j++)
	{
		a_b_colProj.AddSubMatrix(0, 0, a.colProduct(b, j));
	}

	multResultCol = a_b_colProj.fuzzyEq(a_b);

	bool multResultRow = false;
	DMatrix a_b_rowProj;
	a_b_rowProj.Resize(a_b.GetNumRows(), a_b.GetNumCols(), true);
	for (int j = 0; j < a.GetNumRows(); j++)
	{
		a_b_rowProj.AddSubMatrix(j, 0, a.rowProduct(b, j));
	}
	multResultRow = a_b_rowProj.fuzzyEq(a_b);

	return multResultCol && multResultRow;
}