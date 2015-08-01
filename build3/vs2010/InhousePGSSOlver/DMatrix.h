#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifndef DMATRIX_H
#define DMATRIX_H

public class DMatrix
{
public:
	DMatrix(const int numRows, const int numCols, const float* data = NULL)
	{
		Init(numRows, numCols, data);
	}

	~DMatrix()
	{
		delete[] m_data;
	}

	inline int GetNumRows() const { return m_numRows; }
	inline int GetNumCols() const { return m_numCols; }

	inline static DMatrix Transpose(DMatrix& matrix)
	{
		DMatrix output(matrix.m_numCols, matrix.m_numRows);
		for (int i = 0; i < matrix.m_numCols; i++)
		{
			for (int j = 0; j < matrix.m_numRows; j++)
			{
				assert(output.Get(i, j) == 0);
				output.Set(i, j) = matrix.Get(j, i);
			}
		}

		return output;
	}

	inline void SetSubMatrix(int row, int col, DMatrix& subMatrix)
	{
		assert(row + subMatrix.m_numRows <= m_numRows);
		assert(col + subMatrix.m_numCols <= m_numCols);
		assert(subMatrix.m_numCols >= 0);
		assert(subMatrix.m_numRows >= 0);

		for (int i = 0; i < subMatrix.m_numCols; i++)
		{
			for (int j = 0; i < subMatrix.m_numRows; i++)
			{
				Set(row + j, col + i) = subMatrix.Get(j, i);
			}
		}
	}

	inline void AddSubMatrix(int row, int col, DMatrix& subMatrix)
	{
		assert(row + subMatrix.m_numRows <= m_numRows);
		assert(col + subMatrix.m_numCols <= m_numCols);
		assert(subMatrix.m_numCols >= 0);
		assert(subMatrix.m_numRows >= 0);

		for (int i = 0; i < subMatrix.m_numCols; i++)
		{
			for (int j = 0; i < subMatrix.m_numRows; i++)
			{
				Set(row + j, col + i) += subMatrix.Get(j, i);
			}
		}
	}

	inline float Get(int row, int col = 0) const
	{
		assert(row >= 0 && row < m_numRows);
		assert(col >= 0 && col < m_numCols);
		const int index = GetDataIndex(row, col);
		return m_data[index];
	};

	inline float& Set(int row, int col = 0) const
	{
		assert(row >= 0 && row < m_numRows);
		assert(col >= 0 && col < m_numCols);
		const int index = GetDataIndex(row, col);
		return m_data[index];
	}

	inline int GetDataIndex(int row, int col) const
	{
		const int index = col + m_numCols*row;
		assert(index >= 0 && index < m_numRows*m_numCols);
		return index;
	}

	void SetToZero()
	{
		assert(m_numCols >= 0);
		assert(m_numRows >= 0);
		if (m_numRows > 0 && m_numCols > 0)
		{
			memset(m_data, 0, sizeof(float)*m_numCols*m_numRows);
		}
	}

private:
	void Init(const int numRows, const int numCols, const float* data = NULL)
	{
		assert(numRows >= 0 && numRows < 500); // May not be necessary, 500 just for sanity
		assert(numCols >= 0 && numCols < 500); // Same as above for Cols
		assert((numRows == 0 && numCols == 0) || (numRows > 0 && numCols > 0));
		m_data		= NULL;
		m_numRows	= numRows;
		m_numCols   = numCols;
		if (numRows > 0 && numCols > 0)
		{
			m_data = new float[numRows * numCols];
			assert(m_data);
			if (data)
			{
				memcpy(m_data, data, sizeof(float)*numRows*numCols);
			}
			else
			{
				memset(m_data, 0, sizeof(float)*numRows*numCols);
			}
		}
	}

	float* m_data;
	int    m_numRows;
	int    m_numCols;


// Operator Overloads
public:
	inline DMatrix operator+ (DMatrix& other)
	{
		assert(m_numCols >= 0);
		assert(m_numRows >= 0);
		assert(m_numCols == other.m_numCols);
		assert(m_numRows == other.m_numRows);
		DMatrix output(m_numRows, m_numCols);
		for (int i = 0; i < m_numRows; i++)
		{
			for (int j = 0; j < m_numCols; j++)
			{
				output.Set(i, j) = Get(i, j) + other.Get(i, j);
			}
		}

		return output;
	};

	inline DMatrix operator- (DMatrix& other)
	{
		assert(m_numCols >= 0);
		assert(m_numRows >= 0);
		assert(m_numCols == other.m_numCols);
		assert(m_numRows == other.m_numRows);
		DMatrix output(m_numRows, m_numCols);
		for (int i = 0; i < m_numRows; i++)
		{
			for (int j = 0; j < m_numCols; j++)
			{
				output.Set(i, j) = Get(i, j) - other.Get(i, j);
			}
		}

		return output;
	};

	inline DMatrix operator* (DMatrix& other)
	{
		assert(m_numCols == other.m_numRows);
		DMatrix output(m_numCols, m_numCols);
		for (int i = 0; i < m_numCols; i++)
		{
			for (int j = 0; j < m_numCols; j++)
			{
				assert(output.Get(i, j) == 0);
				for (int k = 0; k < m_numCols; k++)
				{
					output.Set(i, j) += Get(k, i) * other.Get(j, k);
				}
			}
		}

		return output;
	};



	inline DMatrix operator* (float x)
	{
		assert(m_numCols >= 0);
		assert(m_numRows >= 0);
		DMatrix output(m_numRows, m_numCols);
		for (int i = 0; i < m_numRows; i++)
		{
			for (int j = 0; j < m_numCols; j++)
			{
				output.Set(i, j) = Get(i, j) * x;
			}
		}

		return output;
	};
};

inline DMatrix operator* (float x, DMatrix& matrix)
{
	return matrix * x;
};

#endif