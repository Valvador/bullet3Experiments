#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

class DMatrix
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

	inline float Get(int row, int col = 0) const
	{
		assert(row >= 0 && row < m_numRows);
		assert(col >= 0 && col < m_numCols);
		const int index = GetDataIndex(row, col);
		return m_data[index];
	}

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
	
};