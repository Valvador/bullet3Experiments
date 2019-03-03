#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <climits>

#ifndef DMATRIX_H
#define DMATRIX_H

namespace PGSSOlver {

	class DMatrix
	{
	public:
		DMatrix()
		{
			Init(0, 0, NULL);
		}

		DMatrix(const int numRows, const int numCols, const float* data = NULL)
		{
			Init(numRows, numCols, data);
		}

		DMatrix(const DMatrix &other)
		{
			Init(other.GetNumRows(), other.GetNumCols(), &other.m_data[0]);
		}

		~DMatrix()
		{
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

		inline static DMatrix multComponents(DMatrix& mA, DMatrix& mB)
		{
			assert(mA.m_numRows == mB.m_numRows);
			assert(mA.m_numCols == mB.m_numCols);
			DMatrix output(mA.m_numRows, mB.m_numCols);
			for (int i = 0; i < output.m_numRows; i++)
			{
				for (int j = 0; j < output.m_numCols; j++)
				{
					output.Set(i, j) = mA.Get(i, j) * mB.Get(i, j);
				}
			}

			return output;
		}

		inline void Resize(const int numRows, const int numCol, bool memReset = false)
		{
			if (memReset)
			{
				Init(numRows, numCol);
			}
			else
			{
				InitSize(numRows, numCol);
			}
		}

		inline void SetSubMatrix(int row, int col, const DMatrix& subMatrix)
		{
			assert(row + subMatrix.m_numRows <= m_numRows);
			assert(col + subMatrix.m_numCols <= m_numCols);
			assert(subMatrix.m_numCols >= 0);
			assert(subMatrix.m_numRows >= 0);

			for (int i = 0; i < subMatrix.m_numCols; i++)
			{
				for (int j = 0; j < subMatrix.m_numRows; j++)
				{
					Set(row + j, col + i) = subMatrix.Get(j, i);
				}
			}
		}

		inline void AddSubMatrix(int row, int col, const DMatrix& subMatrix)
		{
			assert(row + subMatrix.m_numRows <= m_numRows);
			assert(col + subMatrix.m_numCols <= m_numCols);
			assert(subMatrix.m_numCols >= 0);
			assert(subMatrix.m_numRows >= 0);

			for (int i = 0; i < subMatrix.m_numCols; i++)
			{
				for (int j = 0; j < subMatrix.m_numRows; j++)
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

		inline float& Set(int row, int col = 0)
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
				memset(&m_data[0], 0, sizeof(float)*m_numCols*m_numRows);
			}
		}

		inline void Print()
		{
			for (int j = 0; j < GetNumRows(); j++)
			{
				for (int i = 0; i < GetNumCols(); i++)
				{
					printf("%4.4f ", Get(j, i));
				}
				printf("\n");
			}
		}

	private:
		void InitSize(const int numRows, const int numCols)
		{
			assert(numRows >= 0 && numRows < 1000); // May not be necessary, 500 just for sanity
			assert(numCols >= 0 && numCols < 1000); // Same as above for Cols
			assert((numRows == 0 && numCols == 0) || (numRows > 0 && numCols > 0));
			m_numRows = numRows;
			m_numCols = numCols;

			size_t newSize = numRows * numCols;
			if (m_data.size() != newSize)
				m_data.resize(newSize);
		}

		void Init(const int numRows, const int numCols, const float* data = NULL)
		{
			InitSize(numRows, numCols);
			if (m_numRows > 0 && m_numCols > 0)
			{
				if (data)
				{
					memcpy(&m_data[0], data, sizeof(float)*numRows*numCols);
				}
				else
				{
					memset(&m_data[0], 0, sizeof(float)*numRows*numCols);
				}
			}
		}

		std::vector<float> m_data;
		int    m_numRows;
		int    m_numCols;


		// Operator Overloads
	public:
		inline bool fuzzyEq(const DMatrix& other) const
		{
			if (m_numCols != other.m_numCols || m_numRows != other.m_numRows)
			{
				return false;
			}

			for (size_t i = 0; i < m_data.size(); i++)
			{
				if (std::abs(m_data[i] - other.m_data[i]) > 0.00001f)
				{
					return false;
				}
			}

			return true;
		}

		inline DMatrix operator+ (const DMatrix& other) const
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

		inline DMatrix operator- (const DMatrix& other) const
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

		void operator= (const DMatrix& other)
		{
			Init(other.GetNumRows(), other.GetNumCols(), &other.m_data[0]);
		}

		bool operator== (const DMatrix& other)
		{
			if (m_numCols != other.m_numCols || m_numRows != other.m_numRows)
			{
				return false;
			}

			return m_data == other.m_data;
		}

		inline DMatrix operator* (const DMatrix& other) const
		{
			assert(m_numCols == other.m_numRows);
			DMatrix output(m_numRows, other.m_numCols);
			for (int i = 0; i < m_numRows; i++)
			{
				for (int k = 0; k < m_numCols; k++)
				{
					float a_ik = Get(i, k);
					if (a_ik)
					{
						for (int j = 0; j < other.m_numCols; j++)
						{
							assert(k != 0 || output.Get(i, j) == 0);
							float b_kj = other.Get(k, j);
							if (b_kj)
							{
								output.Set(i, j) += a_ik * b_kj;
							}
						}
					}
				}
			}

			return output;
		};



		inline DMatrix operator* (const float x) const
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

		// Generates a Vector with only the Diagonal Components of the
		// Multiplication
		inline DMatrix diagonalProduct(const DMatrix& other) const
		{
			assert(m_numCols == other.m_numRows); // Multiplicable
			assert(m_numRows == other.m_numCols); // Results in Square Matrix
			DMatrix output(m_numRows, 1);
			for (int i = 0; i < m_numRows; i++)
			{
				for (int k = 0; k < m_numCols; k++)
				{
					float a_ik = Get(i, k);
					if (a_ik)
					{
						assert(k != 0 || output.Get(i) == 0);
						float b_ki = other.Get(k, i);
						if (b_ki)
						{
							output.Set(i) += a_ik * b_ki;
						}
					}
				}
			}

			return output;
		}

		// Generates a single row of a product result for Matrix Multiplication.
		// Result dimensions are 1xN
		inline DMatrix rowProduct(const DMatrix& other, int resultRow) const
		{
			assert(m_numCols == other.m_numRows); // Multiplicable
			DMatrix output(1, other.m_numCols);
			int i = resultRow;
			for (int k = 0; k < m_numCols; k++)
			{
				float a_ik = Get(i, k);
				if (a_ik)
				{
					for (int j = 0; j < other.m_numCols; j++)
					{
						assert(k != 0 || output.Get(0, j) == 0);
						float b_kj = other.Get(k, j);
						if (b_kj)
						{
							output.Set(0, j) += a_ik * b_kj;
						}
					}
				}
			}

			return output;
		}

		// Generates a single column of a product result for Matrilx Multiplication
		// Result dimensions are Nx1
		inline DMatrix colProduct(const float otherValue, int resultCol) const
		{
			DMatrix output(m_numRows, 1);
			for (int i = 0; i < m_numRows; i++)
			{
				int k = resultCol;
				float a_ik = Get(i, k);
				if (a_ik)
				{
					float b_kj = otherValue;
					if (b_kj)
					{
						output.Set(i, 0) += a_ik * b_kj;
					}
				}
			}

			return output;
		}

		inline DMatrix colProduct(const DMatrix& other, int resultCol) const
		{
			assert(m_numCols == other.m_numRows); // Multiplicable
			DMatrix output(m_numRows, other.m_numCols);
			for (int i = 0; i < m_numRows; i++)
			{
				int k = resultCol;
				float a_ik = Get(i, k);
				if (a_ik)
				{
					for (int j = 0; j < other.m_numCols; j++)
					{
						assert(output.Get(i, j) == 0);
						float b_kj = other.Get(k, j);
						if (b_kj)
						{
							output.Set(i, j) += a_ik * b_kj;
						}
					}
				}
			}

			return output;
		}
	};

	inline DMatrix operator* (const float x, const DMatrix& matrix)
	{
		assert(matrix.GetNumCols() >= 0);
		assert(matrix.GetNumRows() >= 0);
		DMatrix output(matrix.GetNumRows(), matrix.GetNumCols());
		for (int i = 0; i < matrix.GetNumRows(); i++)
		{
			for (int j = 0; j < matrix.GetNumCols(); j++)
			{
				output.Set(i, j) = matrix.Get(i, j) * x;
			}
		}

		return output;
	};

};
#endif