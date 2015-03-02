#ifndef dsMatrix_H
#define dsMatrix_H

#include <vector>

/*
	matrix is a template class.
*/
template<typename Object>
class dsMatrix{

	public:
		/*
			Constructor. Takes rows and cols.
			Has initializer list that constructs array,
			which is a vector of (vector of object).
			Then it iterates through each row of matrix
			to resize that row with cols number.
		*/
		dsMatrix(int rows, int cols):m_array(rows){
			for (int i = 0; i < rows; i++){
				m_array[i].resize(cols);
			}
		}

		/*
			operator[] overloading.
			this operator will later take one argument: row index.
			it returns vector of obejct located at that index.		
			This is the accessor versio of the operator[]
		*/
		const std::vector<Object> & operator[](int row) const{
			return m_array[row];
		}

		/*
			operator[] mutator overloading.
			returns non-const reference to a vector of object,
			because that reference will be used to change the 
			reference vector of object.
		*/
		std::vector<Object> & operator[](int row){
			return m_array[row];
		}

		int numRows() const{
			return m_array.size();
		}
		
		int numCols() const{
			return (m_array.size() ? m_array[0].size() : 0);
		}

	private:
		std::vector< std::vector<Object> > m_array;

};

#endif
