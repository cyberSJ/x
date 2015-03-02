#include <iostream>
#include <vector>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "invert_matrix.hpp"

int main(int argc, char** argv)
{
	// Simple matrix creation X:
	//------------------------------
	boost::numeric::ublas::matrix<double> m(3,3);
	for (unsigned int i = 0; i < m.size1(); i++)
	{
		for (unsigned int j = 0; j < m.size2(); ++j)
		{
			m(i,j) = 3*i + j;
		}
	}

	std::cout << m << std::endl;


	// Insert an entire row to a matrix:
	//----------------------------------------
	boost::numeric::ublas::matrix<double> mat(3,2);
	boost::numeric::ublas::vector<double> vec(2);
	for (unsigned int row = 0; row < mat.size1(); ++row)
	{
		boost::numeric::ublas::matrix_row<
		boost::numeric::ublas::matrix<double> > mr(mat,row);

		vec[0] = 0.1;
		vec[1] = 4.3;

		mr = vec;
	}
	std::cout << mat << std::endl;


	// Creating diagonal matrix:
	//------------------------------
	boost::numeric::ublas::identity_matrix<double> im(3);
	boost::numeric::ublas::matrix<double> dm = 3*im;
	std::cout << dm << std::endl;

	// Operatoins:
	//---------------
	double initialValues[3][3] = {
		1, 2, 3,
		5, 1, 4,
		6, 7, 1
	};
	std::cout << boost::numeric::ublas::trans(mat) << std::endl;
	std::cout << boost::numeric::ublas::prod(m, m) << std::endl;


	// Inverse (not built-in boost):
	//------------------------------------
	boost::numeric::ublas::matrix<double> input(3,3), inverted(3,3);
	input(0,0) = 1; // not every matrix is invertible
	input(0,1) = 2;
	input(0,2) = 3;
	input(1,0) = 5;
	input(1,1) = 1;
	input(1,2) = 4;
	input(2,0) = 6;
	input(2,1) = 7;
	input(2,2) = 1;
	InvertMatrix(input, inverted);
	std::cout << input << std::endl;
	std::cout << inverted << std::endl;
	return 0;
}
