#include "dsMatrix.h"
#include <iostream>

int main(int argc, char** argv){

	dsMatrix<int> myMatrix(20,30);
	std::cout << "The matrix size: " << myMatrix.numRows() << " by " << myMatrix.numCols() << std::endl;

	myMatrix[5][3] = 40;

	myMatrix[1][2] = myMatrix[5][3];

	std::cout << "myMatrix[5][3] = " << myMatrix[5][3] << std::endl;
	std::cout << "myMatrix[1][2] = " << myMatrix[1][2] << std::endl;
	

	

	return 0;
}
