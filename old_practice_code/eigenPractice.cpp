
#include <iostream>
#include <Eigen/Dense>

// using Eigen::MatrixXd;

int main()
{
// m(0,0) = 3;
// m(1,0) = 2.5;
// m(0,1) = -1;
// m(1,1) = m(1,0) + m(0,1);
// std::cout << m << std::endl;

  Eigen::Matrix2d myMatrix;
  myMatrix(0,0) = 1;
  myMatrix(0,1) = 2;
  myMatrix(1,0) = 3;
  myMatrix(1,1) = 4;
  Eigen::Matrix2d myMatrix2;
  myMatrix2 = myMatrix.inverse();

  std::cout << "myMatrix:\n";
  std::cout << myMatrix << std::endl;
  std::cout << "myMatrix2:\n" << myMatrix2 << std::endl;

}
