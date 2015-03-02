#include <iostream>
#include <matio.h>
#include <boost/multi_array.hpp>
#include <fstream>

using namespace std;

int main(int argc, char** argv){
  cout << "Hello World! "<< MAT_ACC_RDONLY  << endl;

  int err = 0;
  const char *fileName = "vals1.mat";
  mat_t *mat = Mat_Open(fileName, MAT_ACC_RDONLY);
  matvar_t *matvar; 
 
  if (mat){
    matvar = Mat_VarRead(mat, (char*)"vals"); // The name of the variable is "vals"
    if (matvar == NULL){
      err = 1;
    } else{ // Main part that manages the matrix
      // Get the size (number of elements) of the matrix
      unsigned xSize = matvar->nbytes/matvar->data_size;
      
      // Obtain a pointer to the data
      const double *xData = static_cast<const double*>(matvar->data);
      // cout << "x data = " << xData[1] << endl;

      // Get the dimension of the matrix
      for (int i=0; i<matvar->rank; i++){
        cout << "dim[" << i << "] = " << matvar->dims[i] << endl;
      }
      
      // Use boost::multi_array library to form 2D matrix.
      typedef boost::multi_array<double, 2> array_type;
      typedef array_type::index index;
      array_type A(boost::extents[matvar->dims[0]][matvar->dims[1]]);

      // Fill in the multi_array with the data from the mat file
      for(int col = 0; col < matvar->dims[1]; col++){
  	for(int row = 0; row < matvar->dims[0]; row++){
   	  A[row][col] = *xData; // xData must be accesing the matrix col-major.
          xData++;
	}
      }

      // Now use this familiar matrix format to display the data and save it in the txt file
      ofstream out("val1.txt");
      if(!out){
  	cout << "Couldn't open the file\n";
        return 1;
      }

      int j;
      for(j = 0; j < matvar->dims[1]; j++){
   	for(int i = 0; i < matvar->dims[0]; i++){
	  cout << A[i][j] << " ";
          out << A[i][j] << " ";
	}
	cout << "column: " << j << endl;
        out << "column: " << j << endl;
      }
  
      cout << "Last column printed: " << j << endl;
      out.close();
    

      Mat_VarFree(matvar);
    }
    
    Mat_Close(mat);
  }
  return err;
}
