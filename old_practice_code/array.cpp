#include <iostream>

using namespace std;

int main(int argc, char** argv){
  int (*a)[10] = new int[3][10]; 

  int *c = new int[11];

  int **b = new int[3];
  for( int i = 0; i < 3; i++){
    b[i] = new int[10];
  }

  delete [] a;

  return 0;

}
