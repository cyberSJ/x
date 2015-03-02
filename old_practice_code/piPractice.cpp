#include <math.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
  cout << "Pi is : " << M_PI << endl;
  cout << "Pi/180 is: " << M_PI/180 << endl;
  cout << "cos(30deg) is: " << cos(30*M_PI/180) << endl;
  cout << "sin(30deg) is: " << sin(30*M_PI/180) << endl;

  return 0;
}
