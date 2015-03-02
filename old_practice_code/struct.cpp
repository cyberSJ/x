#include <iostream>

typedef struct{
  int var1;
  double var2;
}MyStruct;

int main(int argc, char** argv){
  MyStruct myStruct;
  myStruct.var1 = 1;
  myStruct.var2 = 2;
  std::cout << "My struct: " << myStruct.var1 << " " << myStruct.var2 << std::endl;

  return 0;
}


