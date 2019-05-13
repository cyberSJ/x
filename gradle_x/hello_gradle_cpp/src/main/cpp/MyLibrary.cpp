#include "MyLibrary.h"
#include <iostream>

MyLibrary::MyLibrary()
{
    std::cout << "MyLibrary ctor" << std::endl;
}

void MyLibrary::sayHello()
{
    std::cout << "MyLibrary saying hello" << std::endl;
}
