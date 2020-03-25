#include "MyClass.h"

int main(int argc, char** argv)
{
    MyClass* myClass = (MyClass*) new MyClass;
    //delete myClass;
    myClass = new MyClass;
}
