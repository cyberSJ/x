#include <iostream>
#include "MyClass.h"

void* MyClass::operator new(size_t size)
{
    void* storage = malloc(size);
    if (NULL == storage)
    {
        throw "allocation fail: no free memory";
    }
    std::cout << "Allocated size: " << size << std::endl;
    return storage;
}

void MyClass::operator delete(void* pointer)
{
    std::cout << "delete called" << std::endl;
    if (NULL == pointer)
    {
        return;
    }

    free(pointer);
}

void MyClass::print()
{
    std::cout << "m_int: " << m_int << std::endl;
}
