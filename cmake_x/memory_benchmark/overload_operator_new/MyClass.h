#include <cstdlib>
class MyClass
{
    public:
        void* operator new(size_t size);
        void operator delete(void*);
        void print();
        long m_int; 
        int m_int2; 
};
