#include <cstdlib>
class MyClass
{
    public:
        void* operator new(size_t size);
        void operator delete(void*);
        virtual void print();
        long m_int; 
        int m_int2; 
};

class MySubClass : public MyClass
{
    public:
        MySubClass(){}
        virtual void print();
};
