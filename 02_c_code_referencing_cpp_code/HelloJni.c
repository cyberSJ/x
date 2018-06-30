#include "HelloJniImpl.h"

// Include the implementation. Otherwise, we would have to create a C++ library
// for the sayHello function and link it when we compile this C code.
#include "HelloJniImpl.cpp"

/**
 * This c program will reference a header file that contains yet another
 * implementation of saying hello. That implementation will be defined in cpp
 * file.
 */

// Implement a Java Native Interface function so that Java code can find the
// definition of the function declared in the shared library.
JNIEXPORT void JNICALL Java_HelloJni_sayHello(JNIEnv* env, jobject thisObject)
{
    sayHello();
}
