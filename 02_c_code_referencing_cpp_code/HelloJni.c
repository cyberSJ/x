#include "HelloJniImpl.h"

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
