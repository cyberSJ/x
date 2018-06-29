/**
 * Steps to compile this file:
 * 1. Export the JAVA_HOME
 *     export JAVA_HOME=/usr/lib/jvm/default-java/
 * 2. Compile this c file.
 *     g++ -fPIC -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" -shared -o libhello.so HelloJni.cpp
 *       - where -fPIC means Place Independent Code, which means the generated
 *         library is independent of which address its assembly code was loaded by
 *         another code.
 *       - Includes both the include and include/linux dir.
 *       - -shared means the gcc code will generated a shared library (as oppsed
 *         to a static library.
 */
#include "HelloJni.h"
#include <iostream>

JNIEXPORT void JNICALL Java_HelloJni_sayHello(JNIEnv* env,
                                              jobject thisObj)
{
    std::cout << "Hello world from C++" << std::endl;
    return;
}
