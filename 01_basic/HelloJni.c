#include "HelloJni.h"
#include <stdio.h>

/**
 * Steps to compile this file:
 * 1. Export the JAVA_HOME
 *     export JAVA_HOME=/usr/lib/jvm/default-java/
 * 2. Compile this c file.
 *     gcc -fPIC -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" -shared -o libhello.so HelloJni.c
 *       - where -fPIC means Place Independent Code, which means the generated
 *         library is independent of which address its assembly code was loaded by
 *         another code.
 *       - Includes both the include and include/linux dir.
 *       - -shared means the gcc code will generated a shared library (as oppsed
 *         to a static library.
 */

/**
 * JNIEXPORT and JNICALL are some JNI keyword needed to bridge the gap between 
 * java and c++.
 */
JNIEXPORT void JNICALL Java_HelloJni_sayHello(JNIEnv *env,
                                              jobject thisObj)
{
    printf("Hello world\n");
    return;
}
