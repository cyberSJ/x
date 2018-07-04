#include "my_jni_HelloJni.h"
#include <iostream>

JNIEXPORT void JNICALL Java_my_1jni_HelloJni_sayHello (JNIEnv *, jobject)
{
    std::cout << "hello world" << std::endl;
}
