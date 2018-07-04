#include "HelloJni.h"
#include <iostream>

JNIEXPORT void JNICALL Java_HelloJni_sayHello(JNIEnv *, jobject)
{
	std::cout << "Hello JNI" << std::endl;
}





