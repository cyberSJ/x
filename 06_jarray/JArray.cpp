#include "JArray.h"
#include <iostream>


JNIEXPORT jdoubleArray JNICALL Java_JArray_sumAndAverage
  (JNIEnv *env, jobject, jintArray javaIntArray)
{
    jint* cIntArray = env->GetIntArrayElements(javaIntArray, NULL);
    jsize length = env->GetArrayLength(javaIntArray);
    std::cout << "int array length: " << length << std::endl;

    jint sum = 0;
    for (unsigned int i = 0; i < length; ++i)
    {
        sum += cIntArray[i];
    }

    jdouble average = (jdouble)sum / (jdouble)length;

    jdouble cDoubleArray[] = {sum, average};

    jdoubleArray jDoubleArray = env->NewDoubleArray(2);

    if (jDoubleArray == NULL)
    {
        return NULL;
    }

    // Copy from C world to Java world.
    env->SetDoubleArrayRegion(jDoubleArray, 0, 2, cDoubleArray);
    return jDoubleArray;
}
