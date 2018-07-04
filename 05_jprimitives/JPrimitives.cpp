#include "JPrimitives.h"

JNIEXPORT jdouble JNICALL Java_JPrimitives_average
    (JNIEnv *, jobject, jint n1, jint n2)
{
    return jdouble(n1 + n2) / jdouble(2);
}
