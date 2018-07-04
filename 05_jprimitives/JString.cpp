#include "JString.h"
#include <cstdio>
#include <iostream>
#include <string>


JNIEXPORT jstring JNICALL Java_JString_say
  (JNIEnv *env, jobject, jstring things_to_say)
{
    // I think this is newer, more convenient version of the function
    // GetStringUTFChars(), because you don't have to do the clean up process
    // after getting the chars.
    std::string input(
        env->GetStringUTFChars(things_to_say, 
                               NULL));

    std::cout << "sung got: " << input << std::endl;

    // Prompt user for a string
    char from_user[128];
    std::cout << "Enter a string: ";
    scanf("%s", from_user);

    return env->NewStringUTF(from_user);
}
