#include "JObject.h"
#include <iostream>
#include <string>

JNIEXPORT void JNICALL Java_JObject_modifyInstanceVariable
  (JNIEnv *env, jobject jObject)
{
    jclass thisClass = env->GetObjectClass(jObject);

    /**
     * Gets the field id of the field I want to access.
     *
     * @param thisClass The jclass referencing the jobject.
     * @param "intMember"  The name of the variable in the object you want to
     *     access.
     * @param "I" The type of the variable in the object you want to access..
     */
    jfieldID fieldId = env->GetFieldID(thisClass, "intMember", "I");

    if (fieldId == NULL)
    {
        std::cout << "NULL field ID obtained." << std::endl;
        return;
    }

    /**
     * Get the member variable value using the field id.
     *
     * @param jObject The jObject that owns the member variable. It is
     *     interesting to see that the member variable value is not obtained 
     *     through the C version of the object, but through the jobject.
     * @param fieldId The ID of the field for the member variable that was
     *     obtained through the GetIntField() method.
     */
    jint intMemberValue = env->GetIntField(jObject, fieldId);
    std::cout << "Obtained intMember: " << intMemberValue << std::endl;

    /**
     * Change the variable. Straight-forward.
     */
    env->SetIntField(jObject, fieldId, 99);

    // Get the field ID of the stringMember.
    fieldId = env->GetFieldID(thisClass, "stringMember", "Ljava/lang/String;");
    if (fieldId == NULL)
    {
        std::cout << "Obtained NULL field ID for stringMember" << std::endl;
        return;
    }

    // Obtain the field value.
    //jstring stringMemberObject = env->GetObjectField(env, thisClass, fieldId);


    //// Convert the object to std::string
    //std::string stringMemberValue = env->GetStringUTFChars(stringMemberObject, 
    //                                                       NULL);
    //std::cout << "Obtained stringMember: " << stringMemberValue << std::endl;
}
