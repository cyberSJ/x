#ifndef HELLO_JNI_IMPL_H
#define HELLO_JNI_IMPL_H

#include "HelloJni.h"

// This header declares a function to say hello. This function will be
// implemented in a C++ code, but the declaration will be utilized by a C code.
// This is just to demonstrate that a C code can call C++ function, and not
// really related to the Java Native Interface.

// Do not perform name mangling if this header is included in a C code. Not
// really needed for this simple application where the function is not
// associated with any C++ class, but it is a good practice since this header
// file is intended to be included in a C and C++ code.
#ifdef __cplusplus
extern "C"
{
#endif

    // Declares the function to say hello.
    void sayHello();

#ifdef __cplusplus
}
#endif

#endif
