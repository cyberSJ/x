#!/bin/bash

# Set JAVA_HOME environment variable so that the jni.h file can be found
export JAVA_HOME=/usr/lib/jvm/default-java

# Generate the JNI header file.
javac -h . HelloJni.java

# Create a shared library. The JNI implementation is in HelloJni.c, but that
# code uses another custom function declared in a C++ code. That custom function
# is declared in HelloJniImpl.h, and the implementation of it is in 
# HelloJniImpl.cpp file. The C code references both the custom .h and .cpp files
# so we do not have to link a library when compiling the C code.
# Since the C code references a C++ code, we need to use g++ instead of gcc.
# -fPIC: The generated shared library is independent of which address it is
#    loaded to.
g++ -fPIC -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" --shared -o libhello.so HelloJni.c

# Run the java program that uses JNI to call C function
# -Djava.library.path=.: The shared library that JNI needs is in current
#     directory.
java -Djava.library.path=. HelloJni
