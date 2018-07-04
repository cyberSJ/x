#!/bin/bash

# Generate the C header from the Java code using JNI.
javac -h . JArray.java

# Export the path related to the jni.h directory needed for producing a shared
# library.
export JAVA_HOME=/usr/lib/jvm/default-java

# Create a shared library using the generated JNI C header.
g++ -fPIC  -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" -shared -o libjarray.so JArray.cpp

# Run the java code that uses JNI.
java -Djava.library.path=. JArray
