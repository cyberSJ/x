#!/bin/bash

# Generate the header file that contains JNI API.
javac -h . JObject.java

# Export the location related to the java jni header.
export JAVA_HOME=/usr/lib/jvm/default-java

# Generate a shared library
g++ -fPIC -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" -shared -o libjobject.so JObject.cpp

# Run the Java code using JNI.
#java -Djava.library.path=. JObject

