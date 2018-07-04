#!/bin/bash

# Generate the C header file derived from a Java code.
javac -h . JPrimitives.java

# Get the location of directories under which the JNI C header located somewhere
# down the tree. This is needed by the generated C header.
export JAVA_HOME=/usr/lib/jvm/default-java

# Create a shared library using the generated header and the man-made
# implementation file
g++ -fPIC -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" -shared -o libjprimitives.so JPrimitives.cpp

# Run the java application with JNI referncing C++ library.
java -Djava.library.path=. JPrimitives

# Repeate the same thing with JString.java
javac -h . JString.java
g++ -fPIC -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" -shared -o libjstring.so JString.cpp
java -Djava.library.path=. JString
