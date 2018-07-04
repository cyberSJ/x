#!/bin/bash

export JAVA_HOME=/usr/lib/jvm/default-java

javac -h . my_jni/HelloJni.java

g++ -fPIC -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" --shared -o libhello.so HelloJni.cpp

java -Djava.library.path=. my_jni/HelloJni
