#!/bin/bash

build_dir="./build"

if [[ ! -d ${build_dir} ]]; then
    echo "Creating ${build_dir}"
    mkdir ${build_dir}
fi

# Create the libjfftw.so shared library using CMake.
cd ${build_dir}
cmake ../
make
cd ..

# Compile my custom java source into a .class file.
jfftw_java_source_dir="/home/cyber/sandbag/jfftw-1.2/java/jfftw"
jfftw_java_sources="`find ${jfftw_java_source_dir} -name \"*.java\"`"
javac -d ${build_dir} -Xlint:deprecation ${jfftw_java_sources} HelloJni.java

# Run the .class file
java -Djava.library.path=${build_dir} -classpath ${build_dir} HelloJni
