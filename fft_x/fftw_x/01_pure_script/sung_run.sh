#!/bin/bash

# Specify the directory that will contain the built artifacts
build_dir="./build"

if [[ ! -d ${build_dir} ]]; then
    echo creating ${build_dir}
    mkdir ${build_dir}
fi

# Set the Java home needed to reference the JNI capability. The jni.h file is
# located under this directory tree. The file is needed to compile code that
# uses JNI.
java_home="/usr/lib/jvm/default-java"

# Assumed location of the fftw.h header file from the downloaded FFTW package.
fftw_include_dir="/usr/local/include"

# Assumed location of the FFTW static libraries from the FFTW package.
fftw_library_dir="/usr/local/lib"

# Assume location for the unzipped JFFTW files.
jfftw_dir="/home/cyber/sandbag/jfftw-1.2"

# List the C implementation of the JFFTW, which is interacting agent between the
# Java part of JFFTW and the C FFTW code. These C code are needed to generate
# the shared library described below.
jfftw_c_source_dir="${jfftw_dir}/c"
jfftw_c_sources="${jfftw_c_source_dir}/jfftw_Wisdom.c \
                 ${jfftw_c_source_dir}/jfftw_complex_Plan.c \
                 ${jfftw_c_source_dir}/jfftw_complex_nd_Plan.c \
                 ${jfftw_c_source_dir}/jfftw_real_Plan.c \
                 ${jfftw_c_source_dir}/jfftw_real_nd_Plan.c"

# The following headers are needed to compile a shared library required for the
# JFFTW java JNI to interface with C FFTW:
# 1. The general JNI header provided from Java.
# 2. The linux JNI header provided from Java.
# 3. The FFTW headers for C.
# 4. The JFFTW headers for C.
include_dirs="-I\"${java_home}/include\" \
              -I\"${java_home}/include/linux\" \
              -I\"${fftw_include_dir}\" \
              -I\"${jfftw_c_source_dir}\""

# This library name is assumed by the JFFTW Java code to load during compile and
# run time. This will be the name of the generated shared library.
jfftw_library_name="libjfftw.so"

# Create the shared library for the JNI C++ implementation that bridges the
# JFFTW (java) with the FFTW (C++). This library will be used by the JFFTW Java
# code. No need to loadLibrary() from my code since the JFFTW Java code already
# does that.
# The `eval echo ${include_dirs}` is needed becuase ${include_dirs} itself
# contains another ${} things to expand. eval does the expansion.
# the "-Wl,--whole-archive ..." line is for linking a static library to the
# soon-to-be-generated dynamic library. This is needed because the Java code
# only has JNI API for the JFFTW shared library, but the library needs to
# include the FFTW static library. Assumes that the FFTW was configured with the
# -fPIC CFLAGS option as well since the statlic library is being attached to a
# shared library.
gcc -fPIC `eval echo ${include_dirs}` \
    -L"${fftw_library_dir}" -lfftw -lrfftw\
    -shared -o "${build_dir}/${jfftw_library_name}" \
    -Wl,--whole-archive "${fftw_library_dir}/libfftw.a" \
        "${fftw_library_dir}/librfftw.a" -Wl,--no-whole-archive \
    ${jfftw_c_sources}

# Create .class files from the downloaded JFFTW Java code and my code that
# references those JFFTW Java code.
jfftw_java_source_dir="${jfftw_dir}/java/jfftw"
jfftw_java_sources="`find ${jfftw_java_source_dir} -name \"*.java\"`"
my_java_sources="JavaFftw.java"
echo ${jfftw_java_sources}
javac -d ${build_dir} -Xlint:deprecation ${jfftw_java_sources} \
    ${my_java_sources}

# Run my code.
java -Djava.library.path="${build_dir}" -classpath ${build_dir} JavaFftw
