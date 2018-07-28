cmake_minimum_required(VERSION 3.10)

project(jfftw_cmake)

# These are the required JFFTW C source files for generating libjfftw.so
# These contains the C side of JFFT JNI that talks with the FFTW C code.
set(jfftw_c_source_dir /home/cyber/sandbag/jfftw-1.2/c)
set(jfftw_c_sources ${jfftw_c_source_dir}/jfftw_Wisdom.c
                    ${jfftw_c_source_dir}/jfftw_complex_Plan.c
                    ${jfftw_c_source_dir}/jfftw_complex_nd_Plan.c
                    ${jfftw_c_source_dir}/jfftw_real_Plan.c
                    ${jfftw_c_source_dir}/jfftw_real_nd_Plan.c)

# Let CMake find the headers of FFTW C code.
set(fftw_header_dir /usr/local/include)
set(jni_header_dir /usr/lib/jvm/default-java/include
                    /usr/lib/jvm/default-java/include/linux)
include_directories(${fftw_header_dir}
                    ${jni_header_dir})

get_property(my_include_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${my_include_dirs})
    message(STATUS "dir = '${dir}'")
endforeach()

# Let Cmake find the library of FFTW C code.
set(fftw_lib_dir /usr/local/lib)
link_directories(${fftw_lib_dir})

# Order CMake to create the libjfftw.so later.
set(jfftw_lib_name jfftw)
add_library(${jfftw_lib_name} SHARED ${jfftw_c_sources})

# Link the static JFFTW library to libjfftw.so. Somehow this links the static
# library to the dynamic library without my manual addtion of
# "-Wl,--whole-archive" link flag:
#set_target_properties(${jfftw_lib_name} 
#                       PROPERTIES LINK_FLAGS "-Wl,--whole-archive")
target_link_libraries(${jfftw_lib_name} 
                      ${fftw_lib_dir}/libfftw.a
                      ${fftw_lib_dir}/librfftw.a)
