# Define some variables
CURRENT_DIR=$(shell pwd)
CLASS_PATH=../bin
SRC_PATH=../src
JNI_PATH=../jni
JAVA_HOME=/usr/lib/jvm/default-java

# THIS IS ONLY USEFUL IF .class FILES WERE ALREADY GENEREATED.
# Define a virtual path for .class in the bin directory. 
# vpath is the path to look for dependency header files or any other files we
# need to compile successfully using makefile. It lets makefile know where
# the file is.
# % is the wildcard in makefile.
# So this command remembers all the .class files in the ../bin directory for
# later use.
#vpath %.class $(CLASS_PATH)

# Build the .class file first, and put it in the bin directory
HelloJni.class :
	javac $(SRC_PATH)/HelloJni.java -d $(CLASS_PATH)

# $* matches the target file name without the extension.
# We are going to build HelloJni.h file, which is dependent on HelloJni.class,
# which the Makefile is be able to find, because of the vpath setting.
# Then to generate the output file HelloJni.h, we execute the javah command.
# The makefile knows where to find the HelloJni.class, but the javah or Java
# compiler does not yet know where to find the HelloJni.class file to generate
# the HelloJni.h from. So we need to specify the -classpath for the javah
# command.
# $* is for capturing only the file name (anything before ".") when the make 
# file argument is the "HelloJni.h"
HelloJni.h : HelloJni.class
	javah -classpath $(CLASS_PATH) $*
	
# Create the shared library made possible by combining the generated .h file and
# the man-made .cpp file. Something buggy with the eclipse running the make file
# because this command magically creates a HelloJni.cpp in the bin directory.
libhello.so : HelloJni.h
	g++ -fPIC -I"$(JAVA_HOME)/include" -I"$(JAVA_HOME)/include/linux" -I"$(JNI_PATH)" -shared -o $@ $(SRC_PATH)/HelloJni.cpp
	
# Then go to the bin directory and issue this command to run the Java program
# that uses the shared library. The paths passed to the java command is all 
# relative to the location where the java command is used.
# java -Djava.library.path=../jni HelloJni 
	
clean:
	rm $(CLASS_PATH)/*.class
	rm $(CLASS_PATH)/*.cpp
	rm $(JNI_PATH)/*.so
	rm $(JNI_PATH)/HelloJni.h