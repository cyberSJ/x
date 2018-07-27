.PHONY: all
all: HelloJni.java

build_dir=./build
jfftw_c_source_path=/home/cyber/sandbag/jfftw-1.2/c
jfftw_java_source_path=/home/cyber/sandbag/jfftw-1.2/java/jfftw
jfftw_java_sources=$(shell find ${jfftw_java_source_path} -name "*.java")
fftw_include_path=-I/usr/local/include
jni_include_path=-I/usr/lib/jvm/default-java/include \
                 -I/usr/lib/jvm/default-java/include/linux
fftw_lib_path=/usr/local/lib
jfftw_objects=jfftw_Wisdom.o \
			  jfftw_complex_Plan.o \
              jfftw_complex_nd_Plan.o \
              jfftw_real_Plan.o \
              jfftw_real_nd_Plan.o
flags_link_static_lib=-Wl,--whole-archive \
	   				  ${fftw_lib_path}/libfftw.a ${fftw_lib_path}/librfftw.a \
	   				  -Wl,--no-whole-archive
CFLAGS=-fPIC 
LDFLAGS=-L${fftw_lib_path}
LDLIBS=-lfftw -lrfftw
VPATH=${jfftw_c_source_path}:${build_dir}

# Create the build directory.
${build_dir}: 
	mkdir ${build_dir}

# Add flag to generate the dependecy files .d while doing the compilation at the
# same time.
CFLAGS=-MMD -MP

# User order-only prerequisite for create the build directory only if the
# directory does not exist. The build directory's timestamp is ignored for 
# order-only prerequisites.
# The notdir is because $@ = %.o includes the build_dir if build_dir already
# exists, and does not include it if build_dir does not exist.
%.o: %.c | ${build_dir}
	$(CC) $(CPPFLAGS) $(CFLAGS) ${fftw_include_path} ${jni_include_path} \
		-c -o ${build_dir}/$(notdir $@) $<

# Assuming the dependency flag files are generated (caused by the CFLAGS above),
# declare .d as targets so that if the files in the .d dependency lists change,
# the objects listed in the .d file can be rebuilt.
dep_files=$(wildcard ${build_dir}/*.d)
include ${dep_files}

# Generate the libjfftw.so library with the libfftw.a and librfftw.a static
# libraries from FFTW. The creation of libjfftw.so requires 
# 1. The JFFTW c code
# 2. Headers from FFTW code 
# 3. The static libraries libfftw.a and librfftw.a 
.PHONY: libjfftw.so
${build_dir}/libjfftw.so: ${jfftw_objects}
	$(CC) $(LDFLAGS) $(LDLIBS) $(addprefix ${build_dir}/, ${jfftw_objects}) \
		${flags_link_static_lib} --shared -o $@

# Compile my custom java code that uses JFFTW.
.PHONY: HelloJni.java
HelloJni.java: ${build_dir}/libjfftw.so
	javac -Xlint:deprecation -d ${build_dir} ${jfftw_java_sources} $@

# Run my custom java code that uses JFFTW.
.PHONY: test
test:
	java -classpath ${build_dir} -Djava.library.path=${build_dir} HelloJni

.PHONY: clean
clean:
	rm -rf ${build_dir}
