# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sung/Documents/practice/zmqX

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sung/Documents/practice/zmqX

# Include any dependencies generated for this target.
include CMakeFiles/zmqHelloWorldServerX.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/zmqHelloWorldServerX.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/zmqHelloWorldServerX.dir/flags.make

CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o: CMakeFiles/zmqHelloWorldServerX.dir/flags.make
CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o: zmqHelloWorldServerX.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sung/Documents/practice/zmqX/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o -c /home/sung/Documents/practice/zmqX/zmqHelloWorldServerX.cpp

CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sung/Documents/practice/zmqX/zmqHelloWorldServerX.cpp > CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.i

CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sung/Documents/practice/zmqX/zmqHelloWorldServerX.cpp -o CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.s

CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o.requires:
.PHONY : CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o.requires

CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o.provides: CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o.requires
	$(MAKE) -f CMakeFiles/zmqHelloWorldServerX.dir/build.make CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o.provides.build
.PHONY : CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o.provides

CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o.provides.build: CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o

# Object files for target zmqHelloWorldServerX
zmqHelloWorldServerX_OBJECTS = \
"CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o"

# External object files for target zmqHelloWorldServerX
zmqHelloWorldServerX_EXTERNAL_OBJECTS =

zmqHelloWorldServerX: CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o
zmqHelloWorldServerX: CMakeFiles/zmqHelloWorldServerX.dir/build.make
zmqHelloWorldServerX: CMakeFiles/zmqHelloWorldServerX.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable zmqHelloWorldServerX"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/zmqHelloWorldServerX.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/zmqHelloWorldServerX.dir/build: zmqHelloWorldServerX
.PHONY : CMakeFiles/zmqHelloWorldServerX.dir/build

CMakeFiles/zmqHelloWorldServerX.dir/requires: CMakeFiles/zmqHelloWorldServerX.dir/zmqHelloWorldServerX.cpp.o.requires
.PHONY : CMakeFiles/zmqHelloWorldServerX.dir/requires

CMakeFiles/zmqHelloWorldServerX.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/zmqHelloWorldServerX.dir/cmake_clean.cmake
.PHONY : CMakeFiles/zmqHelloWorldServerX.dir/clean

CMakeFiles/zmqHelloWorldServerX.dir/depend:
	cd /home/sung/Documents/practice/zmqX && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sung/Documents/practice/zmqX /home/sung/Documents/practice/zmqX /home/sung/Documents/practice/zmqX /home/sung/Documents/practice/zmqX /home/sung/Documents/practice/zmqX/CMakeFiles/zmqHelloWorldServerX.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/zmqHelloWorldServerX.dir/depend

