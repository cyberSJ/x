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
CMAKE_SOURCE_DIR = /home/sung/Documents/practice/logSystemX

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sung/Documents/practice/logSystemX

# Include any dependencies generated for this target.
include CMakeFiles/logSystemX.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/logSystemX.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/logSystemX.dir/flags.make

CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o: CMakeFiles/logSystemX.dir/flags.make
CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o: boostTestXMain.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sung/Documents/practice/logSystemX/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o -c /home/sung/Documents/practice/logSystemX/boostTestXMain.cpp

CMakeFiles/logSystemX.dir/boostTestXMain.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/logSystemX.dir/boostTestXMain.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sung/Documents/practice/logSystemX/boostTestXMain.cpp > CMakeFiles/logSystemX.dir/boostTestXMain.cpp.i

CMakeFiles/logSystemX.dir/boostTestXMain.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/logSystemX.dir/boostTestXMain.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sung/Documents/practice/logSystemX/boostTestXMain.cpp -o CMakeFiles/logSystemX.dir/boostTestXMain.cpp.s

CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o.requires:
.PHONY : CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o.requires

CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o.provides: CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o.requires
	$(MAKE) -f CMakeFiles/logSystemX.dir/build.make CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o.provides.build
.PHONY : CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o.provides

CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o.provides.build: CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o

CMakeFiles/logSystemX.dir/logSystemX.cpp.o: CMakeFiles/logSystemX.dir/flags.make
CMakeFiles/logSystemX.dir/logSystemX.cpp.o: logSystemX.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sung/Documents/practice/logSystemX/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/logSystemX.dir/logSystemX.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/logSystemX.dir/logSystemX.cpp.o -c /home/sung/Documents/practice/logSystemX/logSystemX.cpp

CMakeFiles/logSystemX.dir/logSystemX.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/logSystemX.dir/logSystemX.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sung/Documents/practice/logSystemX/logSystemX.cpp > CMakeFiles/logSystemX.dir/logSystemX.cpp.i

CMakeFiles/logSystemX.dir/logSystemX.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/logSystemX.dir/logSystemX.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sung/Documents/practice/logSystemX/logSystemX.cpp -o CMakeFiles/logSystemX.dir/logSystemX.cpp.s

CMakeFiles/logSystemX.dir/logSystemX.cpp.o.requires:
.PHONY : CMakeFiles/logSystemX.dir/logSystemX.cpp.o.requires

CMakeFiles/logSystemX.dir/logSystemX.cpp.o.provides: CMakeFiles/logSystemX.dir/logSystemX.cpp.o.requires
	$(MAKE) -f CMakeFiles/logSystemX.dir/build.make CMakeFiles/logSystemX.dir/logSystemX.cpp.o.provides.build
.PHONY : CMakeFiles/logSystemX.dir/logSystemX.cpp.o.provides

CMakeFiles/logSystemX.dir/logSystemX.cpp.o.provides.build: CMakeFiles/logSystemX.dir/logSystemX.cpp.o

# Object files for target logSystemX
logSystemX_OBJECTS = \
"CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o" \
"CMakeFiles/logSystemX.dir/logSystemX.cpp.o"

# External object files for target logSystemX
logSystemX_EXTERNAL_OBJECTS =

logSystemX: CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o
logSystemX: CMakeFiles/logSystemX.dir/logSystemX.cpp.o
logSystemX: /usr/local/lib/libboost_log.so
logSystemX: /usr/local/lib/libboost_log_setup.so
logSystemX: /usr/lib/libboost_thread-mt.so
logSystemX: /usr/lib/libboost_system-mt.so
logSystemX: /usr/lib/libboost_unit_test_framework-mt.so
logSystemX: CMakeFiles/logSystemX.dir/build.make
logSystemX: CMakeFiles/logSystemX.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable logSystemX"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/logSystemX.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/logSystemX.dir/build: logSystemX
.PHONY : CMakeFiles/logSystemX.dir/build

CMakeFiles/logSystemX.dir/requires: CMakeFiles/logSystemX.dir/boostTestXMain.cpp.o.requires
CMakeFiles/logSystemX.dir/requires: CMakeFiles/logSystemX.dir/logSystemX.cpp.o.requires
.PHONY : CMakeFiles/logSystemX.dir/requires

CMakeFiles/logSystemX.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/logSystemX.dir/cmake_clean.cmake
.PHONY : CMakeFiles/logSystemX.dir/clean

CMakeFiles/logSystemX.dir/depend:
	cd /home/sung/Documents/practice/logSystemX && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sung/Documents/practice/logSystemX /home/sung/Documents/practice/logSystemX /home/sung/Documents/practice/logSystemX /home/sung/Documents/practice/logSystemX /home/sung/Documents/practice/logSystemX/CMakeFiles/logSystemX.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/logSystemX.dir/depend

