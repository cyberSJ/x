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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sung/Documents/practice/cmake_x

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sung/Documents/practice/cmake_x/build

# Include any dependencies generated for this target.
include todo/CMakeFiles/todoLibrary.dir/depend.make

# Include the progress variables for this target.
include todo/CMakeFiles/todoLibrary.dir/progress.make

# Include the compile flags for this target's objects.
include todo/CMakeFiles/todoLibrary.dir/flags.make

todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o: todo/CMakeFiles/todoLibrary.dir/flags.make
todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o: ../todo/Todo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sung/Documents/practice/cmake_x/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o"
	cd /home/sung/Documents/practice/cmake_x/build/todo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/todoLibrary.dir/Todo.cpp.o -c /home/sung/Documents/practice/cmake_x/todo/Todo.cpp

todo/CMakeFiles/todoLibrary.dir/Todo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/todoLibrary.dir/Todo.cpp.i"
	cd /home/sung/Documents/practice/cmake_x/build/todo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sung/Documents/practice/cmake_x/todo/Todo.cpp > CMakeFiles/todoLibrary.dir/Todo.cpp.i

todo/CMakeFiles/todoLibrary.dir/Todo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/todoLibrary.dir/Todo.cpp.s"
	cd /home/sung/Documents/practice/cmake_x/build/todo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sung/Documents/practice/cmake_x/todo/Todo.cpp -o CMakeFiles/todoLibrary.dir/Todo.cpp.s

todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o.requires:
.PHONY : todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o.requires

todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o.provides: todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o.requires
	$(MAKE) -f todo/CMakeFiles/todoLibrary.dir/build.make todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o.provides.build
.PHONY : todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o.provides

todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o.provides.build: todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o

# Object files for target todoLibrary
todoLibrary_OBJECTS = \
"CMakeFiles/todoLibrary.dir/Todo.cpp.o"

# External object files for target todoLibrary
todoLibrary_EXTERNAL_OBJECTS =

todo/libtodoLibrary.so: todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o
todo/libtodoLibrary.so: todo/CMakeFiles/todoLibrary.dir/build.make
todo/libtodoLibrary.so: todo/CMakeFiles/todoLibrary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libtodoLibrary.so"
	cd /home/sung/Documents/practice/cmake_x/build/todo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/todoLibrary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
todo/CMakeFiles/todoLibrary.dir/build: todo/libtodoLibrary.so
.PHONY : todo/CMakeFiles/todoLibrary.dir/build

todo/CMakeFiles/todoLibrary.dir/requires: todo/CMakeFiles/todoLibrary.dir/Todo.cpp.o.requires
.PHONY : todo/CMakeFiles/todoLibrary.dir/requires

todo/CMakeFiles/todoLibrary.dir/clean:
	cd /home/sung/Documents/practice/cmake_x/build/todo && $(CMAKE_COMMAND) -P CMakeFiles/todoLibrary.dir/cmake_clean.cmake
.PHONY : todo/CMakeFiles/todoLibrary.dir/clean

todo/CMakeFiles/todoLibrary.dir/depend:
	cd /home/sung/Documents/practice/cmake_x/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sung/Documents/practice/cmake_x /home/sung/Documents/practice/cmake_x/todo /home/sung/Documents/practice/cmake_x/build /home/sung/Documents/practice/cmake_x/build/todo /home/sung/Documents/practice/cmake_x/build/todo/CMakeFiles/todoLibrary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : todo/CMakeFiles/todoLibrary.dir/depend

