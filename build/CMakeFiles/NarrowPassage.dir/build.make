# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/saslab/Projects/NarrowPassage

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/saslab/Projects/NarrowPassage/build

# Include any dependencies generated for this target.
include CMakeFiles/NarrowPassage.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/NarrowPassage.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/NarrowPassage.dir/flags.make

CMakeFiles/NarrowPassage.dir/src/Image.cpp.o: CMakeFiles/NarrowPassage.dir/flags.make
CMakeFiles/NarrowPassage.dir/src/Image.cpp.o: ../src/Image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saslab/Projects/NarrowPassage/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/NarrowPassage.dir/src/Image.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NarrowPassage.dir/src/Image.cpp.o -c /home/saslab/Projects/NarrowPassage/src/Image.cpp

CMakeFiles/NarrowPassage.dir/src/Image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NarrowPassage.dir/src/Image.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saslab/Projects/NarrowPassage/src/Image.cpp > CMakeFiles/NarrowPassage.dir/src/Image.cpp.i

CMakeFiles/NarrowPassage.dir/src/Image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NarrowPassage.dir/src/Image.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saslab/Projects/NarrowPassage/src/Image.cpp -o CMakeFiles/NarrowPassage.dir/src/Image.cpp.s

CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.o: CMakeFiles/NarrowPassage.dir/flags.make
CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.o: ../src/NarrowCpp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saslab/Projects/NarrowPassage/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.o -c /home/saslab/Projects/NarrowPassage/src/NarrowCpp.cpp

CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saslab/Projects/NarrowPassage/src/NarrowCpp.cpp > CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.i

CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saslab/Projects/NarrowPassage/src/NarrowCpp.cpp -o CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.s

CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.o: CMakeFiles/NarrowPassage.dir/flags.make
CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.o: ../src/Renderer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saslab/Projects/NarrowPassage/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.o -c /home/saslab/Projects/NarrowPassage/src/Renderer.cpp

CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saslab/Projects/NarrowPassage/src/Renderer.cpp > CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.i

CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saslab/Projects/NarrowPassage/src/Renderer.cpp -o CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.s

# Object files for target NarrowPassage
NarrowPassage_OBJECTS = \
"CMakeFiles/NarrowPassage.dir/src/Image.cpp.o" \
"CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.o" \
"CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.o"

# External object files for target NarrowPassage
NarrowPassage_EXTERNAL_OBJECTS =

NarrowPassage: CMakeFiles/NarrowPassage.dir/src/Image.cpp.o
NarrowPassage: CMakeFiles/NarrowPassage.dir/src/NarrowCpp.cpp.o
NarrowPassage: CMakeFiles/NarrowPassage.dir/src/Renderer.cpp.o
NarrowPassage: CMakeFiles/NarrowPassage.dir/build.make
NarrowPassage: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.6.2
NarrowPassage: /usr/lib/x86_64-linux-gnu/libsfml-graphics.so.2.5.1
NarrowPassage: /usr/lib/x86_64-linux-gnu/libsfml-window.so.2.5.1
NarrowPassage: /usr/lib/x86_64-linux-gnu/libsfml-system.so.2.5.1
NarrowPassage: CMakeFiles/NarrowPassage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/saslab/Projects/NarrowPassage/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable NarrowPassage"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/NarrowPassage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/NarrowPassage.dir/build: NarrowPassage

.PHONY : CMakeFiles/NarrowPassage.dir/build

CMakeFiles/NarrowPassage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/NarrowPassage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/NarrowPassage.dir/clean

CMakeFiles/NarrowPassage.dir/depend:
	cd /home/saslab/Projects/NarrowPassage/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/saslab/Projects/NarrowPassage /home/saslab/Projects/NarrowPassage /home/saslab/Projects/NarrowPassage/build /home/saslab/Projects/NarrowPassage/build /home/saslab/Projects/NarrowPassage/build/CMakeFiles/NarrowPassage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/NarrowPassage.dir/depend
