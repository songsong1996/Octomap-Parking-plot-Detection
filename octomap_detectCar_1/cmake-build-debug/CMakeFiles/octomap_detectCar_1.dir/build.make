# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /home/songsong/SoftWare/clion-2018.2.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/songsong/SoftWare/clion-2018.2.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/songsong/Documents/detection/octomap_detectCar_1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/songsong/Documents/detection/octomap_detectCar_1/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/octomap_detectCar_1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/octomap_detectCar_1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/octomap_detectCar_1.dir/flags.make

CMakeFiles/octomap_detectCar_1.dir/main.cpp.o: CMakeFiles/octomap_detectCar_1.dir/flags.make
CMakeFiles/octomap_detectCar_1.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/songsong/Documents/detection/octomap_detectCar_1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/octomap_detectCar_1.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap_detectCar_1.dir/main.cpp.o -c /home/songsong/Documents/detection/octomap_detectCar_1/main.cpp

CMakeFiles/octomap_detectCar_1.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap_detectCar_1.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/songsong/Documents/detection/octomap_detectCar_1/main.cpp > CMakeFiles/octomap_detectCar_1.dir/main.cpp.i

CMakeFiles/octomap_detectCar_1.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap_detectCar_1.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/songsong/Documents/detection/octomap_detectCar_1/main.cpp -o CMakeFiles/octomap_detectCar_1.dir/main.cpp.s

# Object files for target octomap_detectCar_1
octomap_detectCar_1_OBJECTS = \
"CMakeFiles/octomap_detectCar_1.dir/main.cpp.o"

# External object files for target octomap_detectCar_1
octomap_detectCar_1_EXTERNAL_OBJECTS =

octomap_detectCar_1: CMakeFiles/octomap_detectCar_1.dir/main.cpp.o
octomap_detectCar_1: CMakeFiles/octomap_detectCar_1.dir/build.make
octomap_detectCar_1: CMakeFiles/octomap_detectCar_1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/songsong/Documents/detection/octomap_detectCar_1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable octomap_detectCar_1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/octomap_detectCar_1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/octomap_detectCar_1.dir/build: octomap_detectCar_1

.PHONY : CMakeFiles/octomap_detectCar_1.dir/build

CMakeFiles/octomap_detectCar_1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/octomap_detectCar_1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/octomap_detectCar_1.dir/clean

CMakeFiles/octomap_detectCar_1.dir/depend:
	cd /home/songsong/Documents/detection/octomap_detectCar_1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/songsong/Documents/detection/octomap_detectCar_1 /home/songsong/Documents/detection/octomap_detectCar_1 /home/songsong/Documents/detection/octomap_detectCar_1/cmake-build-debug /home/songsong/Documents/detection/octomap_detectCar_1/cmake-build-debug /home/songsong/Documents/detection/octomap_detectCar_1/cmake-build-debug/CMakeFiles/octomap_detectCar_1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/octomap_detectCar_1.dir/depend
