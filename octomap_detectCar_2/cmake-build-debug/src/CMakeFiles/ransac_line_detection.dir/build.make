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
CMAKE_SOURCE_DIR = /home/songsong/Documents/detection/octomap_detectCar_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug

# Include any dependencies generated for this target.
include src/CMakeFiles/ransac_line_detection.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/ransac_line_detection.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/ransac_line_detection.dir/flags.make

src/CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.o: src/CMakeFiles/ransac_line_detection.dir/flags.make
src/CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.o: ../src/line_ransac.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.o"
	cd /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.o -c /home/songsong/Documents/detection/octomap_detectCar_2/src/line_ransac.cpp

src/CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.i"
	cd /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/songsong/Documents/detection/octomap_detectCar_2/src/line_ransac.cpp > CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.i

src/CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.s"
	cd /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/songsong/Documents/detection/octomap_detectCar_2/src/line_ransac.cpp -o CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.s

# Object files for target ransac_line_detection
ransac_line_detection_OBJECTS = \
"CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.o"

# External object files for target ransac_line_detection
ransac_line_detection_EXTERNAL_OBJECTS =

src/libransac_line_detection.a: src/CMakeFiles/ransac_line_detection.dir/line_ransac.cpp.o
src/libransac_line_detection.a: src/CMakeFiles/ransac_line_detection.dir/build.make
src/libransac_line_detection.a: src/CMakeFiles/ransac_line_detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libransac_line_detection.a"
	cd /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/ransac_line_detection.dir/cmake_clean_target.cmake
	cd /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ransac_line_detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/ransac_line_detection.dir/build: src/libransac_line_detection.a

.PHONY : src/CMakeFiles/ransac_line_detection.dir/build

src/CMakeFiles/ransac_line_detection.dir/clean:
	cd /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/ransac_line_detection.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ransac_line_detection.dir/clean

src/CMakeFiles/ransac_line_detection.dir/depend:
	cd /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/songsong/Documents/detection/octomap_detectCar_2 /home/songsong/Documents/detection/octomap_detectCar_2/src /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/src /home/songsong/Documents/detection/octomap_detectCar_2/cmake-build-debug/src/CMakeFiles/ransac_line_detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ransac_line_detection.dir/depend

