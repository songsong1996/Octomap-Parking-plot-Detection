# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/songsong/Documents/Softwares/Clion/clion-2018.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/songsong/Documents/Softwares/Clion/clion-2018.3.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug"

# Include any dependencies generated for this target.
include src/CMakeFiles/detect.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/detect.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/detect.dir/flags.make

src/CMakeFiles/detect.dir/main.cpp.o: src/CMakeFiles/detect.dir/flags.make
src/CMakeFiles/detect.dir/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/detect.dir/main.cpp.o"
	cd "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/src" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/main.cpp.o -c "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/src/main.cpp"

src/CMakeFiles/detect.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/main.cpp.i"
	cd "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/src" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/src/main.cpp" > CMakeFiles/detect.dir/main.cpp.i

src/CMakeFiles/detect.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/main.cpp.s"
	cd "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/src" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/src/main.cpp" -o CMakeFiles/detect.dir/main.cpp.s

# Object files for target detect
detect_OBJECTS = \
"CMakeFiles/detect.dir/main.cpp.o"

# External object files for target detect
detect_EXTERNAL_OBJECTS =

../bin/detect: src/CMakeFiles/detect.dir/main.cpp.o
../bin/detect: src/CMakeFiles/detect.dir/build.make
../bin/detect: src/libread_folder.a
../bin/detect: src/libread_file.a
../bin/detect: src/libdetect_objects.a
../bin/detect: src/libcar_space_norm.a
../bin/detect: src/libransac_line_detection.a
../bin/detect: /usr/local/lib/liboctomap.so
../bin/detect: /usr/local/lib/liboctomath.so
../bin/detect: /usr/local/opencv4/lib/libopencv_dnn.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_gapi.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_ml.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_objdetect.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_photo.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_stitching.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_video.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_calib3d.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_features2d.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_flann.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_highgui.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_videoio.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_imgcodecs.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_imgproc.so.4.0.1
../bin/detect: /usr/local/opencv4/lib/libopencv_core.so.4.0.1
../bin/detect: src/CMakeFiles/detect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/detect"
	cd "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/src" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/detect.dir/build: ../bin/detect

.PHONY : src/CMakeFiles/detect.dir/build

src/CMakeFiles/detect.dir/clean:
	cd "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/src" && $(CMAKE_COMMAND) -P CMakeFiles/detect.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/detect.dir/clean

src/CMakeFiles/detect.dir/depend:
	cd "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)" "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/src" "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug" "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/src" "/home/songsong/Documents/private/detection/octomap_detectCar_2 (copy)/cmake-build-debug/src/CMakeFiles/detect.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : src/CMakeFiles/detect.dir/depend

