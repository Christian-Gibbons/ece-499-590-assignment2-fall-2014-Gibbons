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
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/build

# Include any dependencies generated for this target.
include CMakeFiles/camera_move.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera_move.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera_move.dir/flags.make

CMakeFiles/camera_move.dir/camera_move.cc.o: CMakeFiles/camera_move.dir/flags.make
CMakeFiles/camera_move.dir/camera_move.cc.o: ../camera_move.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camera_move.dir/camera_move.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camera_move.dir/camera_move.cc.o -c /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/camera_move.cc

CMakeFiles/camera_move.dir/camera_move.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_move.dir/camera_move.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/camera_move.cc > CMakeFiles/camera_move.dir/camera_move.cc.i

CMakeFiles/camera_move.dir/camera_move.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_move.dir/camera_move.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/camera_move.cc -o CMakeFiles/camera_move.dir/camera_move.cc.s

CMakeFiles/camera_move.dir/camera_move.cc.o.requires:
.PHONY : CMakeFiles/camera_move.dir/camera_move.cc.o.requires

CMakeFiles/camera_move.dir/camera_move.cc.o.provides: CMakeFiles/camera_move.dir/camera_move.cc.o.requires
	$(MAKE) -f CMakeFiles/camera_move.dir/build.make CMakeFiles/camera_move.dir/camera_move.cc.o.provides.build
.PHONY : CMakeFiles/camera_move.dir/camera_move.cc.o.provides

CMakeFiles/camera_move.dir/camera_move.cc.o.provides.build: CMakeFiles/camera_move.dir/camera_move.cc.o

# Object files for target camera_move
camera_move_OBJECTS = \
"CMakeFiles/camera_move.dir/camera_move.cc.o"

# External object files for target camera_move
camera_move_EXTERNAL_OBJECTS =

libcamera_move.so: CMakeFiles/camera_move.dir/camera_move.cc.o
libcamera_move.so: CMakeFiles/camera_move.dir/build.make
libcamera_move.so: CMakeFiles/camera_move.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libcamera_move.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera_move.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera_move.dir/build: libcamera_move.so
.PHONY : CMakeFiles/camera_move.dir/build

CMakeFiles/camera_move.dir/requires: CMakeFiles/camera_move.dir/camera_move.cc.o.requires
.PHONY : CMakeFiles/camera_move.dir/requires

CMakeFiles/camera_move.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera_move.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera_move.dir/clean

CMakeFiles/camera_move.dir/depend:
	cd /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/build /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/build /home/student/projects/ECE-499-590-Fall-2014/d_diff_drive_robot/build/CMakeFiles/camera_move.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera_move.dir/depend

