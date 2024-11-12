# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/3d-Printed-Differential-Swerve/diff_swerve

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/3d-Printed-Differential-Swerve/diff_swerve

# Include any dependencies generated for this target.
include CMakeFiles/minipid.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/minipid.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/minipid.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/minipid.dir/flags.make

CMakeFiles/minipid.dir/libs/MiniPID.cpp.o: CMakeFiles/minipid.dir/flags.make
CMakeFiles/minipid.dir/libs/MiniPID.cpp.o: libs/MiniPID.cpp
CMakeFiles/minipid.dir/libs/MiniPID.cpp.o: CMakeFiles/minipid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/minipid.dir/libs/MiniPID.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/minipid.dir/libs/MiniPID.cpp.o -MF CMakeFiles/minipid.dir/libs/MiniPID.cpp.o.d -o CMakeFiles/minipid.dir/libs/MiniPID.cpp.o -c /home/pi/3d-Printed-Differential-Swerve/diff_swerve/libs/MiniPID.cpp

CMakeFiles/minipid.dir/libs/MiniPID.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/minipid.dir/libs/MiniPID.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/3d-Printed-Differential-Swerve/diff_swerve/libs/MiniPID.cpp > CMakeFiles/minipid.dir/libs/MiniPID.cpp.i

CMakeFiles/minipid.dir/libs/MiniPID.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/minipid.dir/libs/MiniPID.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/3d-Printed-Differential-Swerve/diff_swerve/libs/MiniPID.cpp -o CMakeFiles/minipid.dir/libs/MiniPID.cpp.s

# Object files for target minipid
minipid_OBJECTS = \
"CMakeFiles/minipid.dir/libs/MiniPID.cpp.o"

# External object files for target minipid
minipid_EXTERNAL_OBJECTS =

libminipid.a: CMakeFiles/minipid.dir/libs/MiniPID.cpp.o
libminipid.a: CMakeFiles/minipid.dir/build.make
libminipid.a: CMakeFiles/minipid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libminipid.a"
	$(CMAKE_COMMAND) -P CMakeFiles/minipid.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/minipid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/minipid.dir/build: libminipid.a
.PHONY : CMakeFiles/minipid.dir/build

CMakeFiles/minipid.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/minipid.dir/cmake_clean.cmake
.PHONY : CMakeFiles/minipid.dir/clean

CMakeFiles/minipid.dir/depend:
	cd /home/pi/3d-Printed-Differential-Swerve/diff_swerve && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/3d-Printed-Differential-Swerve/diff_swerve /home/pi/3d-Printed-Differential-Swerve/diff_swerve /home/pi/3d-Printed-Differential-Swerve/diff_swerve /home/pi/3d-Printed-Differential-Swerve/diff_swerve /home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles/minipid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/minipid.dir/depend

