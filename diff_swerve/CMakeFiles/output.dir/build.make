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
include CMakeFiles/output.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/output.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/output.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/output.dir/flags.make

CMakeFiles/output.dir/src/components/n20_motor.cpp.o: CMakeFiles/output.dir/flags.make
CMakeFiles/output.dir/src/components/n20_motor.cpp.o: src/components/n20_motor.cpp
CMakeFiles/output.dir/src/components/n20_motor.cpp.o: CMakeFiles/output.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/output.dir/src/components/n20_motor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/output.dir/src/components/n20_motor.cpp.o -MF CMakeFiles/output.dir/src/components/n20_motor.cpp.o.d -o CMakeFiles/output.dir/src/components/n20_motor.cpp.o -c /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/n20_motor.cpp

CMakeFiles/output.dir/src/components/n20_motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/output.dir/src/components/n20_motor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/n20_motor.cpp > CMakeFiles/output.dir/src/components/n20_motor.cpp.i

CMakeFiles/output.dir/src/components/n20_motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/output.dir/src/components/n20_motor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/n20_motor.cpp -o CMakeFiles/output.dir/src/components/n20_motor.cpp.s

CMakeFiles/output.dir/src/components/swerve_module.cpp.o: CMakeFiles/output.dir/flags.make
CMakeFiles/output.dir/src/components/swerve_module.cpp.o: src/components/swerve_module.cpp
CMakeFiles/output.dir/src/components/swerve_module.cpp.o: CMakeFiles/output.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/output.dir/src/components/swerve_module.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/output.dir/src/components/swerve_module.cpp.o -MF CMakeFiles/output.dir/src/components/swerve_module.cpp.o.d -o CMakeFiles/output.dir/src/components/swerve_module.cpp.o -c /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/swerve_module.cpp

CMakeFiles/output.dir/src/components/swerve_module.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/output.dir/src/components/swerve_module.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/swerve_module.cpp > CMakeFiles/output.dir/src/components/swerve_module.cpp.i

CMakeFiles/output.dir/src/components/swerve_module.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/output.dir/src/components/swerve_module.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/swerve_module.cpp -o CMakeFiles/output.dir/src/components/swerve_module.cpp.s

CMakeFiles/output.dir/src/components/xbox_controller.cpp.o: CMakeFiles/output.dir/flags.make
CMakeFiles/output.dir/src/components/xbox_controller.cpp.o: src/components/xbox_controller.cpp
CMakeFiles/output.dir/src/components/xbox_controller.cpp.o: CMakeFiles/output.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/output.dir/src/components/xbox_controller.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/output.dir/src/components/xbox_controller.cpp.o -MF CMakeFiles/output.dir/src/components/xbox_controller.cpp.o.d -o CMakeFiles/output.dir/src/components/xbox_controller.cpp.o -c /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/xbox_controller.cpp

CMakeFiles/output.dir/src/components/xbox_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/output.dir/src/components/xbox_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/xbox_controller.cpp > CMakeFiles/output.dir/src/components/xbox_controller.cpp.i

CMakeFiles/output.dir/src/components/xbox_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/output.dir/src/components/xbox_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/components/xbox_controller.cpp -o CMakeFiles/output.dir/src/components/xbox_controller.cpp.s

CMakeFiles/output.dir/src/main.cpp.o: CMakeFiles/output.dir/flags.make
CMakeFiles/output.dir/src/main.cpp.o: src/main.cpp
CMakeFiles/output.dir/src/main.cpp.o: CMakeFiles/output.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/output.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/output.dir/src/main.cpp.o -MF CMakeFiles/output.dir/src/main.cpp.o.d -o CMakeFiles/output.dir/src/main.cpp.o -c /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/main.cpp

CMakeFiles/output.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/output.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/main.cpp > CMakeFiles/output.dir/src/main.cpp.i

CMakeFiles/output.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/output.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/main.cpp -o CMakeFiles/output.dir/src/main.cpp.s

CMakeFiles/output.dir/src/utils/logger.cpp.o: CMakeFiles/output.dir/flags.make
CMakeFiles/output.dir/src/utils/logger.cpp.o: src/utils/logger.cpp
CMakeFiles/output.dir/src/utils/logger.cpp.o: CMakeFiles/output.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/output.dir/src/utils/logger.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/output.dir/src/utils/logger.cpp.o -MF CMakeFiles/output.dir/src/utils/logger.cpp.o.d -o CMakeFiles/output.dir/src/utils/logger.cpp.o -c /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/utils/logger.cpp

CMakeFiles/output.dir/src/utils/logger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/output.dir/src/utils/logger.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/utils/logger.cpp > CMakeFiles/output.dir/src/utils/logger.cpp.i

CMakeFiles/output.dir/src/utils/logger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/output.dir/src/utils/logger.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/utils/logger.cpp -o CMakeFiles/output.dir/src/utils/logger.cpp.s

CMakeFiles/output.dir/src/utils/util.cpp.o: CMakeFiles/output.dir/flags.make
CMakeFiles/output.dir/src/utils/util.cpp.o: src/utils/util.cpp
CMakeFiles/output.dir/src/utils/util.cpp.o: CMakeFiles/output.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/output.dir/src/utils/util.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/output.dir/src/utils/util.cpp.o -MF CMakeFiles/output.dir/src/utils/util.cpp.o.d -o CMakeFiles/output.dir/src/utils/util.cpp.o -c /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/utils/util.cpp

CMakeFiles/output.dir/src/utils/util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/output.dir/src/utils/util.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/utils/util.cpp > CMakeFiles/output.dir/src/utils/util.cpp.i

CMakeFiles/output.dir/src/utils/util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/output.dir/src/utils/util.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/3d-Printed-Differential-Swerve/diff_swerve/src/utils/util.cpp -o CMakeFiles/output.dir/src/utils/util.cpp.s

# Object files for target output
output_OBJECTS = \
"CMakeFiles/output.dir/src/components/n20_motor.cpp.o" \
"CMakeFiles/output.dir/src/components/swerve_module.cpp.o" \
"CMakeFiles/output.dir/src/components/xbox_controller.cpp.o" \
"CMakeFiles/output.dir/src/main.cpp.o" \
"CMakeFiles/output.dir/src/utils/logger.cpp.o" \
"CMakeFiles/output.dir/src/utils/util.cpp.o"

# External object files for target output
output_EXTERNAL_OBJECTS =

output: CMakeFiles/output.dir/src/components/n20_motor.cpp.o
output: CMakeFiles/output.dir/src/components/swerve_module.cpp.o
output: CMakeFiles/output.dir/src/components/xbox_controller.cpp.o
output: CMakeFiles/output.dir/src/main.cpp.o
output: CMakeFiles/output.dir/src/utils/logger.cpp.o
output: CMakeFiles/output.dir/src/utils/util.cpp.o
output: CMakeFiles/output.dir/build.make
output: libminipid.a
output: /usr/local/lib/libpigpio.so
output: /usr/lib/aarch64-linux-gnu/librt.a
output: /usr/lib/aarch64-linux-gnu/libevdev.so
output: CMakeFiles/output.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable output"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/output.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/output.dir/build: output
.PHONY : CMakeFiles/output.dir/build

CMakeFiles/output.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/output.dir/cmake_clean.cmake
.PHONY : CMakeFiles/output.dir/clean

CMakeFiles/output.dir/depend:
	cd /home/pi/3d-Printed-Differential-Swerve/diff_swerve && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/3d-Printed-Differential-Swerve/diff_swerve /home/pi/3d-Printed-Differential-Swerve/diff_swerve /home/pi/3d-Printed-Differential-Swerve/diff_swerve /home/pi/3d-Printed-Differential-Swerve/diff_swerve /home/pi/3d-Printed-Differential-Swerve/diff_swerve/CMakeFiles/output.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/output.dir/depend

