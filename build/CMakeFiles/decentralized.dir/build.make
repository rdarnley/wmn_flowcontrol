# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/ryan/wmn_flowcontrol

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ryan/wmn_flowcontrol/build

# Include any dependencies generated for this target.
include CMakeFiles/decentralized.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/decentralized.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/decentralized.dir/flags.make

CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o: CMakeFiles/decentralized.dir/flags.make
CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o: ../src/decentralized_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryan/wmn_flowcontrol/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o -c /home/ryan/wmn_flowcontrol/src/decentralized_main.cpp

CMakeFiles/decentralized.dir/src/decentralized_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/decentralized.dir/src/decentralized_main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryan/wmn_flowcontrol/src/decentralized_main.cpp > CMakeFiles/decentralized.dir/src/decentralized_main.cpp.i

CMakeFiles/decentralized.dir/src/decentralized_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/decentralized.dir/src/decentralized_main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryan/wmn_flowcontrol/src/decentralized_main.cpp -o CMakeFiles/decentralized.dir/src/decentralized_main.cpp.s

CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o.requires:

.PHONY : CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o.requires

CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o.provides: CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o.requires
	$(MAKE) -f CMakeFiles/decentralized.dir/build.make CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o.provides.build
.PHONY : CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o.provides

CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o.provides.build: CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o


CMakeFiles/decentralized.dir/src/decentralized.cpp.o: CMakeFiles/decentralized.dir/flags.make
CMakeFiles/decentralized.dir/src/decentralized.cpp.o: ../src/decentralized.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryan/wmn_flowcontrol/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/decentralized.dir/src/decentralized.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/decentralized.dir/src/decentralized.cpp.o -c /home/ryan/wmn_flowcontrol/src/decentralized.cpp

CMakeFiles/decentralized.dir/src/decentralized.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/decentralized.dir/src/decentralized.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryan/wmn_flowcontrol/src/decentralized.cpp > CMakeFiles/decentralized.dir/src/decentralized.cpp.i

CMakeFiles/decentralized.dir/src/decentralized.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/decentralized.dir/src/decentralized.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryan/wmn_flowcontrol/src/decentralized.cpp -o CMakeFiles/decentralized.dir/src/decentralized.cpp.s

CMakeFiles/decentralized.dir/src/decentralized.cpp.o.requires:

.PHONY : CMakeFiles/decentralized.dir/src/decentralized.cpp.o.requires

CMakeFiles/decentralized.dir/src/decentralized.cpp.o.provides: CMakeFiles/decentralized.dir/src/decentralized.cpp.o.requires
	$(MAKE) -f CMakeFiles/decentralized.dir/build.make CMakeFiles/decentralized.dir/src/decentralized.cpp.o.provides.build
.PHONY : CMakeFiles/decentralized.dir/src/decentralized.cpp.o.provides

CMakeFiles/decentralized.dir/src/decentralized.cpp.o.provides.build: CMakeFiles/decentralized.dir/src/decentralized.cpp.o


CMakeFiles/decentralized.dir/src/factor.cpp.o: CMakeFiles/decentralized.dir/flags.make
CMakeFiles/decentralized.dir/src/factor.cpp.o: ../src/factor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryan/wmn_flowcontrol/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/decentralized.dir/src/factor.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/decentralized.dir/src/factor.cpp.o -c /home/ryan/wmn_flowcontrol/src/factor.cpp

CMakeFiles/decentralized.dir/src/factor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/decentralized.dir/src/factor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryan/wmn_flowcontrol/src/factor.cpp > CMakeFiles/decentralized.dir/src/factor.cpp.i

CMakeFiles/decentralized.dir/src/factor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/decentralized.dir/src/factor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryan/wmn_flowcontrol/src/factor.cpp -o CMakeFiles/decentralized.dir/src/factor.cpp.s

CMakeFiles/decentralized.dir/src/factor.cpp.o.requires:

.PHONY : CMakeFiles/decentralized.dir/src/factor.cpp.o.requires

CMakeFiles/decentralized.dir/src/factor.cpp.o.provides: CMakeFiles/decentralized.dir/src/factor.cpp.o.requires
	$(MAKE) -f CMakeFiles/decentralized.dir/build.make CMakeFiles/decentralized.dir/src/factor.cpp.o.provides.build
.PHONY : CMakeFiles/decentralized.dir/src/factor.cpp.o.provides

CMakeFiles/decentralized.dir/src/factor.cpp.o.provides.build: CMakeFiles/decentralized.dir/src/factor.cpp.o


# Object files for target decentralized
decentralized_OBJECTS = \
"CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o" \
"CMakeFiles/decentralized.dir/src/decentralized.cpp.o" \
"CMakeFiles/decentralized.dir/src/factor.cpp.o"

# External object files for target decentralized
decentralized_EXTERNAL_OBJECTS =

decentralized: CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o
decentralized: CMakeFiles/decentralized.dir/src/decentralized.cpp.o
decentralized: CMakeFiles/decentralized.dir/src/factor.cpp.o
decentralized: CMakeFiles/decentralized.dir/build.make
decentralized: /usr/local/lib/libgtsam.so.4.1.0
decentralized: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
decentralized: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
decentralized: /usr/lib/x86_64-linux-gnu/libboost_thread.so
decentralized: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
decentralized: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
decentralized: /usr/lib/x86_64-linux-gnu/libboost_regex.so
decentralized: /usr/lib/x86_64-linux-gnu/libboost_timer.so
decentralized: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
decentralized: /usr/lib/x86_64-linux-gnu/libboost_system.so
decentralized: /usr/local/lib/libmetis-gtsam.so
decentralized: CMakeFiles/decentralized.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ryan/wmn_flowcontrol/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable decentralized"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/decentralized.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/decentralized.dir/build: decentralized

.PHONY : CMakeFiles/decentralized.dir/build

CMakeFiles/decentralized.dir/requires: CMakeFiles/decentralized.dir/src/decentralized_main.cpp.o.requires
CMakeFiles/decentralized.dir/requires: CMakeFiles/decentralized.dir/src/decentralized.cpp.o.requires
CMakeFiles/decentralized.dir/requires: CMakeFiles/decentralized.dir/src/factor.cpp.o.requires

.PHONY : CMakeFiles/decentralized.dir/requires

CMakeFiles/decentralized.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/decentralized.dir/cmake_clean.cmake
.PHONY : CMakeFiles/decentralized.dir/clean

CMakeFiles/decentralized.dir/depend:
	cd /home/ryan/wmn_flowcontrol/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ryan/wmn_flowcontrol /home/ryan/wmn_flowcontrol /home/ryan/wmn_flowcontrol/build /home/ryan/wmn_flowcontrol/build /home/ryan/wmn_flowcontrol/build/CMakeFiles/decentralized.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/decentralized.dir/depend
