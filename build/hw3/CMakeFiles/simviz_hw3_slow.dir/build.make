# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.14.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.14.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/davidxue/code/sai2/apps/cs225a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/davidxue/code/sai2/apps/cs225a/build

# Include any dependencies generated for this target.
include hw3/CMakeFiles/simviz_hw3_slow.dir/depend.make

# Include the progress variables for this target.
include hw3/CMakeFiles/simviz_hw3_slow.dir/progress.make

# Include the compile flags for this target's objects.
include hw3/CMakeFiles/simviz_hw3_slow.dir/flags.make

hw3/CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.o: hw3/CMakeFiles/simviz_hw3_slow.dir/flags.make
hw3/CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.o: ../hw3/simviz_hw3_slow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/davidxue/code/sai2/apps/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object hw3/CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.o"
	cd /Users/davidxue/code/sai2/apps/cs225a/build/hw3 && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.o -c /Users/davidxue/code/sai2/apps/cs225a/hw3/simviz_hw3_slow.cpp

hw3/CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.i"
	cd /Users/davidxue/code/sai2/apps/cs225a/build/hw3 && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/davidxue/code/sai2/apps/cs225a/hw3/simviz_hw3_slow.cpp > CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.i

hw3/CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.s"
	cd /Users/davidxue/code/sai2/apps/cs225a/build/hw3 && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/davidxue/code/sai2/apps/cs225a/hw3/simviz_hw3_slow.cpp -o CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.s

# Object files for target simviz_hw3_slow
simviz_hw3_slow_OBJECTS = \
"CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.o"

# External object files for target simviz_hw3_slow
simviz_hw3_slow_EXTERNAL_OBJECTS =

../bin/hw3/simviz_hw3_slow: hw3/CMakeFiles/simviz_hw3_slow.dir/simviz_hw3_slow.cpp.o
../bin/hw3/simviz_hw3_slow: hw3/CMakeFiles/simviz_hw3_slow.dir/build.make
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-common/build/libsai2-common.a
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/chai3d/build/libchai3d.a
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libtinyxml2.dylib
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-simulation/build/libsai2-simulation.a
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-model/build/libsai2-model.a
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libtinyxml2.dylib
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-model/rbdl/build/librbdl.dylib
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-graphics/build/libsai2-graphics.a
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libtinyxml2.dylib
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/chai3d/build/libchai3d.a
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libjsoncpp.dylib
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libhiredis.dylib
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libglfw.dylib
../bin/hw3/simviz_hw3_slow: /Users/davidxue/code/sai2/core/sai2-model/rbdl/build/librbdl.dylib
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libjsoncpp.dylib
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libhiredis.dylib
../bin/hw3/simviz_hw3_slow: /usr/local/lib/libglfw.dylib
../bin/hw3/simviz_hw3_slow: hw3/CMakeFiles/simviz_hw3_slow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/davidxue/code/sai2/apps/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/hw3/simviz_hw3_slow"
	cd /Users/davidxue/code/sai2/apps/cs225a/build/hw3 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simviz_hw3_slow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hw3/CMakeFiles/simviz_hw3_slow.dir/build: ../bin/hw3/simviz_hw3_slow

.PHONY : hw3/CMakeFiles/simviz_hw3_slow.dir/build

hw3/CMakeFiles/simviz_hw3_slow.dir/clean:
	cd /Users/davidxue/code/sai2/apps/cs225a/build/hw3 && $(CMAKE_COMMAND) -P CMakeFiles/simviz_hw3_slow.dir/cmake_clean.cmake
.PHONY : hw3/CMakeFiles/simviz_hw3_slow.dir/clean

hw3/CMakeFiles/simviz_hw3_slow.dir/depend:
	cd /Users/davidxue/code/sai2/apps/cs225a/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/davidxue/code/sai2/apps/cs225a /Users/davidxue/code/sai2/apps/cs225a/hw3 /Users/davidxue/code/sai2/apps/cs225a/build /Users/davidxue/code/sai2/apps/cs225a/build/hw3 /Users/davidxue/code/sai2/apps/cs225a/build/hw3/CMakeFiles/simviz_hw3_slow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hw3/CMakeFiles/simviz_hw3_slow.dir/depend

