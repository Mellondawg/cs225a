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
include project/CMakeFiles/simviz_project.dir/depend.make

# Include the progress variables for this target.
include project/CMakeFiles/simviz_project.dir/progress.make

# Include the compile flags for this target's objects.
include project/CMakeFiles/simviz_project.dir/flags.make

project/CMakeFiles/simviz_project.dir/simviz_project.cpp.o: project/CMakeFiles/simviz_project.dir/flags.make
project/CMakeFiles/simviz_project.dir/simviz_project.cpp.o: ../project/simviz_project.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/davidxue/code/sai2/apps/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object project/CMakeFiles/simviz_project.dir/simviz_project.cpp.o"
	cd /Users/davidxue/code/sai2/apps/cs225a/build/project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simviz_project.dir/simviz_project.cpp.o -c /Users/davidxue/code/sai2/apps/cs225a/project/simviz_project.cpp

project/CMakeFiles/simviz_project.dir/simviz_project.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simviz_project.dir/simviz_project.cpp.i"
	cd /Users/davidxue/code/sai2/apps/cs225a/build/project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/davidxue/code/sai2/apps/cs225a/project/simviz_project.cpp > CMakeFiles/simviz_project.dir/simviz_project.cpp.i

project/CMakeFiles/simviz_project.dir/simviz_project.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simviz_project.dir/simviz_project.cpp.s"
	cd /Users/davidxue/code/sai2/apps/cs225a/build/project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/davidxue/code/sai2/apps/cs225a/project/simviz_project.cpp -o CMakeFiles/simviz_project.dir/simviz_project.cpp.s

# Object files for target simviz_project
simviz_project_OBJECTS = \
"CMakeFiles/simviz_project.dir/simviz_project.cpp.o"

# External object files for target simviz_project
simviz_project_EXTERNAL_OBJECTS =

../bin/project/simviz_project: project/CMakeFiles/simviz_project.dir/simviz_project.cpp.o
../bin/project/simviz_project: project/CMakeFiles/simviz_project.dir/build.make
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-common/build/libsai2-common.a
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/chai3d/build/libchai3d.a
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project/simviz_project: /usr/local/lib/libtinyxml2.dylib
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-simulation/build/libsai2-simulation.a
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-model/build/libsai2-model.a
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project/simviz_project: /usr/local/lib/libtinyxml2.dylib
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-model/rbdl/build/librbdl.dylib
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-graphics/build/libsai2-graphics.a
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project/simviz_project: /usr/local/lib/libtinyxml2.dylib
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/chai3d/build/libchai3d.a
../bin/project/simviz_project: /usr/local/lib/libjsoncpp.dylib
../bin/project/simviz_project: /usr/local/lib/libhiredis.dylib
../bin/project/simviz_project: /usr/local/lib/libglfw.dylib
../bin/project/simviz_project: /Users/davidxue/code/sai2/core/sai2-model/rbdl/build/librbdl.dylib
../bin/project/simviz_project: /usr/local/lib/libjsoncpp.dylib
../bin/project/simviz_project: /usr/local/lib/libhiredis.dylib
../bin/project/simviz_project: /usr/local/lib/libglfw.dylib
../bin/project/simviz_project: project/CMakeFiles/simviz_project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/davidxue/code/sai2/apps/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/project/simviz_project"
	cd /Users/davidxue/code/sai2/apps/cs225a/build/project && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simviz_project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
project/CMakeFiles/simviz_project.dir/build: ../bin/project/simviz_project

.PHONY : project/CMakeFiles/simviz_project.dir/build

project/CMakeFiles/simviz_project.dir/clean:
	cd /Users/davidxue/code/sai2/apps/cs225a/build/project && $(CMAKE_COMMAND) -P CMakeFiles/simviz_project.dir/cmake_clean.cmake
.PHONY : project/CMakeFiles/simviz_project.dir/clean

project/CMakeFiles/simviz_project.dir/depend:
	cd /Users/davidxue/code/sai2/apps/cs225a/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/davidxue/code/sai2/apps/cs225a /Users/davidxue/code/sai2/apps/cs225a/project /Users/davidxue/code/sai2/apps/cs225a/build /Users/davidxue/code/sai2/apps/cs225a/build/project /Users/davidxue/code/sai2/apps/cs225a/build/project/CMakeFiles/simviz_project.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : project/CMakeFiles/simviz_project.dir/depend

