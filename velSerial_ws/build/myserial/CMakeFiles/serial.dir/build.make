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
CMAKE_SOURCE_DIR = /home/cxc/velSerial_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cxc/velSerial_ws/build

# Include any dependencies generated for this target.
include myserial/CMakeFiles/serial.dir/depend.make

# Include the progress variables for this target.
include myserial/CMakeFiles/serial.dir/progress.make

# Include the compile flags for this target's objects.
include myserial/CMakeFiles/serial.dir/flags.make

myserial/CMakeFiles/serial.dir/src/serial_port.cpp.o: myserial/CMakeFiles/serial.dir/flags.make
myserial/CMakeFiles/serial.dir/src/serial_port.cpp.o: /home/cxc/velSerial_ws/src/myserial/src/serial_port.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cxc/velSerial_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object myserial/CMakeFiles/serial.dir/src/serial_port.cpp.o"
	cd /home/cxc/velSerial_ws/build/myserial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial.dir/src/serial_port.cpp.o -c /home/cxc/velSerial_ws/src/myserial/src/serial_port.cpp

myserial/CMakeFiles/serial.dir/src/serial_port.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial.dir/src/serial_port.cpp.i"
	cd /home/cxc/velSerial_ws/build/myserial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cxc/velSerial_ws/src/myserial/src/serial_port.cpp > CMakeFiles/serial.dir/src/serial_port.cpp.i

myserial/CMakeFiles/serial.dir/src/serial_port.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial.dir/src/serial_port.cpp.s"
	cd /home/cxc/velSerial_ws/build/myserial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cxc/velSerial_ws/src/myserial/src/serial_port.cpp -o CMakeFiles/serial.dir/src/serial_port.cpp.s

myserial/CMakeFiles/serial.dir/src/main.cpp.o: myserial/CMakeFiles/serial.dir/flags.make
myserial/CMakeFiles/serial.dir/src/main.cpp.o: /home/cxc/velSerial_ws/src/myserial/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cxc/velSerial_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object myserial/CMakeFiles/serial.dir/src/main.cpp.o"
	cd /home/cxc/velSerial_ws/build/myserial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial.dir/src/main.cpp.o -c /home/cxc/velSerial_ws/src/myserial/src/main.cpp

myserial/CMakeFiles/serial.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial.dir/src/main.cpp.i"
	cd /home/cxc/velSerial_ws/build/myserial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cxc/velSerial_ws/src/myserial/src/main.cpp > CMakeFiles/serial.dir/src/main.cpp.i

myserial/CMakeFiles/serial.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial.dir/src/main.cpp.s"
	cd /home/cxc/velSerial_ws/build/myserial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cxc/velSerial_ws/src/myserial/src/main.cpp -o CMakeFiles/serial.dir/src/main.cpp.s

# Object files for target serial
serial_OBJECTS = \
"CMakeFiles/serial.dir/src/serial_port.cpp.o" \
"CMakeFiles/serial.dir/src/main.cpp.o"

# External object files for target serial
serial_EXTERNAL_OBJECTS =

/home/cxc/velSerial_ws/devel/lib/libserial.so: myserial/CMakeFiles/serial.dir/src/serial_port.cpp.o
/home/cxc/velSerial_ws/devel/lib/libserial.so: myserial/CMakeFiles/serial.dir/src/main.cpp.o
/home/cxc/velSerial_ws/devel/lib/libserial.so: myserial/CMakeFiles/serial.dir/build.make
/home/cxc/velSerial_ws/devel/lib/libserial.so: myserial/CMakeFiles/serial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cxc/velSerial_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library /home/cxc/velSerial_ws/devel/lib/libserial.so"
	cd /home/cxc/velSerial_ws/build/myserial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
myserial/CMakeFiles/serial.dir/build: /home/cxc/velSerial_ws/devel/lib/libserial.so

.PHONY : myserial/CMakeFiles/serial.dir/build

myserial/CMakeFiles/serial.dir/clean:
	cd /home/cxc/velSerial_ws/build/myserial && $(CMAKE_COMMAND) -P CMakeFiles/serial.dir/cmake_clean.cmake
.PHONY : myserial/CMakeFiles/serial.dir/clean

myserial/CMakeFiles/serial.dir/depend:
	cd /home/cxc/velSerial_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cxc/velSerial_ws/src /home/cxc/velSerial_ws/src/myserial /home/cxc/velSerial_ws/build /home/cxc/velSerial_ws/build/myserial /home/cxc/velSerial_ws/build/myserial/CMakeFiles/serial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : myserial/CMakeFiles/serial.dir/depend

