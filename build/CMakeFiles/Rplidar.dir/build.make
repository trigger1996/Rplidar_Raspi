# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/pi/src/projects/00_Rplidar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/src/projects/00_Rplidar/build

# Include any dependencies generated for this target.
include CMakeFiles/Rplidar.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Rplidar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Rplidar.dir/flags.make

CMakeFiles/Rplidar.dir/main.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Rplidar.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/main.cpp.o -c /home/pi/src/projects/00_Rplidar/main.cpp

CMakeFiles/Rplidar.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/main.cpp > CMakeFiles/Rplidar.dir/main.cpp.i

CMakeFiles/Rplidar.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/main.cpp -o CMakeFiles/Rplidar.dir/main.cpp.s

CMakeFiles/Rplidar.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/main.cpp.o.requires

CMakeFiles/Rplidar.dir/main.cpp.o.provides: CMakeFiles/Rplidar.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/main.cpp.o.provides

CMakeFiles/Rplidar.dir/main.cpp.o.provides.build: CMakeFiles/Rplidar.dir/main.cpp.o


CMakeFiles/Rplidar.dir/lidar_driver.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/lidar_driver.cpp.o: ../lidar_driver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Rplidar.dir/lidar_driver.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/lidar_driver.cpp.o -c /home/pi/src/projects/00_Rplidar/lidar_driver.cpp

CMakeFiles/Rplidar.dir/lidar_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/lidar_driver.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/lidar_driver.cpp > CMakeFiles/Rplidar.dir/lidar_driver.cpp.i

CMakeFiles/Rplidar.dir/lidar_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/lidar_driver.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/lidar_driver.cpp -o CMakeFiles/Rplidar.dir/lidar_driver.cpp.s

CMakeFiles/Rplidar.dir/lidar_driver.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/lidar_driver.cpp.o.requires

CMakeFiles/Rplidar.dir/lidar_driver.cpp.o.provides: CMakeFiles/Rplidar.dir/lidar_driver.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/lidar_driver.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/lidar_driver.cpp.o.provides

CMakeFiles/Rplidar.dir/lidar_driver.cpp.o.provides.build: CMakeFiles/Rplidar.dir/lidar_driver.cpp.o


CMakeFiles/Rplidar.dir/og_mapping.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/og_mapping.cpp.o: ../og_mapping.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Rplidar.dir/og_mapping.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/og_mapping.cpp.o -c /home/pi/src/projects/00_Rplidar/og_mapping.cpp

CMakeFiles/Rplidar.dir/og_mapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/og_mapping.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/og_mapping.cpp > CMakeFiles/Rplidar.dir/og_mapping.cpp.i

CMakeFiles/Rplidar.dir/og_mapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/og_mapping.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/og_mapping.cpp -o CMakeFiles/Rplidar.dir/og_mapping.cpp.s

CMakeFiles/Rplidar.dir/og_mapping.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/og_mapping.cpp.o.requires

CMakeFiles/Rplidar.dir/og_mapping.cpp.o.provides: CMakeFiles/Rplidar.dir/og_mapping.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/og_mapping.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/og_mapping.cpp.o.provides

CMakeFiles/Rplidar.dir/og_mapping.cpp.o.provides.build: CMakeFiles/Rplidar.dir/og_mapping.cpp.o


CMakeFiles/Rplidar.dir/icp_interface.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/icp_interface.cpp.o: ../icp_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Rplidar.dir/icp_interface.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/icp_interface.cpp.o -c /home/pi/src/projects/00_Rplidar/icp_interface.cpp

CMakeFiles/Rplidar.dir/icp_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/icp_interface.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/icp_interface.cpp > CMakeFiles/Rplidar.dir/icp_interface.cpp.i

CMakeFiles/Rplidar.dir/icp_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/icp_interface.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/icp_interface.cpp -o CMakeFiles/Rplidar.dir/icp_interface.cpp.s

CMakeFiles/Rplidar.dir/icp_interface.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/icp_interface.cpp.o.requires

CMakeFiles/Rplidar.dir/icp_interface.cpp.o.provides: CMakeFiles/Rplidar.dir/icp_interface.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/icp_interface.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/icp_interface.cpp.o.provides

CMakeFiles/Rplidar.dir/icp_interface.cpp.o.provides.build: CMakeFiles/Rplidar.dir/icp_interface.cpp.o


CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o: ../icp_opencv/icp-opencv/icp.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o   -c /home/pi/src/projects/00_Rplidar/icp_opencv/icp-opencv/icp.c

CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/src/projects/00_Rplidar/icp_opencv/icp-opencv/icp.c > CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.i

CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/src/projects/00_Rplidar/icp_opencv/icp-opencv/icp.c -o CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.s

CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o.requires

CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o.provides: CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o.provides

CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o.provides.build: CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o


CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o: ../icp_opencv/third_party/kdtree/kdtree.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o   -c /home/pi/src/projects/00_Rplidar/icp_opencv/third_party/kdtree/kdtree.c

CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/src/projects/00_Rplidar/icp_opencv/third_party/kdtree/kdtree.c > CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.i

CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/src/projects/00_Rplidar/icp_opencv/third_party/kdtree/kdtree.c -o CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.s

CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o.requires

CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o.provides: CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o.provides

CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o.provides.build: CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o


CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o: ../sdk/src/rplidar_driver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o -c /home/pi/src/projects/00_Rplidar/sdk/src/rplidar_driver.cpp

CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/sdk/src/rplidar_driver.cpp > CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.i

CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/sdk/src/rplidar_driver.cpp -o CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.s

CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o.requires

CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o.provides: CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o.provides

CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o.provides.build: CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o


CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o: ../sdk/src/hal/thread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o -c /home/pi/src/projects/00_Rplidar/sdk/src/hal/thread.cpp

CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/sdk/src/hal/thread.cpp > CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.i

CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/sdk/src/hal/thread.cpp -o CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.s

CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o.requires

CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o.provides: CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o.provides

CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o.provides.build: CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o


CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o: ../sdk/src/arch/linux/net_serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o -c /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/net_serial.cpp

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/net_serial.cpp > CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.i

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/net_serial.cpp -o CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.s

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o.requires

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o.provides: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o.provides

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o.provides.build: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o


CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o: ../sdk/src/arch/linux/net_socket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o -c /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/net_socket.cpp

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/net_socket.cpp > CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.i

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/net_socket.cpp -o CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.s

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o.requires

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o.provides: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o.provides

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o.provides.build: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o


CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o: CMakeFiles/Rplidar.dir/flags.make
CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o: ../sdk/src/arch/linux/timer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o -c /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/timer.cpp

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/timer.cpp > CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.i

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/src/projects/00_Rplidar/sdk/src/arch/linux/timer.cpp -o CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.s

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o.requires:

.PHONY : CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o.requires

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o.provides: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rplidar.dir/build.make CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o.provides.build
.PHONY : CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o.provides

CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o.provides.build: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o


# Object files for target Rplidar
Rplidar_OBJECTS = \
"CMakeFiles/Rplidar.dir/main.cpp.o" \
"CMakeFiles/Rplidar.dir/lidar_driver.cpp.o" \
"CMakeFiles/Rplidar.dir/og_mapping.cpp.o" \
"CMakeFiles/Rplidar.dir/icp_interface.cpp.o" \
"CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o" \
"CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o" \
"CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o" \
"CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o" \
"CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o" \
"CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o" \
"CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o"

# External object files for target Rplidar
Rplidar_EXTERNAL_OBJECTS =

Rplidar: CMakeFiles/Rplidar.dir/main.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/lidar_driver.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/og_mapping.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/icp_interface.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o
Rplidar: CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o
Rplidar: CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o
Rplidar: CMakeFiles/Rplidar.dir/build.make
Rplidar: CMakeFiles/Rplidar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/src/projects/00_Rplidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Linking CXX executable Rplidar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Rplidar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Rplidar.dir/build: Rplidar

.PHONY : CMakeFiles/Rplidar.dir/build

CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/main.cpp.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/lidar_driver.cpp.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/og_mapping.cpp.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/icp_interface.cpp.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/icp_opencv/icp-opencv/icp.c.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/icp_opencv/third_party/kdtree/kdtree.c.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/sdk/src/rplidar_driver.cpp.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/sdk/src/hal/thread.cpp.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_serial.cpp.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/net_socket.cpp.o.requires
CMakeFiles/Rplidar.dir/requires: CMakeFiles/Rplidar.dir/sdk/src/arch/linux/timer.cpp.o.requires

.PHONY : CMakeFiles/Rplidar.dir/requires

CMakeFiles/Rplidar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Rplidar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Rplidar.dir/clean

CMakeFiles/Rplidar.dir/depend:
	cd /home/pi/src/projects/00_Rplidar/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/src/projects/00_Rplidar /home/pi/src/projects/00_Rplidar /home/pi/src/projects/00_Rplidar/build /home/pi/src/projects/00_Rplidar/build /home/pi/src/projects/00_Rplidar/build/CMakeFiles/Rplidar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Rplidar.dir/depend

