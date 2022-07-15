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
CMAKE_SOURCE_DIR = /home/ubuntu/nemo_ws/src/ros2_v4l2_camera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/nemo_ws/src/build/v4l2_camera

# Include any dependencies generated for this target.
include CMakeFiles/v4l2_camera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/v4l2_camera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/v4l2_camera.dir/flags.make

CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.o: CMakeFiles/v4l2_camera.dir/flags.make
CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.o: /home/ubuntu/nemo_ws/src/ros2_v4l2_camera/src/v4l2_camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/nemo_ws/src/build/v4l2_camera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.o -c /home/ubuntu/nemo_ws/src/ros2_v4l2_camera/src/v4l2_camera.cpp

CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/nemo_ws/src/ros2_v4l2_camera/src/v4l2_camera.cpp > CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.i

CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/nemo_ws/src/ros2_v4l2_camera/src/v4l2_camera.cpp -o CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.s

CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.o: CMakeFiles/v4l2_camera.dir/flags.make
CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.o: /home/ubuntu/nemo_ws/src/ros2_v4l2_camera/src/v4l2_camera_device.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/nemo_ws/src/build/v4l2_camera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.o -c /home/ubuntu/nemo_ws/src/ros2_v4l2_camera/src/v4l2_camera_device.cpp

CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/nemo_ws/src/ros2_v4l2_camera/src/v4l2_camera_device.cpp > CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.i

CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/nemo_ws/src/ros2_v4l2_camera/src/v4l2_camera_device.cpp -o CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.s

# Object files for target v4l2_camera
v4l2_camera_OBJECTS = \
"CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.o" \
"CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.o"

# External object files for target v4l2_camera
v4l2_camera_EXTERNAL_OBJECTS =

libv4l2_camera.so: CMakeFiles/v4l2_camera.dir/src/v4l2_camera.cpp.o
libv4l2_camera.so: CMakeFiles/v4l2_camera.dir/src/v4l2_camera_device.cpp.o
libv4l2_camera.so: CMakeFiles/v4l2_camera.dir/build.make
libv4l2_camera.so: /opt/ros/foxy/lib/libcomponent_manager.so
libv4l2_camera.so: /home/ubuntu/nemo_ws/src/install/image_transport/lib/libimage_transport.so
libv4l2_camera.so: /opt/ros/foxy/lib/libmessage_filters.so
libv4l2_camera.so: /opt/ros/foxy/lib/librclcpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_runtime_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libclass_loader.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcutils.so
libv4l2_camera.so: /opt/ros/foxy/lib/libament_index_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libclass_loader.so
libv4l2_camera.so: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
libv4l2_camera.so: /opt/ros/foxy/lib/libament_index_cpp.so
libv4l2_camera.so: /home/ubuntu/nemo_ws/src/install/camera_calibration_parsers/lib/libcamera_calibration_parsers.so
libv4l2_camera.so: /opt/ros/foxy/opt/yaml_cpp_vendor/lib/libyaml-cpp.so.0.6.2
libv4l2_camera.so: /opt/ros/foxy/lib/librclcpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/liblibstatistics_collector.so
libv4l2_camera.so: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcl.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libtracetools.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcpputils.so
libv4l2_camera.so: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
libv4l2_camera.so: /home/ubuntu/nemo_ws/src/install/camera_info_manager/lib/libcamera_info_manager.so
libv4l2_camera.so: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librmw_implementation.so
libv4l2_camera.so: /opt/ros/foxy/lib/librmw.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcl_logging_spdlog.so
libv4l2_camera.so: /usr/lib/aarch64-linux-gnu/libspdlog.so.1.5.0
libv4l2_camera.so: /opt/ros/foxy/lib/libyaml.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/aarch64-linux-gnu/libconsole_bridge.so.1.0
libv4l2_camera.so: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_typesupport_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcpputils.so
libv4l2_camera.so: /opt/ros/foxy/lib/librosidl_runtime_c.so
libv4l2_camera.so: /opt/ros/foxy/lib/librcutils.so
libv4l2_camera.so: CMakeFiles/v4l2_camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/nemo_ws/src/build/v4l2_camera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libv4l2_camera.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/v4l2_camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/v4l2_camera.dir/build: libv4l2_camera.so

.PHONY : CMakeFiles/v4l2_camera.dir/build

CMakeFiles/v4l2_camera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/v4l2_camera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/v4l2_camera.dir/clean

CMakeFiles/v4l2_camera.dir/depend:
	cd /home/ubuntu/nemo_ws/src/build/v4l2_camera && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/nemo_ws/src/ros2_v4l2_camera /home/ubuntu/nemo_ws/src/ros2_v4l2_camera /home/ubuntu/nemo_ws/src/build/v4l2_camera /home/ubuntu/nemo_ws/src/build/v4l2_camera /home/ubuntu/nemo_ws/src/build/v4l2_camera/CMakeFiles/v4l2_camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/v4l2_camera.dir/depend

