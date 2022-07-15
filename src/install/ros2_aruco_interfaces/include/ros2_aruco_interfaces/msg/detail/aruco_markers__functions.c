// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_aruco_interfaces:msg/ArucoMarkers.idl
// generated code does not contain a copyright notice
#include "ros2_aruco_interfaces/msg/detail/aruco_markers__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `marker_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `poses`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
ros2_aruco_interfaces__msg__ArucoMarkers__init(ros2_aruco_interfaces__msg__ArucoMarkers * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ros2_aruco_interfaces__msg__ArucoMarkers__fini(msg);
    return false;
  }
  // marker_ids
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->marker_ids, 0)) {
    ros2_aruco_interfaces__msg__ArucoMarkers__fini(msg);
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->poses, 0)) {
    ros2_aruco_interfaces__msg__ArucoMarkers__fini(msg);
    return false;
  }
  return true;
}

void
ros2_aruco_interfaces__msg__ArucoMarkers__fini(ros2_aruco_interfaces__msg__ArucoMarkers * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // marker_ids
  rosidl_runtime_c__int64__Sequence__fini(&msg->marker_ids);
  // poses
  geometry_msgs__msg__Pose__Sequence__fini(&msg->poses);
}

ros2_aruco_interfaces__msg__ArucoMarkers *
ros2_aruco_interfaces__msg__ArucoMarkers__create()
{
  ros2_aruco_interfaces__msg__ArucoMarkers * msg = (ros2_aruco_interfaces__msg__ArucoMarkers *)malloc(sizeof(ros2_aruco_interfaces__msg__ArucoMarkers));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_aruco_interfaces__msg__ArucoMarkers));
  bool success = ros2_aruco_interfaces__msg__ArucoMarkers__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
ros2_aruco_interfaces__msg__ArucoMarkers__destroy(ros2_aruco_interfaces__msg__ArucoMarkers * msg)
{
  if (msg) {
    ros2_aruco_interfaces__msg__ArucoMarkers__fini(msg);
  }
  free(msg);
}


bool
ros2_aruco_interfaces__msg__ArucoMarkers__Sequence__init(ros2_aruco_interfaces__msg__ArucoMarkers__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  ros2_aruco_interfaces__msg__ArucoMarkers * data = NULL;
  if (size) {
    data = (ros2_aruco_interfaces__msg__ArucoMarkers *)calloc(size, sizeof(ros2_aruco_interfaces__msg__ArucoMarkers));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_aruco_interfaces__msg__ArucoMarkers__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_aruco_interfaces__msg__ArucoMarkers__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_aruco_interfaces__msg__ArucoMarkers__Sequence__fini(ros2_aruco_interfaces__msg__ArucoMarkers__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_aruco_interfaces__msg__ArucoMarkers__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_aruco_interfaces__msg__ArucoMarkers__Sequence *
ros2_aruco_interfaces__msg__ArucoMarkers__Sequence__create(size_t size)
{
  ros2_aruco_interfaces__msg__ArucoMarkers__Sequence * array = (ros2_aruco_interfaces__msg__ArucoMarkers__Sequence *)malloc(sizeof(ros2_aruco_interfaces__msg__ArucoMarkers__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = ros2_aruco_interfaces__msg__ArucoMarkers__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
ros2_aruco_interfaces__msg__ArucoMarkers__Sequence__destroy(ros2_aruco_interfaces__msg__ArucoMarkers__Sequence * array)
{
  if (array) {
    ros2_aruco_interfaces__msg__ArucoMarkers__Sequence__fini(array);
  }
  free(array);
}
