// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from velmwheel_gazebo_msgs:srv/InertiaConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__STRUCT_H_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'base'
// Member 'front'
// Member 'wheel'
#include "geometry_msgs/msg/detail/inertia__struct.h"

/// Struct defined in srv/InertiaConfig in the package velmwheel_gazebo_msgs.
typedef struct velmwheel_gazebo_msgs__srv__InertiaConfig_Request
{
  /// Inertia of the robot's bases
  geometry_msgs__msg__Inertia base;
  /// Inertia of the robot's front part
  geometry_msgs__msg__Inertia front;
  /// Inertia of a robot's wheel
  geometry_msgs__msg__Inertia wheel;
} velmwheel_gazebo_msgs__srv__InertiaConfig_Request;

// Struct for a sequence of velmwheel_gazebo_msgs__srv__InertiaConfig_Request.
typedef struct velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence
{
  velmwheel_gazebo_msgs__srv__InertiaConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/InertiaConfig in the package velmwheel_gazebo_msgs.
typedef struct velmwheel_gazebo_msgs__srv__InertiaConfig_Response
{
  uint8_t structure_needs_at_least_one_member;
} velmwheel_gazebo_msgs__srv__InertiaConfig_Response;

// Struct for a sequence of velmwheel_gazebo_msgs__srv__InertiaConfig_Response.
typedef struct velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence
{
  velmwheel_gazebo_msgs__srv__InertiaConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__STRUCT_H_
