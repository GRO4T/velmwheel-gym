// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from velmwheel_gazebo_msgs:msg/ContactState.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__STRUCT_H_
#define VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'info'
// Member 'collision1_name'
// Member 'collision2_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ContactState in the package velmwheel_gazebo_msgs.
typedef struct velmwheel_gazebo_msgs__msg__ContactState
{
  /// text info on this contact
  rosidl_runtime_c__String info;
  /// name of contact collision1
  rosidl_runtime_c__String collision1_name;
  /// name of contact collision2
  rosidl_runtime_c__String collision2_name;
} velmwheel_gazebo_msgs__msg__ContactState;

// Struct for a sequence of velmwheel_gazebo_msgs__msg__ContactState.
typedef struct velmwheel_gazebo_msgs__msg__ContactState__Sequence
{
  velmwheel_gazebo_msgs__msg__ContactState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velmwheel_gazebo_msgs__msg__ContactState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__STRUCT_H_
