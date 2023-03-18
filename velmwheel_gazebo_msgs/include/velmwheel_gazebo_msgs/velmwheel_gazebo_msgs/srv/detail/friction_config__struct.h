// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from velmwheel_gazebo_msgs:srv/FrictionConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__STRUCT_H_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/FrictionConfig in the package velmwheel_gazebo_msgs.
typedef struct velmwheel_gazebo_msgs__srv__FrictionConfig_Request
{
  /// Friction value of the vector parallel to the roll's axis (vector perpendicular to the rotation axis);
  double mu1;
  /// Friction value of the vector perpendicular to the roll's axis (vector of the rotation axis);
  double mu2;
} velmwheel_gazebo_msgs__srv__FrictionConfig_Request;

// Struct for a sequence of velmwheel_gazebo_msgs__srv__FrictionConfig_Request.
typedef struct velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence
{
  velmwheel_gazebo_msgs__srv__FrictionConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/FrictionConfig in the package velmwheel_gazebo_msgs.
typedef struct velmwheel_gazebo_msgs__srv__FrictionConfig_Response
{
  uint8_t structure_needs_at_least_one_member;
} velmwheel_gazebo_msgs__srv__FrictionConfig_Response;

// Struct for a sequence of velmwheel_gazebo_msgs__srv__FrictionConfig_Response.
typedef struct velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence
{
  velmwheel_gazebo_msgs__srv__FrictionConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__STRUCT_H_
