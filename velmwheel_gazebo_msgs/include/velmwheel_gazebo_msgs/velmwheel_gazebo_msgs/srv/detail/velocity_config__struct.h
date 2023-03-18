// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from velmwheel_gazebo_msgs:srv/VelocityConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__STRUCT_H_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/VelocityConfig in the package velmwheel_gazebo_msgs.
typedef struct velmwheel_gazebo_msgs__srv__VelocityConfig_Request
{
  /// Setpoint angular velocity of the front right wheel in
  double fr;
  /// Setpoint angular velocity of the front left wheel in
  double fl;
  /// Setpoint angular velocity of the rear right wheel in
  double rl;
  /// Setpoint angular velocity of the rear left wheel in
  double rr;
} velmwheel_gazebo_msgs__srv__VelocityConfig_Request;

// Struct for a sequence of velmwheel_gazebo_msgs__srv__VelocityConfig_Request.
typedef struct velmwheel_gazebo_msgs__srv__VelocityConfig_Request__Sequence
{
  velmwheel_gazebo_msgs__srv__VelocityConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velmwheel_gazebo_msgs__srv__VelocityConfig_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/VelocityConfig in the package velmwheel_gazebo_msgs.
typedef struct velmwheel_gazebo_msgs__srv__VelocityConfig_Response
{
  uint8_t structure_needs_at_least_one_member;
} velmwheel_gazebo_msgs__srv__VelocityConfig_Response;

// Struct for a sequence of velmwheel_gazebo_msgs__srv__VelocityConfig_Response.
typedef struct velmwheel_gazebo_msgs__srv__VelocityConfig_Response__Sequence
{
  velmwheel_gazebo_msgs__srv__VelocityConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velmwheel_gazebo_msgs__srv__VelocityConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__STRUCT_H_
