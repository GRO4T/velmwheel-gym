// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from velmwheel_gazebo_msgs:msg/ContactState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "velmwheel_gazebo_msgs/msg/detail/contact_state__rosidl_typesupport_introspection_c.h"
#include "velmwheel_gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "velmwheel_gazebo_msgs/msg/detail/contact_state__functions.h"
#include "velmwheel_gazebo_msgs/msg/detail/contact_state__struct.h"


// Include directives for member types
// Member `info`
// Member `collision1_name`
// Member `collision2_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  velmwheel_gazebo_msgs__msg__ContactState__init(message_memory);
}

void velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_fini_function(void * message_memory)
{
  velmwheel_gazebo_msgs__msg__ContactState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velmwheel_gazebo_msgs__msg__ContactState, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "collision1_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velmwheel_gazebo_msgs__msg__ContactState, collision1_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "collision2_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velmwheel_gazebo_msgs__msg__ContactState, collision2_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_message_members = {
  "velmwheel_gazebo_msgs__msg",  // message namespace
  "ContactState",  // message name
  3,  // number of fields
  sizeof(velmwheel_gazebo_msgs__msg__ContactState),
  velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_message_member_array,  // message members
  velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_init_function,  // function to initialize message memory (memory has to be allocated)
  velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_message_type_support_handle = {
  0,
  &velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_velmwheel_gazebo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, velmwheel_gazebo_msgs, msg, ContactState)() {
  if (!velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_message_type_support_handle.typesupport_identifier) {
    velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &velmwheel_gazebo_msgs__msg__ContactState__rosidl_typesupport_introspection_c__ContactState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
