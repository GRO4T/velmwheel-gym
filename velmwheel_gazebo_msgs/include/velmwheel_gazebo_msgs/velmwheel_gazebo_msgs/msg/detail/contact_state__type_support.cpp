// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from velmwheel_gazebo_msgs:msg/ContactState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "velmwheel_gazebo_msgs/msg/detail/contact_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace velmwheel_gazebo_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ContactState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) velmwheel_gazebo_msgs::msg::ContactState(_init);
}

void ContactState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<velmwheel_gazebo_msgs::msg::ContactState *>(message_memory);
  typed_message->~ContactState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ContactState_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velmwheel_gazebo_msgs::msg::ContactState, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "collision1_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velmwheel_gazebo_msgs::msg::ContactState, collision1_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "collision2_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velmwheel_gazebo_msgs::msg::ContactState, collision2_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ContactState_message_members = {
  "velmwheel_gazebo_msgs::msg",  // message namespace
  "ContactState",  // message name
  3,  // number of fields
  sizeof(velmwheel_gazebo_msgs::msg::ContactState),
  ContactState_message_member_array,  // message members
  ContactState_init_function,  // function to initialize message memory (memory has to be allocated)
  ContactState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ContactState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ContactState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace velmwheel_gazebo_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<velmwheel_gazebo_msgs::msg::ContactState>()
{
  return &::velmwheel_gazebo_msgs::msg::rosidl_typesupport_introspection_cpp::ContactState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, velmwheel_gazebo_msgs, msg, ContactState)() {
  return &::velmwheel_gazebo_msgs::msg::rosidl_typesupport_introspection_cpp::ContactState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
