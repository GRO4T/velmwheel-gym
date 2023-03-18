// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from velmwheel_gazebo_msgs:msg/ContactState.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__TRAITS_HPP_
#define VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "velmwheel_gazebo_msgs/msg/detail/contact_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace velmwheel_gazebo_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ContactState & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    rosidl_generator_traits::value_to_yaml(msg.info, out);
    out << ", ";
  }

  // member: collision1_name
  {
    out << "collision1_name: ";
    rosidl_generator_traits::value_to_yaml(msg.collision1_name, out);
    out << ", ";
  }

  // member: collision2_name
  {
    out << "collision2_name: ";
    rosidl_generator_traits::value_to_yaml(msg.collision2_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ContactState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info: ";
    rosidl_generator_traits::value_to_yaml(msg.info, out);
    out << "\n";
  }

  // member: collision1_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision1_name: ";
    rosidl_generator_traits::value_to_yaml(msg.collision1_name, out);
    out << "\n";
  }

  // member: collision2_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision2_name: ";
    rosidl_generator_traits::value_to_yaml(msg.collision2_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ContactState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace velmwheel_gazebo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use velmwheel_gazebo_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const velmwheel_gazebo_msgs::msg::ContactState & msg,
  std::ostream & out, size_t indentation = 0)
{
  velmwheel_gazebo_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use velmwheel_gazebo_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const velmwheel_gazebo_msgs::msg::ContactState & msg)
{
  return velmwheel_gazebo_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<velmwheel_gazebo_msgs::msg::ContactState>()
{
  return "velmwheel_gazebo_msgs::msg::ContactState";
}

template<>
inline const char * name<velmwheel_gazebo_msgs::msg::ContactState>()
{
  return "velmwheel_gazebo_msgs/msg/ContactState";
}

template<>
struct has_fixed_size<velmwheel_gazebo_msgs::msg::ContactState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<velmwheel_gazebo_msgs::msg::ContactState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<velmwheel_gazebo_msgs::msg::ContactState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__TRAITS_HPP_
