// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from velmwheel_gazebo_msgs:srv/InertiaConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__TRAITS_HPP_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "velmwheel_gazebo_msgs/srv/detail/inertia_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'base'
// Member 'front'
// Member 'wheel'
#include "geometry_msgs/msg/detail/inertia__traits.hpp"

namespace velmwheel_gazebo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const InertiaConfig_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: base
  {
    out << "base: ";
    to_flow_style_yaml(msg.base, out);
    out << ", ";
  }

  // member: front
  {
    out << "front: ";
    to_flow_style_yaml(msg.front, out);
    out << ", ";
  }

  // member: wheel
  {
    out << "wheel: ";
    to_flow_style_yaml(msg.wheel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InertiaConfig_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: base
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base:\n";
    to_block_style_yaml(msg.base, out, indentation + 2);
  }

  // member: front
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front:\n";
    to_block_style_yaml(msg.front, out, indentation + 2);
  }

  // member: wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheel:\n";
    to_block_style_yaml(msg.wheel, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InertiaConfig_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use velmwheel_gazebo_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const velmwheel_gazebo_msgs::srv::InertiaConfig_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  velmwheel_gazebo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use velmwheel_gazebo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const velmwheel_gazebo_msgs::srv::InertiaConfig_Request & msg)
{
  return velmwheel_gazebo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<velmwheel_gazebo_msgs::srv::InertiaConfig_Request>()
{
  return "velmwheel_gazebo_msgs::srv::InertiaConfig_Request";
}

template<>
inline const char * name<velmwheel_gazebo_msgs::srv::InertiaConfig_Request>()
{
  return "velmwheel_gazebo_msgs/srv/InertiaConfig_Request";
}

template<>
struct has_fixed_size<velmwheel_gazebo_msgs::srv::InertiaConfig_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Inertia>::value> {};

template<>
struct has_bounded_size<velmwheel_gazebo_msgs::srv::InertiaConfig_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Inertia>::value> {};

template<>
struct is_message<velmwheel_gazebo_msgs::srv::InertiaConfig_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace velmwheel_gazebo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const InertiaConfig_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InertiaConfig_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InertiaConfig_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use velmwheel_gazebo_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const velmwheel_gazebo_msgs::srv::InertiaConfig_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  velmwheel_gazebo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use velmwheel_gazebo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const velmwheel_gazebo_msgs::srv::InertiaConfig_Response & msg)
{
  return velmwheel_gazebo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<velmwheel_gazebo_msgs::srv::InertiaConfig_Response>()
{
  return "velmwheel_gazebo_msgs::srv::InertiaConfig_Response";
}

template<>
inline const char * name<velmwheel_gazebo_msgs::srv::InertiaConfig_Response>()
{
  return "velmwheel_gazebo_msgs/srv/InertiaConfig_Response";
}

template<>
struct has_fixed_size<velmwheel_gazebo_msgs::srv::InertiaConfig_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<velmwheel_gazebo_msgs::srv::InertiaConfig_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<velmwheel_gazebo_msgs::srv::InertiaConfig_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<velmwheel_gazebo_msgs::srv::InertiaConfig>()
{
  return "velmwheel_gazebo_msgs::srv::InertiaConfig";
}

template<>
inline const char * name<velmwheel_gazebo_msgs::srv::InertiaConfig>()
{
  return "velmwheel_gazebo_msgs/srv/InertiaConfig";
}

template<>
struct has_fixed_size<velmwheel_gazebo_msgs::srv::InertiaConfig>
  : std::integral_constant<
    bool,
    has_fixed_size<velmwheel_gazebo_msgs::srv::InertiaConfig_Request>::value &&
    has_fixed_size<velmwheel_gazebo_msgs::srv::InertiaConfig_Response>::value
  >
{
};

template<>
struct has_bounded_size<velmwheel_gazebo_msgs::srv::InertiaConfig>
  : std::integral_constant<
    bool,
    has_bounded_size<velmwheel_gazebo_msgs::srv::InertiaConfig_Request>::value &&
    has_bounded_size<velmwheel_gazebo_msgs::srv::InertiaConfig_Response>::value
  >
{
};

template<>
struct is_service<velmwheel_gazebo_msgs::srv::InertiaConfig>
  : std::true_type
{
};

template<>
struct is_service_request<velmwheel_gazebo_msgs::srv::InertiaConfig_Request>
  : std::true_type
{
};

template<>
struct is_service_response<velmwheel_gazebo_msgs::srv::InertiaConfig_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__TRAITS_HPP_
