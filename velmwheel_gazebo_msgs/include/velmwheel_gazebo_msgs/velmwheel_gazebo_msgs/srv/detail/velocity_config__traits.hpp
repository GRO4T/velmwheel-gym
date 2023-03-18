// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from velmwheel_gazebo_msgs:srv/VelocityConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__TRAITS_HPP_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "velmwheel_gazebo_msgs/srv/detail/velocity_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace velmwheel_gazebo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VelocityConfig_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: fr
  {
    out << "fr: ";
    rosidl_generator_traits::value_to_yaml(msg.fr, out);
    out << ", ";
  }

  // member: fl
  {
    out << "fl: ";
    rosidl_generator_traits::value_to_yaml(msg.fl, out);
    out << ", ";
  }

  // member: rl
  {
    out << "rl: ";
    rosidl_generator_traits::value_to_yaml(msg.rl, out);
    out << ", ";
  }

  // member: rr
  {
    out << "rr: ";
    rosidl_generator_traits::value_to_yaml(msg.rr, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VelocityConfig_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fr: ";
    rosidl_generator_traits::value_to_yaml(msg.fr, out);
    out << "\n";
  }

  // member: fl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fl: ";
    rosidl_generator_traits::value_to_yaml(msg.fl, out);
    out << "\n";
  }

  // member: rl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rl: ";
    rosidl_generator_traits::value_to_yaml(msg.rl, out);
    out << "\n";
  }

  // member: rr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rr: ";
    rosidl_generator_traits::value_to_yaml(msg.rr, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VelocityConfig_Request & msg, bool use_flow_style = false)
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
  const velmwheel_gazebo_msgs::srv::VelocityConfig_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  velmwheel_gazebo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use velmwheel_gazebo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const velmwheel_gazebo_msgs::srv::VelocityConfig_Request & msg)
{
  return velmwheel_gazebo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<velmwheel_gazebo_msgs::srv::VelocityConfig_Request>()
{
  return "velmwheel_gazebo_msgs::srv::VelocityConfig_Request";
}

template<>
inline const char * name<velmwheel_gazebo_msgs::srv::VelocityConfig_Request>()
{
  return "velmwheel_gazebo_msgs/srv/VelocityConfig_Request";
}

template<>
struct has_fixed_size<velmwheel_gazebo_msgs::srv::VelocityConfig_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<velmwheel_gazebo_msgs::srv::VelocityConfig_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<velmwheel_gazebo_msgs::srv::VelocityConfig_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace velmwheel_gazebo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VelocityConfig_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VelocityConfig_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VelocityConfig_Response & msg, bool use_flow_style = false)
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
  const velmwheel_gazebo_msgs::srv::VelocityConfig_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  velmwheel_gazebo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use velmwheel_gazebo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const velmwheel_gazebo_msgs::srv::VelocityConfig_Response & msg)
{
  return velmwheel_gazebo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<velmwheel_gazebo_msgs::srv::VelocityConfig_Response>()
{
  return "velmwheel_gazebo_msgs::srv::VelocityConfig_Response";
}

template<>
inline const char * name<velmwheel_gazebo_msgs::srv::VelocityConfig_Response>()
{
  return "velmwheel_gazebo_msgs/srv/VelocityConfig_Response";
}

template<>
struct has_fixed_size<velmwheel_gazebo_msgs::srv::VelocityConfig_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<velmwheel_gazebo_msgs::srv::VelocityConfig_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<velmwheel_gazebo_msgs::srv::VelocityConfig_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<velmwheel_gazebo_msgs::srv::VelocityConfig>()
{
  return "velmwheel_gazebo_msgs::srv::VelocityConfig";
}

template<>
inline const char * name<velmwheel_gazebo_msgs::srv::VelocityConfig>()
{
  return "velmwheel_gazebo_msgs/srv/VelocityConfig";
}

template<>
struct has_fixed_size<velmwheel_gazebo_msgs::srv::VelocityConfig>
  : std::integral_constant<
    bool,
    has_fixed_size<velmwheel_gazebo_msgs::srv::VelocityConfig_Request>::value &&
    has_fixed_size<velmwheel_gazebo_msgs::srv::VelocityConfig_Response>::value
  >
{
};

template<>
struct has_bounded_size<velmwheel_gazebo_msgs::srv::VelocityConfig>
  : std::integral_constant<
    bool,
    has_bounded_size<velmwheel_gazebo_msgs::srv::VelocityConfig_Request>::value &&
    has_bounded_size<velmwheel_gazebo_msgs::srv::VelocityConfig_Response>::value
  >
{
};

template<>
struct is_service<velmwheel_gazebo_msgs::srv::VelocityConfig>
  : std::true_type
{
};

template<>
struct is_service_request<velmwheel_gazebo_msgs::srv::VelocityConfig_Request>
  : std::true_type
{
};

template<>
struct is_service_response<velmwheel_gazebo_msgs::srv::VelocityConfig_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__TRAITS_HPP_
