// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from velmwheel_gazebo_msgs:srv/InertiaConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__BUILDER_HPP_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "velmwheel_gazebo_msgs/srv/detail/inertia_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace velmwheel_gazebo_msgs
{

namespace srv
{

namespace builder
{

class Init_InertiaConfig_Request_wheel
{
public:
  explicit Init_InertiaConfig_Request_wheel(::velmwheel_gazebo_msgs::srv::InertiaConfig_Request & msg)
  : msg_(msg)
  {}
  ::velmwheel_gazebo_msgs::srv::InertiaConfig_Request wheel(::velmwheel_gazebo_msgs::srv::InertiaConfig_Request::_wheel_type arg)
  {
    msg_.wheel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::InertiaConfig_Request msg_;
};

class Init_InertiaConfig_Request_front
{
public:
  explicit Init_InertiaConfig_Request_front(::velmwheel_gazebo_msgs::srv::InertiaConfig_Request & msg)
  : msg_(msg)
  {}
  Init_InertiaConfig_Request_wheel front(::velmwheel_gazebo_msgs::srv::InertiaConfig_Request::_front_type arg)
  {
    msg_.front = std::move(arg);
    return Init_InertiaConfig_Request_wheel(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::InertiaConfig_Request msg_;
};

class Init_InertiaConfig_Request_base
{
public:
  Init_InertiaConfig_Request_base()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InertiaConfig_Request_front base(::velmwheel_gazebo_msgs::srv::InertiaConfig_Request::_base_type arg)
  {
    msg_.base = std::move(arg);
    return Init_InertiaConfig_Request_front(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::InertiaConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::velmwheel_gazebo_msgs::srv::InertiaConfig_Request>()
{
  return velmwheel_gazebo_msgs::srv::builder::Init_InertiaConfig_Request_base();
}

}  // namespace velmwheel_gazebo_msgs


namespace velmwheel_gazebo_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::velmwheel_gazebo_msgs::srv::InertiaConfig_Response>()
{
  return ::velmwheel_gazebo_msgs::srv::InertiaConfig_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace velmwheel_gazebo_msgs

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__BUILDER_HPP_
