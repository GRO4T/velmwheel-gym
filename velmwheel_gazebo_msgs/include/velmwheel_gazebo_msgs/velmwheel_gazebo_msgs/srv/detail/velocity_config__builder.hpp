// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from velmwheel_gazebo_msgs:srv/VelocityConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__BUILDER_HPP_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "velmwheel_gazebo_msgs/srv/detail/velocity_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace velmwheel_gazebo_msgs
{

namespace srv
{

namespace builder
{

class Init_VelocityConfig_Request_rr
{
public:
  explicit Init_VelocityConfig_Request_rr(::velmwheel_gazebo_msgs::srv::VelocityConfig_Request & msg)
  : msg_(msg)
  {}
  ::velmwheel_gazebo_msgs::srv::VelocityConfig_Request rr(::velmwheel_gazebo_msgs::srv::VelocityConfig_Request::_rr_type arg)
  {
    msg_.rr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::VelocityConfig_Request msg_;
};

class Init_VelocityConfig_Request_rl
{
public:
  explicit Init_VelocityConfig_Request_rl(::velmwheel_gazebo_msgs::srv::VelocityConfig_Request & msg)
  : msg_(msg)
  {}
  Init_VelocityConfig_Request_rr rl(::velmwheel_gazebo_msgs::srv::VelocityConfig_Request::_rl_type arg)
  {
    msg_.rl = std::move(arg);
    return Init_VelocityConfig_Request_rr(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::VelocityConfig_Request msg_;
};

class Init_VelocityConfig_Request_fl
{
public:
  explicit Init_VelocityConfig_Request_fl(::velmwheel_gazebo_msgs::srv::VelocityConfig_Request & msg)
  : msg_(msg)
  {}
  Init_VelocityConfig_Request_rl fl(::velmwheel_gazebo_msgs::srv::VelocityConfig_Request::_fl_type arg)
  {
    msg_.fl = std::move(arg);
    return Init_VelocityConfig_Request_rl(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::VelocityConfig_Request msg_;
};

class Init_VelocityConfig_Request_fr
{
public:
  Init_VelocityConfig_Request_fr()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelocityConfig_Request_fl fr(::velmwheel_gazebo_msgs::srv::VelocityConfig_Request::_fr_type arg)
  {
    msg_.fr = std::move(arg);
    return Init_VelocityConfig_Request_fl(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::VelocityConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::velmwheel_gazebo_msgs::srv::VelocityConfig_Request>()
{
  return velmwheel_gazebo_msgs::srv::builder::Init_VelocityConfig_Request_fr();
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
auto build<::velmwheel_gazebo_msgs::srv::VelocityConfig_Response>()
{
  return ::velmwheel_gazebo_msgs::srv::VelocityConfig_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace velmwheel_gazebo_msgs

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__BUILDER_HPP_
