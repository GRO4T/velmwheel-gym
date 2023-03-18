// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from velmwheel_gazebo_msgs:srv/FrictionConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__BUILDER_HPP_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "velmwheel_gazebo_msgs/srv/detail/friction_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace velmwheel_gazebo_msgs
{

namespace srv
{

namespace builder
{

class Init_FrictionConfig_Request_mu2
{
public:
  explicit Init_FrictionConfig_Request_mu2(::velmwheel_gazebo_msgs::srv::FrictionConfig_Request & msg)
  : msg_(msg)
  {}
  ::velmwheel_gazebo_msgs::srv::FrictionConfig_Request mu2(::velmwheel_gazebo_msgs::srv::FrictionConfig_Request::_mu2_type arg)
  {
    msg_.mu2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::FrictionConfig_Request msg_;
};

class Init_FrictionConfig_Request_mu1
{
public:
  Init_FrictionConfig_Request_mu1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FrictionConfig_Request_mu2 mu1(::velmwheel_gazebo_msgs::srv::FrictionConfig_Request::_mu1_type arg)
  {
    msg_.mu1 = std::move(arg);
    return Init_FrictionConfig_Request_mu2(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::srv::FrictionConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::velmwheel_gazebo_msgs::srv::FrictionConfig_Request>()
{
  return velmwheel_gazebo_msgs::srv::builder::Init_FrictionConfig_Request_mu1();
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
auto build<::velmwheel_gazebo_msgs::srv::FrictionConfig_Response>()
{
  return ::velmwheel_gazebo_msgs::srv::FrictionConfig_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace velmwheel_gazebo_msgs

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__BUILDER_HPP_
