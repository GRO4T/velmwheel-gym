// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from velmwheel_gazebo_msgs:msg/ContactState.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__BUILDER_HPP_
#define VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "velmwheel_gazebo_msgs/msg/detail/contact_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace velmwheel_gazebo_msgs
{

namespace msg
{

namespace builder
{

class Init_ContactState_collision2_name
{
public:
  explicit Init_ContactState_collision2_name(::velmwheel_gazebo_msgs::msg::ContactState & msg)
  : msg_(msg)
  {}
  ::velmwheel_gazebo_msgs::msg::ContactState collision2_name(::velmwheel_gazebo_msgs::msg::ContactState::_collision2_name_type arg)
  {
    msg_.collision2_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::msg::ContactState msg_;
};

class Init_ContactState_collision1_name
{
public:
  explicit Init_ContactState_collision1_name(::velmwheel_gazebo_msgs::msg::ContactState & msg)
  : msg_(msg)
  {}
  Init_ContactState_collision2_name collision1_name(::velmwheel_gazebo_msgs::msg::ContactState::_collision1_name_type arg)
  {
    msg_.collision1_name = std::move(arg);
    return Init_ContactState_collision2_name(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::msg::ContactState msg_;
};

class Init_ContactState_info
{
public:
  Init_ContactState_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContactState_collision1_name info(::velmwheel_gazebo_msgs::msg::ContactState::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ContactState_collision1_name(msg_);
  }

private:
  ::velmwheel_gazebo_msgs::msg::ContactState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::velmwheel_gazebo_msgs::msg::ContactState>()
{
  return velmwheel_gazebo_msgs::msg::builder::Init_ContactState_info();
}

}  // namespace velmwheel_gazebo_msgs

#endif  // VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__BUILDER_HPP_
