// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from velmwheel_gazebo_msgs:msg/ContactState.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__STRUCT_HPP_
#define VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__velmwheel_gazebo_msgs__msg__ContactState __attribute__((deprecated))
#else
# define DEPRECATED__velmwheel_gazebo_msgs__msg__ContactState __declspec(deprecated)
#endif

namespace velmwheel_gazebo_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ContactState_
{
  using Type = ContactState_<ContainerAllocator>;

  explicit ContactState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->info = "";
      this->collision1_name = "";
      this->collision2_name = "";
    }
  }

  explicit ContactState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc),
    collision1_name(_alloc),
    collision2_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->info = "";
      this->collision1_name = "";
      this->collision2_name = "";
    }
  }

  // field types and members
  using _info_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _info_type info;
  using _collision1_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _collision1_name_type collision1_name;
  using _collision2_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _collision2_name_type collision2_name;

  // setters for named parameter idiom
  Type & set__info(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__collision1_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->collision1_name = _arg;
    return *this;
  }
  Type & set__collision2_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->collision2_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator> *;
  using ConstRawPtr =
    const velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velmwheel_gazebo_msgs__msg__ContactState
    std::shared_ptr<velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velmwheel_gazebo_msgs__msg__ContactState
    std::shared_ptr<velmwheel_gazebo_msgs::msg::ContactState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContactState_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->collision1_name != other.collision1_name) {
      return false;
    }
    if (this->collision2_name != other.collision2_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContactState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContactState_

// alias to use template instance with default allocator
using ContactState =
  velmwheel_gazebo_msgs::msg::ContactState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace velmwheel_gazebo_msgs

#endif  // VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__STRUCT_HPP_
