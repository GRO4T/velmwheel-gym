// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from velmwheel_gazebo_msgs:srv/FrictionConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__STRUCT_HPP_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__velmwheel_gazebo_msgs__srv__FrictionConfig_Request __attribute__((deprecated))
#else
# define DEPRECATED__velmwheel_gazebo_msgs__srv__FrictionConfig_Request __declspec(deprecated)
#endif

namespace velmwheel_gazebo_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FrictionConfig_Request_
{
  using Type = FrictionConfig_Request_<ContainerAllocator>;

  explicit FrictionConfig_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mu1 = 0.0;
      this->mu2 = 0.0;
    }
  }

  explicit FrictionConfig_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mu1 = 0.0;
      this->mu2 = 0.0;
    }
  }

  // field types and members
  using _mu1_type =
    double;
  _mu1_type mu1;
  using _mu2_type =
    double;
  _mu2_type mu2;

  // setters for named parameter idiom
  Type & set__mu1(
    const double & _arg)
  {
    this->mu1 = _arg;
    return *this;
  }
  Type & set__mu2(
    const double & _arg)
  {
    this->mu2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__FrictionConfig_Request
    std::shared_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__FrictionConfig_Request
    std::shared_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FrictionConfig_Request_ & other) const
  {
    if (this->mu1 != other.mu1) {
      return false;
    }
    if (this->mu2 != other.mu2) {
      return false;
    }
    return true;
  }
  bool operator!=(const FrictionConfig_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FrictionConfig_Request_

// alias to use template instance with default allocator
using FrictionConfig_Request =
  velmwheel_gazebo_msgs::srv::FrictionConfig_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs


#ifndef _WIN32
# define DEPRECATED__velmwheel_gazebo_msgs__srv__FrictionConfig_Response __attribute__((deprecated))
#else
# define DEPRECATED__velmwheel_gazebo_msgs__srv__FrictionConfig_Response __declspec(deprecated)
#endif

namespace velmwheel_gazebo_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FrictionConfig_Response_
{
  using Type = FrictionConfig_Response_<ContainerAllocator>;

  explicit FrictionConfig_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit FrictionConfig_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__FrictionConfig_Response
    std::shared_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__FrictionConfig_Response
    std::shared_ptr<velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FrictionConfig_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const FrictionConfig_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FrictionConfig_Response_

// alias to use template instance with default allocator
using FrictionConfig_Response =
  velmwheel_gazebo_msgs::srv::FrictionConfig_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs

namespace velmwheel_gazebo_msgs
{

namespace srv
{

struct FrictionConfig
{
  using Request = velmwheel_gazebo_msgs::srv::FrictionConfig_Request;
  using Response = velmwheel_gazebo_msgs::srv::FrictionConfig_Response;
};

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__STRUCT_HPP_
