// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from velmwheel_gazebo_msgs:srv/VelocityConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__STRUCT_HPP_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__velmwheel_gazebo_msgs__srv__VelocityConfig_Request __attribute__((deprecated))
#else
# define DEPRECATED__velmwheel_gazebo_msgs__srv__VelocityConfig_Request __declspec(deprecated)
#endif

namespace velmwheel_gazebo_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct VelocityConfig_Request_
{
  using Type = VelocityConfig_Request_<ContainerAllocator>;

  explicit VelocityConfig_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fr = 0.0;
      this->fl = 0.0;
      this->rl = 0.0;
      this->rr = 0.0;
    }
  }

  explicit VelocityConfig_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fr = 0.0;
      this->fl = 0.0;
      this->rl = 0.0;
      this->rr = 0.0;
    }
  }

  // field types and members
  using _fr_type =
    double;
  _fr_type fr;
  using _fl_type =
    double;
  _fl_type fl;
  using _rl_type =
    double;
  _rl_type rl;
  using _rr_type =
    double;
  _rr_type rr;

  // setters for named parameter idiom
  Type & set__fr(
    const double & _arg)
  {
    this->fr = _arg;
    return *this;
  }
  Type & set__fl(
    const double & _arg)
  {
    this->fl = _arg;
    return *this;
  }
  Type & set__rl(
    const double & _arg)
  {
    this->rl = _arg;
    return *this;
  }
  Type & set__rr(
    const double & _arg)
  {
    this->rr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__VelocityConfig_Request
    std::shared_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__VelocityConfig_Request
    std::shared_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelocityConfig_Request_ & other) const
  {
    if (this->fr != other.fr) {
      return false;
    }
    if (this->fl != other.fl) {
      return false;
    }
    if (this->rl != other.rl) {
      return false;
    }
    if (this->rr != other.rr) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelocityConfig_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelocityConfig_Request_

// alias to use template instance with default allocator
using VelocityConfig_Request =
  velmwheel_gazebo_msgs::srv::VelocityConfig_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs


#ifndef _WIN32
# define DEPRECATED__velmwheel_gazebo_msgs__srv__VelocityConfig_Response __attribute__((deprecated))
#else
# define DEPRECATED__velmwheel_gazebo_msgs__srv__VelocityConfig_Response __declspec(deprecated)
#endif

namespace velmwheel_gazebo_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct VelocityConfig_Response_
{
  using Type = VelocityConfig_Response_<ContainerAllocator>;

  explicit VelocityConfig_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit VelocityConfig_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__VelocityConfig_Response
    std::shared_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__VelocityConfig_Response
    std::shared_ptr<velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelocityConfig_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelocityConfig_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelocityConfig_Response_

// alias to use template instance with default allocator
using VelocityConfig_Response =
  velmwheel_gazebo_msgs::srv::VelocityConfig_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs

namespace velmwheel_gazebo_msgs
{

namespace srv
{

struct VelocityConfig
{
  using Request = velmwheel_gazebo_msgs::srv::VelocityConfig_Request;
  using Response = velmwheel_gazebo_msgs::srv::VelocityConfig_Response;
};

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__VELOCITY_CONFIG__STRUCT_HPP_
