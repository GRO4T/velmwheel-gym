// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from velmwheel_gazebo_msgs:srv/InertiaConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__STRUCT_HPP_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'base'
// Member 'front'
// Member 'wheel'
#include "geometry_msgs/msg/detail/inertia__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__velmwheel_gazebo_msgs__srv__InertiaConfig_Request __attribute__((deprecated))
#else
# define DEPRECATED__velmwheel_gazebo_msgs__srv__InertiaConfig_Request __declspec(deprecated)
#endif

namespace velmwheel_gazebo_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InertiaConfig_Request_
{
  using Type = InertiaConfig_Request_<ContainerAllocator>;

  explicit InertiaConfig_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : base(_init),
    front(_init),
    wheel(_init)
  {
    (void)_init;
  }

  explicit InertiaConfig_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : base(_alloc, _init),
    front(_alloc, _init),
    wheel(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _base_type =
    geometry_msgs::msg::Inertia_<ContainerAllocator>;
  _base_type base;
  using _front_type =
    geometry_msgs::msg::Inertia_<ContainerAllocator>;
  _front_type front;
  using _wheel_type =
    geometry_msgs::msg::Inertia_<ContainerAllocator>;
  _wheel_type wheel;

  // setters for named parameter idiom
  Type & set__base(
    const geometry_msgs::msg::Inertia_<ContainerAllocator> & _arg)
  {
    this->base = _arg;
    return *this;
  }
  Type & set__front(
    const geometry_msgs::msg::Inertia_<ContainerAllocator> & _arg)
  {
    this->front = _arg;
    return *this;
  }
  Type & set__wheel(
    const geometry_msgs::msg::Inertia_<ContainerAllocator> & _arg)
  {
    this->wheel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__InertiaConfig_Request
    std::shared_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__InertiaConfig_Request
    std::shared_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InertiaConfig_Request_ & other) const
  {
    if (this->base != other.base) {
      return false;
    }
    if (this->front != other.front) {
      return false;
    }
    if (this->wheel != other.wheel) {
      return false;
    }
    return true;
  }
  bool operator!=(const InertiaConfig_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InertiaConfig_Request_

// alias to use template instance with default allocator
using InertiaConfig_Request =
  velmwheel_gazebo_msgs::srv::InertiaConfig_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs


#ifndef _WIN32
# define DEPRECATED__velmwheel_gazebo_msgs__srv__InertiaConfig_Response __attribute__((deprecated))
#else
# define DEPRECATED__velmwheel_gazebo_msgs__srv__InertiaConfig_Response __declspec(deprecated)
#endif

namespace velmwheel_gazebo_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InertiaConfig_Response_
{
  using Type = InertiaConfig_Response_<ContainerAllocator>;

  explicit InertiaConfig_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit InertiaConfig_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__InertiaConfig_Response
    std::shared_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velmwheel_gazebo_msgs__srv__InertiaConfig_Response
    std::shared_ptr<velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InertiaConfig_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const InertiaConfig_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InertiaConfig_Response_

// alias to use template instance with default allocator
using InertiaConfig_Response =
  velmwheel_gazebo_msgs::srv::InertiaConfig_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs

namespace velmwheel_gazebo_msgs
{

namespace srv
{

struct InertiaConfig
{
  using Request = velmwheel_gazebo_msgs::srv::InertiaConfig_Request;
  using Response = velmwheel_gazebo_msgs::srv::InertiaConfig_Response;
};

}  // namespace srv

}  // namespace velmwheel_gazebo_msgs

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__INERTIA_CONFIG__STRUCT_HPP_
