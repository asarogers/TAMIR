// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tamir_interface:msg/Behaviors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/msg/behaviors.hpp"


#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__STRUCT_HPP_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tamir_interface__msg__Behaviors __attribute__((deprecated))
#else
# define DEPRECATED__tamir_interface__msg__Behaviors __declspec(deprecated)
#endif

namespace tamir_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Behaviors_
{
  using Type = Behaviors_<ContainerAllocator>;

  explicit Behaviors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->state = false;
    }
  }

  explicit Behaviors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->state = false;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _state_type =
    bool;
  _state_type state;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__state(
    const bool & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tamir_interface::msg::Behaviors_<ContainerAllocator> *;
  using ConstRawPtr =
    const tamir_interface::msg::Behaviors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tamir_interface::msg::Behaviors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tamir_interface::msg::Behaviors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tamir_interface::msg::Behaviors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tamir_interface::msg::Behaviors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tamir_interface::msg::Behaviors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tamir_interface::msg::Behaviors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tamir_interface::msg::Behaviors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tamir_interface::msg::Behaviors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tamir_interface__msg__Behaviors
    std::shared_ptr<tamir_interface::msg::Behaviors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tamir_interface__msg__Behaviors
    std::shared_ptr<tamir_interface::msg::Behaviors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Behaviors_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Behaviors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Behaviors_

// alias to use template instance with default allocator
using Behaviors =
  tamir_interface::msg::Behaviors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tamir_interface

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__STRUCT_HPP_
