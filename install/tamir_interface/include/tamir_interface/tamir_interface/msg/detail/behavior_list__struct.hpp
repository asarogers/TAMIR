// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/msg/behavior_list.hpp"


#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__STRUCT_HPP_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'states'
#include "tamir_interface/msg/detail/behaviors__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tamir_interface__msg__BehaviorList __attribute__((deprecated))
#else
# define DEPRECATED__tamir_interface__msg__BehaviorList __declspec(deprecated)
#endif

namespace tamir_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BehaviorList_
{
  using Type = BehaviorList_<ContainerAllocator>;

  explicit BehaviorList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BehaviorList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _states_type =
    std::vector<tamir_interface::msg::Behaviors_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<tamir_interface::msg::Behaviors_<ContainerAllocator>>>;
  _states_type states;

  // setters for named parameter idiom
  Type & set__states(
    const std::vector<tamir_interface::msg::Behaviors_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<tamir_interface::msg::Behaviors_<ContainerAllocator>>> & _arg)
  {
    this->states = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tamir_interface::msg::BehaviorList_<ContainerAllocator> *;
  using ConstRawPtr =
    const tamir_interface::msg::BehaviorList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tamir_interface::msg::BehaviorList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tamir_interface::msg::BehaviorList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tamir_interface::msg::BehaviorList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tamir_interface::msg::BehaviorList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tamir_interface::msg::BehaviorList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tamir_interface::msg::BehaviorList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tamir_interface::msg::BehaviorList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tamir_interface::msg::BehaviorList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tamir_interface__msg__BehaviorList
    std::shared_ptr<tamir_interface::msg::BehaviorList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tamir_interface__msg__BehaviorList
    std::shared_ptr<tamir_interface::msg::BehaviorList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BehaviorList_ & other) const
  {
    if (this->states != other.states) {
      return false;
    }
    return true;
  }
  bool operator!=(const BehaviorList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BehaviorList_

// alias to use template instance with default allocator
using BehaviorList =
  tamir_interface::msg::BehaviorList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tamir_interface

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__STRUCT_HPP_
