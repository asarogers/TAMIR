// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/msg/behavior_list.hpp"


#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__BUILDER_HPP_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tamir_interface/msg/detail/behavior_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tamir_interface
{

namespace msg
{

namespace builder
{

class Init_BehaviorList_states
{
public:
  Init_BehaviorList_states()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tamir_interface::msg::BehaviorList states(::tamir_interface::msg::BehaviorList::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tamir_interface::msg::BehaviorList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tamir_interface::msg::BehaviorList>()
{
  return tamir_interface::msg::builder::Init_BehaviorList_states();
}

}  // namespace tamir_interface

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__BUILDER_HPP_
