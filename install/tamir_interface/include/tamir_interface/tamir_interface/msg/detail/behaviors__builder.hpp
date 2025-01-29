// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tamir_interface:msg/Behaviors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/msg/behaviors.hpp"


#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__BUILDER_HPP_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tamir_interface/msg/detail/behaviors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tamir_interface
{

namespace msg
{

namespace builder
{

class Init_Behaviors_state
{
public:
  explicit Init_Behaviors_state(::tamir_interface::msg::Behaviors & msg)
  : msg_(msg)
  {}
  ::tamir_interface::msg::Behaviors state(::tamir_interface::msg::Behaviors::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tamir_interface::msg::Behaviors msg_;
};

class Init_Behaviors_name
{
public:
  Init_Behaviors_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Behaviors_state name(::tamir_interface::msg::Behaviors::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Behaviors_state(msg_);
  }

private:
  ::tamir_interface::msg::Behaviors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tamir_interface::msg::Behaviors>()
{
  return tamir_interface::msg::builder::Init_Behaviors_name();
}

}  // namespace tamir_interface

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__BUILDER_HPP_
