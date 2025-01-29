// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/msg/behavior_list.hpp"


#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__TRAITS_HPP_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tamir_interface/msg/detail/behavior_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'states'
#include "tamir_interface/msg/detail/behaviors__traits.hpp"

namespace tamir_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const BehaviorList & msg,
  std::ostream & out)
{
  out << "{";
  // member: states
  {
    if (msg.states.size() == 0) {
      out << "states: []";
    } else {
      out << "states: [";
      size_t pending_items = msg.states.size();
      for (auto item : msg.states) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BehaviorList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.states.size() == 0) {
      out << "states: []\n";
    } else {
      out << "states:\n";
      for (auto item : msg.states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BehaviorList & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tamir_interface

namespace rosidl_generator_traits
{

[[deprecated("use tamir_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tamir_interface::msg::BehaviorList & msg,
  std::ostream & out, size_t indentation = 0)
{
  tamir_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tamir_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const tamir_interface::msg::BehaviorList & msg)
{
  return tamir_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tamir_interface::msg::BehaviorList>()
{
  return "tamir_interface::msg::BehaviorList";
}

template<>
inline const char * name<tamir_interface::msg::BehaviorList>()
{
  return "tamir_interface/msg/BehaviorList";
}

template<>
struct has_fixed_size<tamir_interface::msg::BehaviorList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tamir_interface::msg::BehaviorList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tamir_interface::msg::BehaviorList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__TRAITS_HPP_
