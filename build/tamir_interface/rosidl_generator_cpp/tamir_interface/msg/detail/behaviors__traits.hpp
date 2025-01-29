// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tamir_interface:msg/Behaviors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/msg/behaviors.hpp"


#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__TRAITS_HPP_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tamir_interface/msg/detail/behaviors__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tamir_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Behaviors & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Behaviors & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Behaviors & msg, bool use_flow_style = false)
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
  const tamir_interface::msg::Behaviors & msg,
  std::ostream & out, size_t indentation = 0)
{
  tamir_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tamir_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const tamir_interface::msg::Behaviors & msg)
{
  return tamir_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tamir_interface::msg::Behaviors>()
{
  return "tamir_interface::msg::Behaviors";
}

template<>
inline const char * name<tamir_interface::msg::Behaviors>()
{
  return "tamir_interface/msg/Behaviors";
}

template<>
struct has_fixed_size<tamir_interface::msg::Behaviors>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tamir_interface::msg::Behaviors>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tamir_interface::msg::Behaviors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__TRAITS_HPP_
