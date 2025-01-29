// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tamir_interface:srv/Waypoints.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/srv/waypoints.hpp"


#ifndef TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__TRAITS_HPP_
#define TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tamir_interface/srv/detail/waypoints__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'waypoints'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace tamir_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Waypoints_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: waypoints
  {
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []";
    } else {
      out << "waypoints: [";
      size_t pending_items = msg.waypoints.size();
      for (auto item : msg.waypoints) {
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
  const Waypoints_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []\n";
    } else {
      out << "waypoints:\n";
      for (auto item : msg.waypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Waypoints_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tamir_interface

namespace rosidl_generator_traits
{

[[deprecated("use tamir_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tamir_interface::srv::Waypoints_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  tamir_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tamir_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const tamir_interface::srv::Waypoints_Request & msg)
{
  return tamir_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tamir_interface::srv::Waypoints_Request>()
{
  return "tamir_interface::srv::Waypoints_Request";
}

template<>
inline const char * name<tamir_interface::srv::Waypoints_Request>()
{
  return "tamir_interface/srv/Waypoints_Request";
}

template<>
struct has_fixed_size<tamir_interface::srv::Waypoints_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tamir_interface::srv::Waypoints_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tamir_interface::srv::Waypoints_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace tamir_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Waypoints_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Waypoints_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Waypoints_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tamir_interface

namespace rosidl_generator_traits
{

[[deprecated("use tamir_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tamir_interface::srv::Waypoints_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  tamir_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tamir_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const tamir_interface::srv::Waypoints_Response & msg)
{
  return tamir_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tamir_interface::srv::Waypoints_Response>()
{
  return "tamir_interface::srv::Waypoints_Response";
}

template<>
inline const char * name<tamir_interface::srv::Waypoints_Response>()
{
  return "tamir_interface/srv/Waypoints_Response";
}

template<>
struct has_fixed_size<tamir_interface::srv::Waypoints_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tamir_interface::srv::Waypoints_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tamir_interface::srv::Waypoints_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace tamir_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Waypoints_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
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
  const Waypoints_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Waypoints_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tamir_interface

namespace rosidl_generator_traits
{

[[deprecated("use tamir_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tamir_interface::srv::Waypoints_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  tamir_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tamir_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const tamir_interface::srv::Waypoints_Event & msg)
{
  return tamir_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tamir_interface::srv::Waypoints_Event>()
{
  return "tamir_interface::srv::Waypoints_Event";
}

template<>
inline const char * name<tamir_interface::srv::Waypoints_Event>()
{
  return "tamir_interface/srv/Waypoints_Event";
}

template<>
struct has_fixed_size<tamir_interface::srv::Waypoints_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tamir_interface::srv::Waypoints_Event>
  : std::integral_constant<bool, has_bounded_size<service_msgs::msg::ServiceEventInfo>::value && has_bounded_size<tamir_interface::srv::Waypoints_Request>::value && has_bounded_size<tamir_interface::srv::Waypoints_Response>::value> {};

template<>
struct is_message<tamir_interface::srv::Waypoints_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tamir_interface::srv::Waypoints>()
{
  return "tamir_interface::srv::Waypoints";
}

template<>
inline const char * name<tamir_interface::srv::Waypoints>()
{
  return "tamir_interface/srv/Waypoints";
}

template<>
struct has_fixed_size<tamir_interface::srv::Waypoints>
  : std::integral_constant<
    bool,
    has_fixed_size<tamir_interface::srv::Waypoints_Request>::value &&
    has_fixed_size<tamir_interface::srv::Waypoints_Response>::value
  >
{
};

template<>
struct has_bounded_size<tamir_interface::srv::Waypoints>
  : std::integral_constant<
    bool,
    has_bounded_size<tamir_interface::srv::Waypoints_Request>::value &&
    has_bounded_size<tamir_interface::srv::Waypoints_Response>::value
  >
{
};

template<>
struct is_service<tamir_interface::srv::Waypoints>
  : std::true_type
{
};

template<>
struct is_service_request<tamir_interface::srv::Waypoints_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tamir_interface::srv::Waypoints_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__TRAITS_HPP_
