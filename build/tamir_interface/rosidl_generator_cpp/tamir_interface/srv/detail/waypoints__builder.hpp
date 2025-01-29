// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tamir_interface:srv/Waypoints.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/srv/waypoints.hpp"


#ifndef TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__BUILDER_HPP_
#define TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tamir_interface/srv/detail/waypoints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tamir_interface
{

namespace srv
{

namespace builder
{

class Init_Waypoints_Request_waypoints
{
public:
  Init_Waypoints_Request_waypoints()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tamir_interface::srv::Waypoints_Request waypoints(::tamir_interface::srv::Waypoints_Request::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tamir_interface::srv::Waypoints_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tamir_interface::srv::Waypoints_Request>()
{
  return tamir_interface::srv::builder::Init_Waypoints_Request_waypoints();
}

}  // namespace tamir_interface


namespace tamir_interface
{

namespace srv
{

namespace builder
{

class Init_Waypoints_Response_distance
{
public:
  Init_Waypoints_Response_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tamir_interface::srv::Waypoints_Response distance(::tamir_interface::srv::Waypoints_Response::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tamir_interface::srv::Waypoints_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tamir_interface::srv::Waypoints_Response>()
{
  return tamir_interface::srv::builder::Init_Waypoints_Response_distance();
}

}  // namespace tamir_interface


namespace tamir_interface
{

namespace srv
{

namespace builder
{

class Init_Waypoints_Event_response
{
public:
  explicit Init_Waypoints_Event_response(::tamir_interface::srv::Waypoints_Event & msg)
  : msg_(msg)
  {}
  ::tamir_interface::srv::Waypoints_Event response(::tamir_interface::srv::Waypoints_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tamir_interface::srv::Waypoints_Event msg_;
};

class Init_Waypoints_Event_request
{
public:
  explicit Init_Waypoints_Event_request(::tamir_interface::srv::Waypoints_Event & msg)
  : msg_(msg)
  {}
  Init_Waypoints_Event_response request(::tamir_interface::srv::Waypoints_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Waypoints_Event_response(msg_);
  }

private:
  ::tamir_interface::srv::Waypoints_Event msg_;
};

class Init_Waypoints_Event_info
{
public:
  Init_Waypoints_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Waypoints_Event_request info(::tamir_interface::srv::Waypoints_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Waypoints_Event_request(msg_);
  }

private:
  ::tamir_interface::srv::Waypoints_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tamir_interface::srv::Waypoints_Event>()
{
  return tamir_interface::srv::builder::Init_Waypoints_Event_info();
}

}  // namespace tamir_interface

#endif  // TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__BUILDER_HPP_
