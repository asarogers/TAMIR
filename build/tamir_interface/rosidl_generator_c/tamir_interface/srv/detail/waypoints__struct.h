// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tamir_interface:srv/Waypoints.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/srv/waypoints.h"


#ifndef TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__STRUCT_H_
#define TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'waypoints'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/Waypoints in the package tamir_interface.
typedef struct tamir_interface__srv__Waypoints_Request
{
  /// an array of geometry_msgs/msg/Point objects, z is ignored
  geometry_msgs__msg__Point__Sequence waypoints;
} tamir_interface__srv__Waypoints_Request;

// Struct for a sequence of tamir_interface__srv__Waypoints_Request.
typedef struct tamir_interface__srv__Waypoints_Request__Sequence
{
  tamir_interface__srv__Waypoints_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tamir_interface__srv__Waypoints_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Waypoints in the package tamir_interface.
typedef struct tamir_interface__srv__Waypoints_Response
{
  double distance;
} tamir_interface__srv__Waypoints_Response;

// Struct for a sequence of tamir_interface__srv__Waypoints_Response.
typedef struct tamir_interface__srv__Waypoints_Response__Sequence
{
  tamir_interface__srv__Waypoints_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tamir_interface__srv__Waypoints_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  tamir_interface__srv__Waypoints_Event__request__MAX_SIZE = 1
};
// response
enum
{
  tamir_interface__srv__Waypoints_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Waypoints in the package tamir_interface.
typedef struct tamir_interface__srv__Waypoints_Event
{
  service_msgs__msg__ServiceEventInfo info;
  tamir_interface__srv__Waypoints_Request__Sequence request;
  tamir_interface__srv__Waypoints_Response__Sequence response;
} tamir_interface__srv__Waypoints_Event;

// Struct for a sequence of tamir_interface__srv__Waypoints_Event.
typedef struct tamir_interface__srv__Waypoints_Event__Sequence
{
  tamir_interface__srv__Waypoints_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tamir_interface__srv__Waypoints_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TAMIR_INTERFACE__SRV__DETAIL__WAYPOINTS__STRUCT_H_
