// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from tamir_interface:srv/Waypoints.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "tamir_interface/srv/detail/waypoints__functions.h"
#include "tamir_interface/srv/detail/waypoints__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace tamir_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Waypoints_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Waypoints_Request_type_support_ids_t;

static const _Waypoints_Request_type_support_ids_t _Waypoints_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Waypoints_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Waypoints_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Waypoints_Request_type_support_symbol_names_t _Waypoints_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tamir_interface, srv, Waypoints_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tamir_interface, srv, Waypoints_Request)),
  }
};

typedef struct _Waypoints_Request_type_support_data_t
{
  void * data[2];
} _Waypoints_Request_type_support_data_t;

static _Waypoints_Request_type_support_data_t _Waypoints_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Waypoints_Request_message_typesupport_map = {
  2,
  "tamir_interface",
  &_Waypoints_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Waypoints_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Waypoints_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Waypoints_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Waypoints_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &tamir_interface__srv__Waypoints_Request__get_type_hash,
  &tamir_interface__srv__Waypoints_Request__get_type_description,
  &tamir_interface__srv__Waypoints_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace tamir_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tamir_interface::srv::Waypoints_Request>()
{
  return &::tamir_interface::srv::rosidl_typesupport_cpp::Waypoints_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, tamir_interface, srv, Waypoints_Request)() {
  return get_message_type_support_handle<tamir_interface::srv::Waypoints_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "tamir_interface/srv/detail/waypoints__functions.h"
// already included above
// #include "tamir_interface/srv/detail/waypoints__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace tamir_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Waypoints_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Waypoints_Response_type_support_ids_t;

static const _Waypoints_Response_type_support_ids_t _Waypoints_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Waypoints_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Waypoints_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Waypoints_Response_type_support_symbol_names_t _Waypoints_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tamir_interface, srv, Waypoints_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tamir_interface, srv, Waypoints_Response)),
  }
};

typedef struct _Waypoints_Response_type_support_data_t
{
  void * data[2];
} _Waypoints_Response_type_support_data_t;

static _Waypoints_Response_type_support_data_t _Waypoints_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Waypoints_Response_message_typesupport_map = {
  2,
  "tamir_interface",
  &_Waypoints_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Waypoints_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Waypoints_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Waypoints_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Waypoints_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &tamir_interface__srv__Waypoints_Response__get_type_hash,
  &tamir_interface__srv__Waypoints_Response__get_type_description,
  &tamir_interface__srv__Waypoints_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace tamir_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tamir_interface::srv::Waypoints_Response>()
{
  return &::tamir_interface::srv::rosidl_typesupport_cpp::Waypoints_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, tamir_interface, srv, Waypoints_Response)() {
  return get_message_type_support_handle<tamir_interface::srv::Waypoints_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "tamir_interface/srv/detail/waypoints__functions.h"
// already included above
// #include "tamir_interface/srv/detail/waypoints__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace tamir_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Waypoints_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Waypoints_Event_type_support_ids_t;

static const _Waypoints_Event_type_support_ids_t _Waypoints_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Waypoints_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Waypoints_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Waypoints_Event_type_support_symbol_names_t _Waypoints_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tamir_interface, srv, Waypoints_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tamir_interface, srv, Waypoints_Event)),
  }
};

typedef struct _Waypoints_Event_type_support_data_t
{
  void * data[2];
} _Waypoints_Event_type_support_data_t;

static _Waypoints_Event_type_support_data_t _Waypoints_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Waypoints_Event_message_typesupport_map = {
  2,
  "tamir_interface",
  &_Waypoints_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Waypoints_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Waypoints_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Waypoints_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Waypoints_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &tamir_interface__srv__Waypoints_Event__get_type_hash,
  &tamir_interface__srv__Waypoints_Event__get_type_description,
  &tamir_interface__srv__Waypoints_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace tamir_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tamir_interface::srv::Waypoints_Event>()
{
  return &::tamir_interface::srv::rosidl_typesupport_cpp::Waypoints_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, tamir_interface, srv, Waypoints_Event)() {
  return get_message_type_support_handle<tamir_interface::srv::Waypoints_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "tamir_interface/srv/detail/waypoints__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace tamir_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Waypoints_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Waypoints_type_support_ids_t;

static const _Waypoints_type_support_ids_t _Waypoints_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Waypoints_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Waypoints_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Waypoints_type_support_symbol_names_t _Waypoints_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tamir_interface, srv, Waypoints)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tamir_interface, srv, Waypoints)),
  }
};

typedef struct _Waypoints_type_support_data_t
{
  void * data[2];
} _Waypoints_type_support_data_t;

static _Waypoints_type_support_data_t _Waypoints_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Waypoints_service_typesupport_map = {
  2,
  "tamir_interface",
  &_Waypoints_service_typesupport_ids.typesupport_identifier[0],
  &_Waypoints_service_typesupport_symbol_names.symbol_name[0],
  &_Waypoints_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Waypoints_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Waypoints_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<tamir_interface::srv::Waypoints_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<tamir_interface::srv::Waypoints_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<tamir_interface::srv::Waypoints_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<tamir_interface::srv::Waypoints>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<tamir_interface::srv::Waypoints>,
  &tamir_interface__srv__Waypoints__get_type_hash,
  &tamir_interface__srv__Waypoints__get_type_description,
  &tamir_interface__srv__Waypoints__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace tamir_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<tamir_interface::srv::Waypoints>()
{
  return &::tamir_interface::srv::rosidl_typesupport_cpp::Waypoints_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, tamir_interface, srv, Waypoints)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<tamir_interface::srv::Waypoints>();
}

#ifdef __cplusplus
}
#endif
