// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tamir_interface:msg/Behaviors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/msg/behaviors.h"


#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__STRUCT_H_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Behaviors in the package tamir_interface.
typedef struct tamir_interface__msg__Behaviors
{
  rosidl_runtime_c__String name;
  bool state;
} tamir_interface__msg__Behaviors;

// Struct for a sequence of tamir_interface__msg__Behaviors.
typedef struct tamir_interface__msg__Behaviors__Sequence
{
  tamir_interface__msg__Behaviors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tamir_interface__msg__Behaviors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIORS__STRUCT_H_
