// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tamir_interface/msg/behavior_list.h"


#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__STRUCT_H_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'states'
#include "tamir_interface/msg/detail/behaviors__struct.h"

/// Struct defined in msg/BehaviorList in the package tamir_interface.
typedef struct tamir_interface__msg__BehaviorList
{
  tamir_interface__msg__Behaviors__Sequence states;
} tamir_interface__msg__BehaviorList;

// Struct for a sequence of tamir_interface__msg__BehaviorList.
typedef struct tamir_interface__msg__BehaviorList__Sequence
{
  tamir_interface__msg__BehaviorList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tamir_interface__msg__BehaviorList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__STRUCT_H_
