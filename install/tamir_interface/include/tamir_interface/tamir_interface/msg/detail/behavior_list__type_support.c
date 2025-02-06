// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tamir_interface/msg/detail/behavior_list__rosidl_typesupport_introspection_c.h"
#include "tamir_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tamir_interface/msg/detail/behavior_list__functions.h"
#include "tamir_interface/msg/detail/behavior_list__struct.h"


// Include directives for member types
// Member `states`
#include "tamir_interface/msg/behaviors.h"
// Member `states`
#include "tamir_interface/msg/detail/behaviors__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tamir_interface__msg__BehaviorList__init(message_memory);
}

void tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_fini_function(void * message_memory)
{
  tamir_interface__msg__BehaviorList__fini(message_memory);
}

size_t tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__size_function__BehaviorList__states(
  const void * untyped_member)
{
  const tamir_interface__msg__Behaviors__Sequence * member =
    (const tamir_interface__msg__Behaviors__Sequence *)(untyped_member);
  return member->size;
}

const void * tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__get_const_function__BehaviorList__states(
  const void * untyped_member, size_t index)
{
  const tamir_interface__msg__Behaviors__Sequence * member =
    (const tamir_interface__msg__Behaviors__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__get_function__BehaviorList__states(
  void * untyped_member, size_t index)
{
  tamir_interface__msg__Behaviors__Sequence * member =
    (tamir_interface__msg__Behaviors__Sequence *)(untyped_member);
  return &member->data[index];
}

void tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__fetch_function__BehaviorList__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const tamir_interface__msg__Behaviors * item =
    ((const tamir_interface__msg__Behaviors *)
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__get_const_function__BehaviorList__states(untyped_member, index));
  tamir_interface__msg__Behaviors * value =
    (tamir_interface__msg__Behaviors *)(untyped_value);
  *value = *item;
}

void tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__assign_function__BehaviorList__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  tamir_interface__msg__Behaviors * item =
    ((tamir_interface__msg__Behaviors *)
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__get_function__BehaviorList__states(untyped_member, index));
  const tamir_interface__msg__Behaviors * value =
    (const tamir_interface__msg__Behaviors *)(untyped_value);
  *item = *value;
}

bool tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__resize_function__BehaviorList__states(
  void * untyped_member, size_t size)
{
  tamir_interface__msg__Behaviors__Sequence * member =
    (tamir_interface__msg__Behaviors__Sequence *)(untyped_member);
  tamir_interface__msg__Behaviors__Sequence__fini(member);
  return tamir_interface__msg__Behaviors__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_member_array[1] = {
  {
    "states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tamir_interface__msg__BehaviorList, states),  // bytes offset in struct
    NULL,  // default value
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__size_function__BehaviorList__states,  // size() function pointer
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__get_const_function__BehaviorList__states,  // get_const(index) function pointer
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__get_function__BehaviorList__states,  // get(index) function pointer
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__fetch_function__BehaviorList__states,  // fetch(index, &value) function pointer
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__assign_function__BehaviorList__states,  // assign(index, value) function pointer
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__resize_function__BehaviorList__states  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_members = {
  "tamir_interface__msg",  // message namespace
  "BehaviorList",  // message name
  1,  // number of fields
  sizeof(tamir_interface__msg__BehaviorList),
  false,  // has_any_key_member_
  tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_member_array,  // message members
  tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_init_function,  // function to initialize message memory (memory has to be allocated)
  tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_type_support_handle = {
  0,
  &tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_members,
  get_message_typesupport_handle_function,
  &tamir_interface__msg__BehaviorList__get_type_hash,
  &tamir_interface__msg__BehaviorList__get_type_description,
  &tamir_interface__msg__BehaviorList__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tamir_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tamir_interface, msg, BehaviorList)() {
  tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tamir_interface, msg, Behaviors)();
  if (!tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_type_support_handle.typesupport_identifier) {
    tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tamir_interface__msg__BehaviorList__rosidl_typesupport_introspection_c__BehaviorList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
