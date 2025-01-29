// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice
#ifndef TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "tamir_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "tamir_interface/msg/detail/behavior_list__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
bool cdr_serialize_tamir_interface__msg__BehaviorList(
  const tamir_interface__msg__BehaviorList * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
bool cdr_deserialize_tamir_interface__msg__BehaviorList(
  eprosima::fastcdr::Cdr &,
  tamir_interface__msg__BehaviorList * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
size_t get_serialized_size_tamir_interface__msg__BehaviorList(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
size_t max_serialized_size_tamir_interface__msg__BehaviorList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
bool cdr_serialize_key_tamir_interface__msg__BehaviorList(
  const tamir_interface__msg__BehaviorList * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
size_t get_serialized_size_key_tamir_interface__msg__BehaviorList(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
size_t max_serialized_size_key_tamir_interface__msg__BehaviorList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tamir_interface, msg, BehaviorList)();

#ifdef __cplusplus
}
#endif

#endif  // TAMIR_INTERFACE__MSG__DETAIL__BEHAVIOR_LIST__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
