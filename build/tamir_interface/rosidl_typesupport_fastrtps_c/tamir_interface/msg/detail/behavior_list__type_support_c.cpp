// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice
#include "tamir_interface/msg/detail/behavior_list__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "tamir_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "tamir_interface/msg/detail/behavior_list__struct.h"
#include "tamir_interface/msg/detail/behavior_list__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "tamir_interface/msg/detail/behaviors__functions.h"  // states

// forward declare type support functions

bool cdr_serialize_tamir_interface__msg__Behaviors(
  const tamir_interface__msg__Behaviors * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_tamir_interface__msg__Behaviors(
  eprosima::fastcdr::Cdr & cdr,
  tamir_interface__msg__Behaviors * ros_message);

size_t get_serialized_size_tamir_interface__msg__Behaviors(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_tamir_interface__msg__Behaviors(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_tamir_interface__msg__Behaviors(
  const tamir_interface__msg__Behaviors * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_tamir_interface__msg__Behaviors(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_tamir_interface__msg__Behaviors(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tamir_interface, msg, Behaviors)();


using _BehaviorList__ros_msg_type = tamir_interface__msg__BehaviorList;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
bool cdr_serialize_tamir_interface__msg__BehaviorList(
  const tamir_interface__msg__BehaviorList * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: states
  {
    size_t size = ros_message->states.size;
    auto array_ptr = ros_message->states.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_tamir_interface__msg__Behaviors(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
bool cdr_deserialize_tamir_interface__msg__BehaviorList(
  eprosima::fastcdr::Cdr & cdr,
  tamir_interface__msg__BehaviorList * ros_message)
{
  // Field name: states
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->states.data) {
      tamir_interface__msg__Behaviors__Sequence__fini(&ros_message->states);
    }
    if (!tamir_interface__msg__Behaviors__Sequence__init(&ros_message->states, size)) {
      fprintf(stderr, "failed to create array for field 'states'");
      return false;
    }
    auto array_ptr = ros_message->states.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_tamir_interface__msg__Behaviors(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
size_t get_serialized_size_tamir_interface__msg__BehaviorList(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BehaviorList__ros_msg_type * ros_message = static_cast<const _BehaviorList__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: states
  {
    size_t array_size = ros_message->states.size;
    auto array_ptr = ros_message->states.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_tamir_interface__msg__Behaviors(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
size_t max_serialized_size_tamir_interface__msg__BehaviorList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: states
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_tamir_interface__msg__Behaviors(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = tamir_interface__msg__BehaviorList;
    is_plain =
      (
      offsetof(DataType, states) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
bool cdr_serialize_key_tamir_interface__msg__BehaviorList(
  const tamir_interface__msg__BehaviorList * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: states
  {
    size_t size = ros_message->states.size;
    auto array_ptr = ros_message->states.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_tamir_interface__msg__Behaviors(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
size_t get_serialized_size_key_tamir_interface__msg__BehaviorList(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BehaviorList__ros_msg_type * ros_message = static_cast<const _BehaviorList__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: states
  {
    size_t array_size = ros_message->states.size;
    auto array_ptr = ros_message->states.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_tamir_interface__msg__Behaviors(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tamir_interface
size_t max_serialized_size_key_tamir_interface__msg__BehaviorList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: states
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_tamir_interface__msg__Behaviors(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = tamir_interface__msg__BehaviorList;
    is_plain =
      (
      offsetof(DataType, states) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _BehaviorList__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const tamir_interface__msg__BehaviorList * ros_message = static_cast<const tamir_interface__msg__BehaviorList *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_tamir_interface__msg__BehaviorList(ros_message, cdr);
}

static bool _BehaviorList__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  tamir_interface__msg__BehaviorList * ros_message = static_cast<tamir_interface__msg__BehaviorList *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_tamir_interface__msg__BehaviorList(cdr, ros_message);
}

static uint32_t _BehaviorList__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_tamir_interface__msg__BehaviorList(
      untyped_ros_message, 0));
}

static size_t _BehaviorList__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_tamir_interface__msg__BehaviorList(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BehaviorList = {
  "tamir_interface::msg",
  "BehaviorList",
  _BehaviorList__cdr_serialize,
  _BehaviorList__cdr_deserialize,
  _BehaviorList__get_serialized_size,
  _BehaviorList__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _BehaviorList__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BehaviorList,
  get_message_typesupport_handle_function,
  &tamir_interface__msg__BehaviorList__get_type_hash,
  &tamir_interface__msg__BehaviorList__get_type_description,
  &tamir_interface__msg__BehaviorList__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tamir_interface, msg, BehaviorList)() {
  return &_BehaviorList__type_support;
}

#if defined(__cplusplus)
}
#endif
