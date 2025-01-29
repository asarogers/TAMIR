// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tamir_interface/msg/detail/behavior_list__functions.h"
#include "tamir_interface/msg/detail/behavior_list__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tamir_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BehaviorList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tamir_interface::msg::BehaviorList(_init);
}

void BehaviorList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tamir_interface::msg::BehaviorList *>(message_memory);
  typed_message->~BehaviorList();
}

size_t size_function__BehaviorList__states(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<tamir_interface::msg::Behaviors> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BehaviorList__states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<tamir_interface::msg::Behaviors> *>(untyped_member);
  return &member[index];
}

void * get_function__BehaviorList__states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<tamir_interface::msg::Behaviors> *>(untyped_member);
  return &member[index];
}

void fetch_function__BehaviorList__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const tamir_interface::msg::Behaviors *>(
    get_const_function__BehaviorList__states(untyped_member, index));
  auto & value = *reinterpret_cast<tamir_interface::msg::Behaviors *>(untyped_value);
  value = item;
}

void assign_function__BehaviorList__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<tamir_interface::msg::Behaviors *>(
    get_function__BehaviorList__states(untyped_member, index));
  const auto & value = *reinterpret_cast<const tamir_interface::msg::Behaviors *>(untyped_value);
  item = value;
}

void resize_function__BehaviorList__states(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<tamir_interface::msg::Behaviors> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BehaviorList_message_member_array[1] = {
  {
    "states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<tamir_interface::msg::Behaviors>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tamir_interface::msg::BehaviorList, states),  // bytes offset in struct
    nullptr,  // default value
    size_function__BehaviorList__states,  // size() function pointer
    get_const_function__BehaviorList__states,  // get_const(index) function pointer
    get_function__BehaviorList__states,  // get(index) function pointer
    fetch_function__BehaviorList__states,  // fetch(index, &value) function pointer
    assign_function__BehaviorList__states,  // assign(index, value) function pointer
    resize_function__BehaviorList__states  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BehaviorList_message_members = {
  "tamir_interface::msg",  // message namespace
  "BehaviorList",  // message name
  1,  // number of fields
  sizeof(tamir_interface::msg::BehaviorList),
  false,  // has_any_key_member_
  BehaviorList_message_member_array,  // message members
  BehaviorList_init_function,  // function to initialize message memory (memory has to be allocated)
  BehaviorList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BehaviorList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BehaviorList_message_members,
  get_message_typesupport_handle_function,
  &tamir_interface__msg__BehaviorList__get_type_hash,
  &tamir_interface__msg__BehaviorList__get_type_description,
  &tamir_interface__msg__BehaviorList__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tamir_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tamir_interface::msg::BehaviorList>()
{
  return &::tamir_interface::msg::rosidl_typesupport_introspection_cpp::BehaviorList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tamir_interface, msg, BehaviorList)() {
  return &::tamir_interface::msg::rosidl_typesupport_introspection_cpp::BehaviorList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
