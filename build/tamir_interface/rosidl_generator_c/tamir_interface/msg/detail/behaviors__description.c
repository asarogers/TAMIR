// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tamir_interface:msg/Behaviors.idl
// generated code does not contain a copyright notice

#include "tamir_interface/msg/detail/behaviors__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tamir_interface
const rosidl_type_hash_t *
tamir_interface__msg__Behaviors__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe3, 0xb1, 0x07, 0x4c, 0x3d, 0x50, 0xad, 0x94,
      0x8b, 0xe7, 0x8b, 0x13, 0xe3, 0xd2, 0xad, 0x83,
      0x2b, 0x50, 0x64, 0x90, 0xf3, 0x65, 0x54, 0xc3,
      0x8c, 0x19, 0x7c, 0xa5, 0x5b, 0x7b, 0x3f, 0xcf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char tamir_interface__msg__Behaviors__TYPE_NAME[] = "tamir_interface/msg/Behaviors";

// Define type names, field names, and default values
static char tamir_interface__msg__Behaviors__FIELD_NAME__name[] = "name";
static char tamir_interface__msg__Behaviors__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field tamir_interface__msg__Behaviors__FIELDS[] = {
  {
    {tamir_interface__msg__Behaviors__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tamir_interface__msg__Behaviors__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tamir_interface__msg__Behaviors__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tamir_interface__msg__Behaviors__TYPE_NAME, 29, 29},
      {tamir_interface__msg__Behaviors__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string name\n"
  "bool state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tamir_interface__msg__Behaviors__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tamir_interface__msg__Behaviors__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 22, 22},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tamir_interface__msg__Behaviors__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tamir_interface__msg__Behaviors__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
