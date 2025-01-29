// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice

#include "tamir_interface/msg/detail/behavior_list__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tamir_interface
const rosidl_type_hash_t *
tamir_interface__msg__BehaviorList__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x44, 0x3a, 0xc0, 0x20, 0xbd, 0xaa, 0x42, 0xd5,
      0xb2, 0xda, 0xfd, 0xc8, 0x8b, 0x8f, 0x8f, 0xa5,
      0x75, 0x94, 0xb9, 0xad, 0x21, 0x18, 0x7e, 0xe4,
      0x0e, 0x97, 0xc7, 0xac, 0x22, 0x99, 0x3e, 0x4d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "tamir_interface/msg/detail/behaviors__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t tamir_interface__msg__Behaviors__EXPECTED_HASH = {1, {
    0xe3, 0xb1, 0x07, 0x4c, 0x3d, 0x50, 0xad, 0x94,
    0x8b, 0xe7, 0x8b, 0x13, 0xe3, 0xd2, 0xad, 0x83,
    0x2b, 0x50, 0x64, 0x90, 0xf3, 0x65, 0x54, 0xc3,
    0x8c, 0x19, 0x7c, 0xa5, 0x5b, 0x7b, 0x3f, 0xcf,
  }};
#endif

static char tamir_interface__msg__BehaviorList__TYPE_NAME[] = "tamir_interface/msg/BehaviorList";
static char tamir_interface__msg__Behaviors__TYPE_NAME[] = "tamir_interface/msg/Behaviors";

// Define type names, field names, and default values
static char tamir_interface__msg__BehaviorList__FIELD_NAME__states[] = "states";

static rosidl_runtime_c__type_description__Field tamir_interface__msg__BehaviorList__FIELDS[] = {
  {
    {tamir_interface__msg__BehaviorList__FIELD_NAME__states, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {tamir_interface__msg__Behaviors__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription tamir_interface__msg__BehaviorList__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {tamir_interface__msg__Behaviors__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tamir_interface__msg__BehaviorList__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tamir_interface__msg__BehaviorList__TYPE_NAME, 32, 32},
      {tamir_interface__msg__BehaviorList__FIELDS, 1, 1},
    },
    {tamir_interface__msg__BehaviorList__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&tamir_interface__msg__Behaviors__EXPECTED_HASH, tamir_interface__msg__Behaviors__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = tamir_interface__msg__Behaviors__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "Behaviors[] states";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tamir_interface__msg__BehaviorList__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tamir_interface__msg__BehaviorList__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 19, 19},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tamir_interface__msg__BehaviorList__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tamir_interface__msg__BehaviorList__get_individual_type_description_source(NULL),
    sources[1] = *tamir_interface__msg__Behaviors__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
