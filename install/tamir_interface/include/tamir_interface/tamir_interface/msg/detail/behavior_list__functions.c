// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice
#include "tamir_interface/msg/detail/behavior_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `states`
#include "tamir_interface/msg/detail/behaviors__functions.h"

bool
tamir_interface__msg__BehaviorList__init(tamir_interface__msg__BehaviorList * msg)
{
  if (!msg) {
    return false;
  }
  // states
  if (!tamir_interface__msg__Behaviors__Sequence__init(&msg->states, 0)) {
    tamir_interface__msg__BehaviorList__fini(msg);
    return false;
  }
  return true;
}

void
tamir_interface__msg__BehaviorList__fini(tamir_interface__msg__BehaviorList * msg)
{
  if (!msg) {
    return;
  }
  // states
  tamir_interface__msg__Behaviors__Sequence__fini(&msg->states);
}

bool
tamir_interface__msg__BehaviorList__are_equal(const tamir_interface__msg__BehaviorList * lhs, const tamir_interface__msg__BehaviorList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // states
  if (!tamir_interface__msg__Behaviors__Sequence__are_equal(
      &(lhs->states), &(rhs->states)))
  {
    return false;
  }
  return true;
}

bool
tamir_interface__msg__BehaviorList__copy(
  const tamir_interface__msg__BehaviorList * input,
  tamir_interface__msg__BehaviorList * output)
{
  if (!input || !output) {
    return false;
  }
  // states
  if (!tamir_interface__msg__Behaviors__Sequence__copy(
      &(input->states), &(output->states)))
  {
    return false;
  }
  return true;
}

tamir_interface__msg__BehaviorList *
tamir_interface__msg__BehaviorList__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__msg__BehaviorList * msg = (tamir_interface__msg__BehaviorList *)allocator.allocate(sizeof(tamir_interface__msg__BehaviorList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tamir_interface__msg__BehaviorList));
  bool success = tamir_interface__msg__BehaviorList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tamir_interface__msg__BehaviorList__destroy(tamir_interface__msg__BehaviorList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tamir_interface__msg__BehaviorList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tamir_interface__msg__BehaviorList__Sequence__init(tamir_interface__msg__BehaviorList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__msg__BehaviorList * data = NULL;

  if (size) {
    data = (tamir_interface__msg__BehaviorList *)allocator.zero_allocate(size, sizeof(tamir_interface__msg__BehaviorList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tamir_interface__msg__BehaviorList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tamir_interface__msg__BehaviorList__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tamir_interface__msg__BehaviorList__Sequence__fini(tamir_interface__msg__BehaviorList__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tamir_interface__msg__BehaviorList__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tamir_interface__msg__BehaviorList__Sequence *
tamir_interface__msg__BehaviorList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__msg__BehaviorList__Sequence * array = (tamir_interface__msg__BehaviorList__Sequence *)allocator.allocate(sizeof(tamir_interface__msg__BehaviorList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tamir_interface__msg__BehaviorList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tamir_interface__msg__BehaviorList__Sequence__destroy(tamir_interface__msg__BehaviorList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tamir_interface__msg__BehaviorList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tamir_interface__msg__BehaviorList__Sequence__are_equal(const tamir_interface__msg__BehaviorList__Sequence * lhs, const tamir_interface__msg__BehaviorList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tamir_interface__msg__BehaviorList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tamir_interface__msg__BehaviorList__Sequence__copy(
  const tamir_interface__msg__BehaviorList__Sequence * input,
  tamir_interface__msg__BehaviorList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tamir_interface__msg__BehaviorList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tamir_interface__msg__BehaviorList * data =
      (tamir_interface__msg__BehaviorList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tamir_interface__msg__BehaviorList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tamir_interface__msg__BehaviorList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tamir_interface__msg__BehaviorList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
