// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tamir_interface:srv/Waypoints.idl
// generated code does not contain a copyright notice
#include "tamir_interface/srv/detail/waypoints__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `waypoints`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
tamir_interface__srv__Waypoints_Request__init(tamir_interface__srv__Waypoints_Request * msg)
{
  if (!msg) {
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->waypoints, 0)) {
    tamir_interface__srv__Waypoints_Request__fini(msg);
    return false;
  }
  return true;
}

void
tamir_interface__srv__Waypoints_Request__fini(tamir_interface__srv__Waypoints_Request * msg)
{
  if (!msg) {
    return;
  }
  // waypoints
  geometry_msgs__msg__Point__Sequence__fini(&msg->waypoints);
}

bool
tamir_interface__srv__Waypoints_Request__are_equal(const tamir_interface__srv__Waypoints_Request * lhs, const tamir_interface__srv__Waypoints_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->waypoints), &(rhs->waypoints)))
  {
    return false;
  }
  return true;
}

bool
tamir_interface__srv__Waypoints_Request__copy(
  const tamir_interface__srv__Waypoints_Request * input,
  tamir_interface__srv__Waypoints_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->waypoints), &(output->waypoints)))
  {
    return false;
  }
  return true;
}

tamir_interface__srv__Waypoints_Request *
tamir_interface__srv__Waypoints_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Request * msg = (tamir_interface__srv__Waypoints_Request *)allocator.allocate(sizeof(tamir_interface__srv__Waypoints_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tamir_interface__srv__Waypoints_Request));
  bool success = tamir_interface__srv__Waypoints_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tamir_interface__srv__Waypoints_Request__destroy(tamir_interface__srv__Waypoints_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tamir_interface__srv__Waypoints_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tamir_interface__srv__Waypoints_Request__Sequence__init(tamir_interface__srv__Waypoints_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Request * data = NULL;

  if (size) {
    data = (tamir_interface__srv__Waypoints_Request *)allocator.zero_allocate(size, sizeof(tamir_interface__srv__Waypoints_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tamir_interface__srv__Waypoints_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tamir_interface__srv__Waypoints_Request__fini(&data[i - 1]);
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
tamir_interface__srv__Waypoints_Request__Sequence__fini(tamir_interface__srv__Waypoints_Request__Sequence * array)
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
      tamir_interface__srv__Waypoints_Request__fini(&array->data[i]);
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

tamir_interface__srv__Waypoints_Request__Sequence *
tamir_interface__srv__Waypoints_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Request__Sequence * array = (tamir_interface__srv__Waypoints_Request__Sequence *)allocator.allocate(sizeof(tamir_interface__srv__Waypoints_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tamir_interface__srv__Waypoints_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tamir_interface__srv__Waypoints_Request__Sequence__destroy(tamir_interface__srv__Waypoints_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tamir_interface__srv__Waypoints_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tamir_interface__srv__Waypoints_Request__Sequence__are_equal(const tamir_interface__srv__Waypoints_Request__Sequence * lhs, const tamir_interface__srv__Waypoints_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tamir_interface__srv__Waypoints_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tamir_interface__srv__Waypoints_Request__Sequence__copy(
  const tamir_interface__srv__Waypoints_Request__Sequence * input,
  tamir_interface__srv__Waypoints_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tamir_interface__srv__Waypoints_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tamir_interface__srv__Waypoints_Request * data =
      (tamir_interface__srv__Waypoints_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tamir_interface__srv__Waypoints_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tamir_interface__srv__Waypoints_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tamir_interface__srv__Waypoints_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
tamir_interface__srv__Waypoints_Response__init(tamir_interface__srv__Waypoints_Response * msg)
{
  if (!msg) {
    return false;
  }
  // distance
  return true;
}

void
tamir_interface__srv__Waypoints_Response__fini(tamir_interface__srv__Waypoints_Response * msg)
{
  if (!msg) {
    return;
  }
  // distance
}

bool
tamir_interface__srv__Waypoints_Response__are_equal(const tamir_interface__srv__Waypoints_Response * lhs, const tamir_interface__srv__Waypoints_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
tamir_interface__srv__Waypoints_Response__copy(
  const tamir_interface__srv__Waypoints_Response * input,
  tamir_interface__srv__Waypoints_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // distance
  output->distance = input->distance;
  return true;
}

tamir_interface__srv__Waypoints_Response *
tamir_interface__srv__Waypoints_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Response * msg = (tamir_interface__srv__Waypoints_Response *)allocator.allocate(sizeof(tamir_interface__srv__Waypoints_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tamir_interface__srv__Waypoints_Response));
  bool success = tamir_interface__srv__Waypoints_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tamir_interface__srv__Waypoints_Response__destroy(tamir_interface__srv__Waypoints_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tamir_interface__srv__Waypoints_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tamir_interface__srv__Waypoints_Response__Sequence__init(tamir_interface__srv__Waypoints_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Response * data = NULL;

  if (size) {
    data = (tamir_interface__srv__Waypoints_Response *)allocator.zero_allocate(size, sizeof(tamir_interface__srv__Waypoints_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tamir_interface__srv__Waypoints_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tamir_interface__srv__Waypoints_Response__fini(&data[i - 1]);
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
tamir_interface__srv__Waypoints_Response__Sequence__fini(tamir_interface__srv__Waypoints_Response__Sequence * array)
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
      tamir_interface__srv__Waypoints_Response__fini(&array->data[i]);
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

tamir_interface__srv__Waypoints_Response__Sequence *
tamir_interface__srv__Waypoints_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Response__Sequence * array = (tamir_interface__srv__Waypoints_Response__Sequence *)allocator.allocate(sizeof(tamir_interface__srv__Waypoints_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tamir_interface__srv__Waypoints_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tamir_interface__srv__Waypoints_Response__Sequence__destroy(tamir_interface__srv__Waypoints_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tamir_interface__srv__Waypoints_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tamir_interface__srv__Waypoints_Response__Sequence__are_equal(const tamir_interface__srv__Waypoints_Response__Sequence * lhs, const tamir_interface__srv__Waypoints_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tamir_interface__srv__Waypoints_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tamir_interface__srv__Waypoints_Response__Sequence__copy(
  const tamir_interface__srv__Waypoints_Response__Sequence * input,
  tamir_interface__srv__Waypoints_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tamir_interface__srv__Waypoints_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tamir_interface__srv__Waypoints_Response * data =
      (tamir_interface__srv__Waypoints_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tamir_interface__srv__Waypoints_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tamir_interface__srv__Waypoints_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tamir_interface__srv__Waypoints_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "tamir_interface/srv/detail/waypoints__functions.h"

bool
tamir_interface__srv__Waypoints_Event__init(tamir_interface__srv__Waypoints_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    tamir_interface__srv__Waypoints_Event__fini(msg);
    return false;
  }
  // request
  if (!tamir_interface__srv__Waypoints_Request__Sequence__init(&msg->request, 0)) {
    tamir_interface__srv__Waypoints_Event__fini(msg);
    return false;
  }
  // response
  if (!tamir_interface__srv__Waypoints_Response__Sequence__init(&msg->response, 0)) {
    tamir_interface__srv__Waypoints_Event__fini(msg);
    return false;
  }
  return true;
}

void
tamir_interface__srv__Waypoints_Event__fini(tamir_interface__srv__Waypoints_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  tamir_interface__srv__Waypoints_Request__Sequence__fini(&msg->request);
  // response
  tamir_interface__srv__Waypoints_Response__Sequence__fini(&msg->response);
}

bool
tamir_interface__srv__Waypoints_Event__are_equal(const tamir_interface__srv__Waypoints_Event * lhs, const tamir_interface__srv__Waypoints_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!tamir_interface__srv__Waypoints_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!tamir_interface__srv__Waypoints_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
tamir_interface__srv__Waypoints_Event__copy(
  const tamir_interface__srv__Waypoints_Event * input,
  tamir_interface__srv__Waypoints_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!tamir_interface__srv__Waypoints_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!tamir_interface__srv__Waypoints_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

tamir_interface__srv__Waypoints_Event *
tamir_interface__srv__Waypoints_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Event * msg = (tamir_interface__srv__Waypoints_Event *)allocator.allocate(sizeof(tamir_interface__srv__Waypoints_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tamir_interface__srv__Waypoints_Event));
  bool success = tamir_interface__srv__Waypoints_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tamir_interface__srv__Waypoints_Event__destroy(tamir_interface__srv__Waypoints_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tamir_interface__srv__Waypoints_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tamir_interface__srv__Waypoints_Event__Sequence__init(tamir_interface__srv__Waypoints_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Event * data = NULL;

  if (size) {
    data = (tamir_interface__srv__Waypoints_Event *)allocator.zero_allocate(size, sizeof(tamir_interface__srv__Waypoints_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tamir_interface__srv__Waypoints_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tamir_interface__srv__Waypoints_Event__fini(&data[i - 1]);
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
tamir_interface__srv__Waypoints_Event__Sequence__fini(tamir_interface__srv__Waypoints_Event__Sequence * array)
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
      tamir_interface__srv__Waypoints_Event__fini(&array->data[i]);
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

tamir_interface__srv__Waypoints_Event__Sequence *
tamir_interface__srv__Waypoints_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__srv__Waypoints_Event__Sequence * array = (tamir_interface__srv__Waypoints_Event__Sequence *)allocator.allocate(sizeof(tamir_interface__srv__Waypoints_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tamir_interface__srv__Waypoints_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tamir_interface__srv__Waypoints_Event__Sequence__destroy(tamir_interface__srv__Waypoints_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tamir_interface__srv__Waypoints_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tamir_interface__srv__Waypoints_Event__Sequence__are_equal(const tamir_interface__srv__Waypoints_Event__Sequence * lhs, const tamir_interface__srv__Waypoints_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tamir_interface__srv__Waypoints_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tamir_interface__srv__Waypoints_Event__Sequence__copy(
  const tamir_interface__srv__Waypoints_Event__Sequence * input,
  tamir_interface__srv__Waypoints_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tamir_interface__srv__Waypoints_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tamir_interface__srv__Waypoints_Event * data =
      (tamir_interface__srv__Waypoints_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tamir_interface__srv__Waypoints_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tamir_interface__srv__Waypoints_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tamir_interface__srv__Waypoints_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
