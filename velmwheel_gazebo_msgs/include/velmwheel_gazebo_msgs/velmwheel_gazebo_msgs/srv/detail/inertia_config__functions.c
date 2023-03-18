// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from velmwheel_gazebo_msgs:srv/InertiaConfig.idl
// generated code does not contain a copyright notice
#include "velmwheel_gazebo_msgs/srv/detail/inertia_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `base`
// Member `front`
// Member `wheel`
#include "geometry_msgs/msg/detail/inertia__functions.h"

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__init(velmwheel_gazebo_msgs__srv__InertiaConfig_Request * msg)
{
  if (!msg) {
    return false;
  }
  // base
  if (!geometry_msgs__msg__Inertia__init(&msg->base)) {
    velmwheel_gazebo_msgs__srv__InertiaConfig_Request__fini(msg);
    return false;
  }
  // front
  if (!geometry_msgs__msg__Inertia__init(&msg->front)) {
    velmwheel_gazebo_msgs__srv__InertiaConfig_Request__fini(msg);
    return false;
  }
  // wheel
  if (!geometry_msgs__msg__Inertia__init(&msg->wheel)) {
    velmwheel_gazebo_msgs__srv__InertiaConfig_Request__fini(msg);
    return false;
  }
  return true;
}

void
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__fini(velmwheel_gazebo_msgs__srv__InertiaConfig_Request * msg)
{
  if (!msg) {
    return;
  }
  // base
  geometry_msgs__msg__Inertia__fini(&msg->base);
  // front
  geometry_msgs__msg__Inertia__fini(&msg->front);
  // wheel
  geometry_msgs__msg__Inertia__fini(&msg->wheel);
}

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__are_equal(const velmwheel_gazebo_msgs__srv__InertiaConfig_Request * lhs, const velmwheel_gazebo_msgs__srv__InertiaConfig_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // base
  if (!geometry_msgs__msg__Inertia__are_equal(
      &(lhs->base), &(rhs->base)))
  {
    return false;
  }
  // front
  if (!geometry_msgs__msg__Inertia__are_equal(
      &(lhs->front), &(rhs->front)))
  {
    return false;
  }
  // wheel
  if (!geometry_msgs__msg__Inertia__are_equal(
      &(lhs->wheel), &(rhs->wheel)))
  {
    return false;
  }
  return true;
}

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__copy(
  const velmwheel_gazebo_msgs__srv__InertiaConfig_Request * input,
  velmwheel_gazebo_msgs__srv__InertiaConfig_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // base
  if (!geometry_msgs__msg__Inertia__copy(
      &(input->base), &(output->base)))
  {
    return false;
  }
  // front
  if (!geometry_msgs__msg__Inertia__copy(
      &(input->front), &(output->front)))
  {
    return false;
  }
  // wheel
  if (!geometry_msgs__msg__Inertia__copy(
      &(input->wheel), &(output->wheel)))
  {
    return false;
  }
  return true;
}

velmwheel_gazebo_msgs__srv__InertiaConfig_Request *
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velmwheel_gazebo_msgs__srv__InertiaConfig_Request * msg = (velmwheel_gazebo_msgs__srv__InertiaConfig_Request *)allocator.allocate(sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Request));
  bool success = velmwheel_gazebo_msgs__srv__InertiaConfig_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__destroy(velmwheel_gazebo_msgs__srv__InertiaConfig_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    velmwheel_gazebo_msgs__srv__InertiaConfig_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence__init(velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velmwheel_gazebo_msgs__srv__InertiaConfig_Request * data = NULL;

  if (size) {
    data = (velmwheel_gazebo_msgs__srv__InertiaConfig_Request *)allocator.zero_allocate(size, sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = velmwheel_gazebo_msgs__srv__InertiaConfig_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        velmwheel_gazebo_msgs__srv__InertiaConfig_Request__fini(&data[i - 1]);
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
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence__fini(velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence * array)
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
      velmwheel_gazebo_msgs__srv__InertiaConfig_Request__fini(&array->data[i]);
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

velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence *
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence * array = (velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence *)allocator.allocate(sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence__destroy(velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence__are_equal(const velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence * lhs, const velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!velmwheel_gazebo_msgs__srv__InertiaConfig_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence__copy(
  const velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence * input,
  velmwheel_gazebo_msgs__srv__InertiaConfig_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    velmwheel_gazebo_msgs__srv__InertiaConfig_Request * data =
      (velmwheel_gazebo_msgs__srv__InertiaConfig_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!velmwheel_gazebo_msgs__srv__InertiaConfig_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          velmwheel_gazebo_msgs__srv__InertiaConfig_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!velmwheel_gazebo_msgs__srv__InertiaConfig_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__init(velmwheel_gazebo_msgs__srv__InertiaConfig_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__fini(velmwheel_gazebo_msgs__srv__InertiaConfig_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__are_equal(const velmwheel_gazebo_msgs__srv__InertiaConfig_Response * lhs, const velmwheel_gazebo_msgs__srv__InertiaConfig_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__copy(
  const velmwheel_gazebo_msgs__srv__InertiaConfig_Response * input,
  velmwheel_gazebo_msgs__srv__InertiaConfig_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

velmwheel_gazebo_msgs__srv__InertiaConfig_Response *
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velmwheel_gazebo_msgs__srv__InertiaConfig_Response * msg = (velmwheel_gazebo_msgs__srv__InertiaConfig_Response *)allocator.allocate(sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Response));
  bool success = velmwheel_gazebo_msgs__srv__InertiaConfig_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__destroy(velmwheel_gazebo_msgs__srv__InertiaConfig_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    velmwheel_gazebo_msgs__srv__InertiaConfig_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence__init(velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velmwheel_gazebo_msgs__srv__InertiaConfig_Response * data = NULL;

  if (size) {
    data = (velmwheel_gazebo_msgs__srv__InertiaConfig_Response *)allocator.zero_allocate(size, sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = velmwheel_gazebo_msgs__srv__InertiaConfig_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        velmwheel_gazebo_msgs__srv__InertiaConfig_Response__fini(&data[i - 1]);
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
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence__fini(velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence * array)
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
      velmwheel_gazebo_msgs__srv__InertiaConfig_Response__fini(&array->data[i]);
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

velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence *
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence * array = (velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence *)allocator.allocate(sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence__destroy(velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence__are_equal(const velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence * lhs, const velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!velmwheel_gazebo_msgs__srv__InertiaConfig_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence__copy(
  const velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence * input,
  velmwheel_gazebo_msgs__srv__InertiaConfig_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(velmwheel_gazebo_msgs__srv__InertiaConfig_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    velmwheel_gazebo_msgs__srv__InertiaConfig_Response * data =
      (velmwheel_gazebo_msgs__srv__InertiaConfig_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!velmwheel_gazebo_msgs__srv__InertiaConfig_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          velmwheel_gazebo_msgs__srv__InertiaConfig_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!velmwheel_gazebo_msgs__srv__InertiaConfig_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
