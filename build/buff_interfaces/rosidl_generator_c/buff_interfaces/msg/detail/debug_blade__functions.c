// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from buff_interfaces:msg/DebugBlade.idl
// generated code does not contain a copyright notice
#include "buff_interfaces/msg/detail/debug_blade__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `center`
// Member `kpt`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
buff_interfaces__msg__DebugBlade__init(buff_interfaces__msg__DebugBlade * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // width
  // height
  // label
  // prob
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    buff_interfaces__msg__DebugBlade__fini(msg);
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__init(&msg->center)) {
    buff_interfaces__msg__DebugBlade__fini(msg);
    return false;
  }
  // kpt
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->kpt, 0)) {
    buff_interfaces__msg__DebugBlade__fini(msg);
    return false;
  }
  return true;
}

void
buff_interfaces__msg__DebugBlade__fini(buff_interfaces__msg__DebugBlade * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // width
  // height
  // label
  // prob
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // center
  geometry_msgs__msg__Point__fini(&msg->center);
  // kpt
  geometry_msgs__msg__Point__Sequence__fini(&msg->kpt);
}

bool
buff_interfaces__msg__DebugBlade__are_equal(const buff_interfaces__msg__DebugBlade * lhs, const buff_interfaces__msg__DebugBlade * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // label
  if (lhs->label != rhs->label) {
    return false;
  }
  // prob
  if (lhs->prob != rhs->prob) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->center), &(rhs->center)))
  {
    return false;
  }
  // kpt
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->kpt), &(rhs->kpt)))
  {
    return false;
  }
  return true;
}

bool
buff_interfaces__msg__DebugBlade__copy(
  const buff_interfaces__msg__DebugBlade * input,
  buff_interfaces__msg__DebugBlade * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // label
  output->label = input->label;
  // prob
  output->prob = input->prob;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__copy(
      &(input->center), &(output->center)))
  {
    return false;
  }
  // kpt
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->kpt), &(output->kpt)))
  {
    return false;
  }
  return true;
}

buff_interfaces__msg__DebugBlade *
buff_interfaces__msg__DebugBlade__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__DebugBlade * msg = (buff_interfaces__msg__DebugBlade *)allocator.allocate(sizeof(buff_interfaces__msg__DebugBlade), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(buff_interfaces__msg__DebugBlade));
  bool success = buff_interfaces__msg__DebugBlade__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
buff_interfaces__msg__DebugBlade__destroy(buff_interfaces__msg__DebugBlade * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    buff_interfaces__msg__DebugBlade__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
buff_interfaces__msg__DebugBlade__Sequence__init(buff_interfaces__msg__DebugBlade__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__DebugBlade * data = NULL;

  if (size) {
    data = (buff_interfaces__msg__DebugBlade *)allocator.zero_allocate(size, sizeof(buff_interfaces__msg__DebugBlade), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = buff_interfaces__msg__DebugBlade__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        buff_interfaces__msg__DebugBlade__fini(&data[i - 1]);
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
buff_interfaces__msg__DebugBlade__Sequence__fini(buff_interfaces__msg__DebugBlade__Sequence * array)
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
      buff_interfaces__msg__DebugBlade__fini(&array->data[i]);
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

buff_interfaces__msg__DebugBlade__Sequence *
buff_interfaces__msg__DebugBlade__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__DebugBlade__Sequence * array = (buff_interfaces__msg__DebugBlade__Sequence *)allocator.allocate(sizeof(buff_interfaces__msg__DebugBlade__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = buff_interfaces__msg__DebugBlade__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
buff_interfaces__msg__DebugBlade__Sequence__destroy(buff_interfaces__msg__DebugBlade__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    buff_interfaces__msg__DebugBlade__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
buff_interfaces__msg__DebugBlade__Sequence__are_equal(const buff_interfaces__msg__DebugBlade__Sequence * lhs, const buff_interfaces__msg__DebugBlade__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!buff_interfaces__msg__DebugBlade__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
buff_interfaces__msg__DebugBlade__Sequence__copy(
  const buff_interfaces__msg__DebugBlade__Sequence * input,
  buff_interfaces__msg__DebugBlade__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(buff_interfaces__msg__DebugBlade);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    buff_interfaces__msg__DebugBlade * data =
      (buff_interfaces__msg__DebugBlade *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!buff_interfaces__msg__DebugBlade__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          buff_interfaces__msg__DebugBlade__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!buff_interfaces__msg__DebugBlade__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}