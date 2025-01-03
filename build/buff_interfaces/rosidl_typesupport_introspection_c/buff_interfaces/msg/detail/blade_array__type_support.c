// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from buff_interfaces:msg/BladeArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "buff_interfaces/msg/detail/blade_array__rosidl_typesupport_introspection_c.h"
#include "buff_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "buff_interfaces/msg/detail/blade_array__functions.h"
#include "buff_interfaces/msg/detail/blade_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `blades`
#include "buff_interfaces/msg/blade.h"
// Member `blades`
#include "buff_interfaces/msg/detail/blade__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  buff_interfaces__msg__BladeArray__init(message_memory);
}

void buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_fini_function(void * message_memory)
{
  buff_interfaces__msg__BladeArray__fini(message_memory);
}

size_t buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__size_function__BladeArray__blades(
  const void * untyped_member)
{
  const buff_interfaces__msg__Blade__Sequence * member =
    (const buff_interfaces__msg__Blade__Sequence *)(untyped_member);
  return member->size;
}

const void * buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__get_const_function__BladeArray__blades(
  const void * untyped_member, size_t index)
{
  const buff_interfaces__msg__Blade__Sequence * member =
    (const buff_interfaces__msg__Blade__Sequence *)(untyped_member);
  return &member->data[index];
}

void * buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__get_function__BladeArray__blades(
  void * untyped_member, size_t index)
{
  buff_interfaces__msg__Blade__Sequence * member =
    (buff_interfaces__msg__Blade__Sequence *)(untyped_member);
  return &member->data[index];
}

void buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__fetch_function__BladeArray__blades(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const buff_interfaces__msg__Blade * item =
    ((const buff_interfaces__msg__Blade *)
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__get_const_function__BladeArray__blades(untyped_member, index));
  buff_interfaces__msg__Blade * value =
    (buff_interfaces__msg__Blade *)(untyped_value);
  *value = *item;
}

void buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__assign_function__BladeArray__blades(
  void * untyped_member, size_t index, const void * untyped_value)
{
  buff_interfaces__msg__Blade * item =
    ((buff_interfaces__msg__Blade *)
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__get_function__BladeArray__blades(untyped_member, index));
  const buff_interfaces__msg__Blade * value =
    (const buff_interfaces__msg__Blade *)(untyped_value);
  *item = *value;
}

bool buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__resize_function__BladeArray__blades(
  void * untyped_member, size_t size)
{
  buff_interfaces__msg__Blade__Sequence * member =
    (buff_interfaces__msg__Blade__Sequence *)(untyped_member);
  buff_interfaces__msg__Blade__Sequence__fini(member);
  return buff_interfaces__msg__Blade__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__BladeArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "blades",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__BladeArray, blades),  // bytes offset in struct
    NULL,  // default value
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__size_function__BladeArray__blades,  // size() function pointer
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__get_const_function__BladeArray__blades,  // get_const(index) function pointer
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__get_function__BladeArray__blades,  // get(index) function pointer
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__fetch_function__BladeArray__blades,  // fetch(index, &value) function pointer
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__assign_function__BladeArray__blades,  // assign(index, value) function pointer
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__resize_function__BladeArray__blades  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_members = {
  "buff_interfaces__msg",  // message namespace
  "BladeArray",  // message name
  2,  // number of fields
  sizeof(buff_interfaces__msg__BladeArray),
  buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_member_array,  // message members
  buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_init_function,  // function to initialize message memory (memory has to be allocated)
  buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_type_support_handle = {
  0,
  &buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_buff_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, buff_interfaces, msg, BladeArray)() {
  buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, buff_interfaces, msg, Blade)();
  if (!buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_type_support_handle.typesupport_identifier) {
    buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &buff_interfaces__msg__BladeArray__rosidl_typesupport_introspection_c__BladeArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif