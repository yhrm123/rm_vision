// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from buff_interfaces:msg/TimeInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "buff_interfaces/msg/detail/time_info__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace buff_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TimeInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) buff_interfaces::msg::TimeInfo(_init);
}

void TimeInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<buff_interfaces::msg::TimeInfo *>(message_memory);
  typed_message->~TimeInfo();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TimeInfo_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces::msg::TimeInfo, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces::msg::TimeInfo, time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TimeInfo_message_members = {
  "buff_interfaces::msg",  // message namespace
  "TimeInfo",  // message name
  2,  // number of fields
  sizeof(buff_interfaces::msg::TimeInfo),
  TimeInfo_message_member_array,  // message members
  TimeInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  TimeInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TimeInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TimeInfo_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace buff_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<buff_interfaces::msg::TimeInfo>()
{
  return &::buff_interfaces::msg::rosidl_typesupport_introspection_cpp::TimeInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, buff_interfaces, msg, TimeInfo)() {
  return &::buff_interfaces::msg::rosidl_typesupport_introspection_cpp::TimeInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
