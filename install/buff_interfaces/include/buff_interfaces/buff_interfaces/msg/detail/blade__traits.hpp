// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from buff_interfaces:msg/Blade.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__BLADE__TRAITS_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__BLADE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "buff_interfaces/msg/detail/blade__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace buff_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Blade & msg,
  std::ostream & out)
{
  out << "{";
  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << ", ";
  }

  // member: prob
  {
    out << "prob: ";
    rosidl_generator_traits::value_to_yaml(msg.prob, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Blade & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }

  // member: prob
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prob: ";
    rosidl_generator_traits::value_to_yaml(msg.prob, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Blade & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace buff_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use buff_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const buff_interfaces::msg::Blade & msg,
  std::ostream & out, size_t indentation = 0)
{
  buff_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use buff_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const buff_interfaces::msg::Blade & msg)
{
  return buff_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<buff_interfaces::msg::Blade>()
{
  return "buff_interfaces::msg::Blade";
}

template<>
inline const char * name<buff_interfaces::msg::Blade>()
{
  return "buff_interfaces/msg/Blade";
}

template<>
struct has_fixed_size<buff_interfaces::msg::Blade>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<buff_interfaces::msg::Blade>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<buff_interfaces::msg::Blade>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BUFF_INTERFACES__MSG__DETAIL__BLADE__TRAITS_HPP_
