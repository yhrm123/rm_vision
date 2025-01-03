// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/TimeSeries.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TIME_SERIES__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TIME_SERIES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/time_series__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TimeSeries_z
{
public:
  explicit Init_TimeSeries_z(::auto_aim_interfaces::msg::TimeSeries & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::TimeSeries z(::auto_aim_interfaces::msg::TimeSeries::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TimeSeries msg_;
};

class Init_TimeSeries_y
{
public:
  explicit Init_TimeSeries_y(::auto_aim_interfaces::msg::TimeSeries & msg)
  : msg_(msg)
  {}
  Init_TimeSeries_z y(::auto_aim_interfaces::msg::TimeSeries::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_TimeSeries_z(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TimeSeries msg_;
};

class Init_TimeSeries_x
{
public:
  explicit Init_TimeSeries_x(::auto_aim_interfaces::msg::TimeSeries & msg)
  : msg_(msg)
  {}
  Init_TimeSeries_y x(::auto_aim_interfaces::msg::TimeSeries::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_TimeSeries_y(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TimeSeries msg_;
};

class Init_TimeSeries_header
{
public:
  Init_TimeSeries_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimeSeries_x header(::auto_aim_interfaces::msg::TimeSeries::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TimeSeries_x(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TimeSeries msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::TimeSeries>()
{
  return auto_aim_interfaces::msg::builder::Init_TimeSeries_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TIME_SERIES__BUILDER_HPP_
