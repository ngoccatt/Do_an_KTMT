// Generated by gencpp from file lab_config/ChangeRange.msg
// DO NOT EDIT!


#ifndef LAB_CONFIG_MESSAGE_CHANGERANGE_H
#define LAB_CONFIG_MESSAGE_CHANGERANGE_H

#include <ros/service_traits.h>


#include <lab_config/ChangeRangeRequest.h>
#include <lab_config/ChangeRangeResponse.h>


namespace lab_config
{

struct ChangeRange
{

typedef ChangeRangeRequest Request;
typedef ChangeRangeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ChangeRange
} // namespace lab_config


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::lab_config::ChangeRange > {
  static const char* value()
  {
    return "b6426602006389ecb4fb8d1c27273f68";
  }

  static const char* value(const ::lab_config::ChangeRange&) { return value(); }
};

template<>
struct DataType< ::lab_config::ChangeRange > {
  static const char* value()
  {
    return "lab_config/ChangeRange";
  }

  static const char* value(const ::lab_config::ChangeRange&) { return value(); }
};


// service_traits::MD5Sum< ::lab_config::ChangeRangeRequest> should match
// service_traits::MD5Sum< ::lab_config::ChangeRange >
template<>
struct MD5Sum< ::lab_config::ChangeRangeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::lab_config::ChangeRange >::value();
  }
  static const char* value(const ::lab_config::ChangeRangeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::lab_config::ChangeRangeRequest> should match
// service_traits::DataType< ::lab_config::ChangeRange >
template<>
struct DataType< ::lab_config::ChangeRangeRequest>
{
  static const char* value()
  {
    return DataType< ::lab_config::ChangeRange >::value();
  }
  static const char* value(const ::lab_config::ChangeRangeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::lab_config::ChangeRangeResponse> should match
// service_traits::MD5Sum< ::lab_config::ChangeRange >
template<>
struct MD5Sum< ::lab_config::ChangeRangeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::lab_config::ChangeRange >::value();
  }
  static const char* value(const ::lab_config::ChangeRangeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::lab_config::ChangeRangeResponse> should match
// service_traits::DataType< ::lab_config::ChangeRange >
template<>
struct DataType< ::lab_config::ChangeRangeResponse>
{
  static const char* value()
  {
    return DataType< ::lab_config::ChangeRange >::value();
  }
  static const char* value(const ::lab_config::ChangeRangeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LAB_CONFIG_MESSAGE_CHANGERANGE_H