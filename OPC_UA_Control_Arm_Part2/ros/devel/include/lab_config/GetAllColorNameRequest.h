// Generated by gencpp from file lab_config/GetAllColorNameRequest.msg
// DO NOT EDIT!


#ifndef LAB_CONFIG_MESSAGE_GETALLCOLORNAMEREQUEST_H
#define LAB_CONFIG_MESSAGE_GETALLCOLORNAMEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lab_config
{
template <class ContainerAllocator>
struct GetAllColorNameRequest_
{
  typedef GetAllColorNameRequest_<ContainerAllocator> Type;

  GetAllColorNameRequest_()
    {
    }
  GetAllColorNameRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetAllColorNameRequest_

typedef ::lab_config::GetAllColorNameRequest_<std::allocator<void> > GetAllColorNameRequest;

typedef boost::shared_ptr< ::lab_config::GetAllColorNameRequest > GetAllColorNameRequestPtr;
typedef boost::shared_ptr< ::lab_config::GetAllColorNameRequest const> GetAllColorNameRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lab_config::GetAllColorNameRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace lab_config

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::lab_config::GetAllColorNameRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lab_config/GetAllColorNameRequest";
  }

  static const char* value(const ::lab_config::GetAllColorNameRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::lab_config::GetAllColorNameRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetAllColorNameRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lab_config::GetAllColorNameRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::lab_config::GetAllColorNameRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // LAB_CONFIG_MESSAGE_GETALLCOLORNAMEREQUEST_H
