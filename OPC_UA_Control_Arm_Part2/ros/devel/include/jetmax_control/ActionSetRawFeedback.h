// Generated by gencpp from file jetmax_control/ActionSetRawFeedback.msg
// DO NOT EDIT!


#ifndef JETMAX_CONTROL_MESSAGE_ACTIONSETRAWFEEDBACK_H
#define JETMAX_CONTROL_MESSAGE_ACTIONSETRAWFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jetmax_control
{
template <class ContainerAllocator>
struct ActionSetRawFeedback_
{
  typedef ActionSetRawFeedback_<ContainerAllocator> Type;

  ActionSetRawFeedback_()
    : index(0)
    , count(0)  {
    }
  ActionSetRawFeedback_(const ContainerAllocator& _alloc)
    : index(0)
    , count(0)  {
  (void)_alloc;
    }



   typedef uint32_t _index_type;
  _index_type index;

   typedef uint32_t _count_type;
  _count_type count;





  typedef boost::shared_ptr< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct ActionSetRawFeedback_

typedef ::jetmax_control::ActionSetRawFeedback_<std::allocator<void> > ActionSetRawFeedback;

typedef boost::shared_ptr< ::jetmax_control::ActionSetRawFeedback > ActionSetRawFeedbackPtr;
typedef boost::shared_ptr< ::jetmax_control::ActionSetRawFeedback const> ActionSetRawFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator1> & lhs, const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.count == rhs.count;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator1> & lhs, const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jetmax_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9bd522b74b786e5c761ef1c23a598fdf";
  }

  static const char* value(const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9bd522b74b786e5cULL;
  static const uint64_t static_value2 = 0x761ef1c23a598fdfULL;
};

template<class ContainerAllocator>
struct DataType< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jetmax_control/ActionSetRawFeedback";
  }

  static const char* value(const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"uint32 index\n"
"uint32 count\n"
;
  }

  static const char* value(const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.count);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ActionSetRawFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jetmax_control::ActionSetRawFeedback_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.index);
    s << indent << "count: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.count);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JETMAX_CONTROL_MESSAGE_ACTIONSETRAWFEEDBACK_H
