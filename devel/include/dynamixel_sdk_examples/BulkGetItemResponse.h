// Generated by gencpp from file dynamixel_sdk_examples/BulkGetItemResponse.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_SDK_EXAMPLES_MESSAGE_BULKGETITEMRESPONSE_H
#define DYNAMIXEL_SDK_EXAMPLES_MESSAGE_BULKGETITEMRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dynamixel_sdk_examples
{
template <class ContainerAllocator>
struct BulkGetItemResponse_
{
  typedef BulkGetItemResponse_<ContainerAllocator> Type;

  BulkGetItemResponse_()
    : value1(0)
    , value2(0)  {
    }
  BulkGetItemResponse_(const ContainerAllocator& _alloc)
    : value1(0)
    , value2(0)  {
  (void)_alloc;
    }



   typedef int32_t _value1_type;
  _value1_type value1;

   typedef int32_t _value2_type;
  _value2_type value2;





  typedef boost::shared_ptr< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> const> ConstPtr;

}; // struct BulkGetItemResponse_

typedef ::dynamixel_sdk_examples::BulkGetItemResponse_<std::allocator<void> > BulkGetItemResponse;

typedef boost::shared_ptr< ::dynamixel_sdk_examples::BulkGetItemResponse > BulkGetItemResponsePtr;
typedef boost::shared_ptr< ::dynamixel_sdk_examples::BulkGetItemResponse const> BulkGetItemResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator1> & lhs, const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator2> & rhs)
{
  return lhs.value1 == rhs.value1 &&
    lhs.value2 == rhs.value2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator1> & lhs, const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dynamixel_sdk_examples

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e5963c91be0598b7f68fed70b98f2326";
  }

  static const char* value(const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe5963c91be0598b7ULL;
  static const uint64_t static_value2 = 0xf68fed70b98f2326ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamixel_sdk_examples/BulkGetItemResponse";
  }

  static const char* value(const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 value1\n"
"int32 value2\n"
"\n"
;
  }

  static const char* value(const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value1);
      stream.next(m.value2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BulkGetItemResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dynamixel_sdk_examples::BulkGetItemResponse_<ContainerAllocator>& v)
  {
    s << indent << "value1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.value1);
    s << indent << "value2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.value2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIXEL_SDK_EXAMPLES_MESSAGE_BULKGETITEMRESPONSE_H
