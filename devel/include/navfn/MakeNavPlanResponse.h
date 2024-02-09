// Generated by gencpp from file navfn/MakeNavPlanResponse.msg
// DO NOT EDIT!


#ifndef NAVFN_MESSAGE_MAKENAVPLANRESPONSE_H
#define NAVFN_MESSAGE_MAKENAVPLANRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>

namespace navfn
{
template <class ContainerAllocator>
struct MakeNavPlanResponse_
{
  typedef MakeNavPlanResponse_<ContainerAllocator> Type;

  MakeNavPlanResponse_()
    : plan_found(0)
    , error_message()
    , path()  {
    }
  MakeNavPlanResponse_(const ContainerAllocator& _alloc)
    : plan_found(0)
    , error_message(_alloc)
    , path(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _plan_found_type;
  _plan_found_type plan_found;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _error_message_type;
  _error_message_type error_message;

   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::PoseStamped_<ContainerAllocator> >> _path_type;
  _path_type path;





  typedef boost::shared_ptr< ::navfn::MakeNavPlanResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navfn::MakeNavPlanResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MakeNavPlanResponse_

typedef ::navfn::MakeNavPlanResponse_<std::allocator<void> > MakeNavPlanResponse;

typedef boost::shared_ptr< ::navfn::MakeNavPlanResponse > MakeNavPlanResponsePtr;
typedef boost::shared_ptr< ::navfn::MakeNavPlanResponse const> MakeNavPlanResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navfn::MakeNavPlanResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::navfn::MakeNavPlanResponse_<ContainerAllocator1> & lhs, const ::navfn::MakeNavPlanResponse_<ContainerAllocator2> & rhs)
{
  return lhs.plan_found == rhs.plan_found &&
    lhs.error_message == rhs.error_message &&
    lhs.path == rhs.path;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::navfn::MakeNavPlanResponse_<ContainerAllocator1> & lhs, const ::navfn::MakeNavPlanResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace navfn

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navfn::MakeNavPlanResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navfn::MakeNavPlanResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navfn::MakeNavPlanResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b8ed7edf1b237dc9ddda8c8ffed5d3a";
  }

  static const char* value(const ::navfn::MakeNavPlanResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b8ed7edf1b237dcULL;
  static const uint64_t static_value2 = 0x9ddda8c8ffed5d3aULL;
};

template<class ContainerAllocator>
struct DataType< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navfn/MakeNavPlanResponse";
  }

  static const char* value(const ::navfn::MakeNavPlanResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint8 plan_found\n"
"string error_message\n"
"\n"
"# if plan_found is true, this is an array of waypoints from start to goal, where the first one equals start and the last one equals goal\n"
"geometry_msgs/PoseStamped[] path\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::navfn::MakeNavPlanResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.plan_found);
      stream.next(m.error_message);
      stream.next(m.path);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MakeNavPlanResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navfn::MakeNavPlanResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navfn::MakeNavPlanResponse_<ContainerAllocator>& v)
  {
    s << indent << "plan_found: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.plan_found);
    s << indent << "error_message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.error_message);
    s << indent << "path[]" << std::endl;
    for (size_t i = 0; i < v.path.size(); ++i)
    {
      s << indent << "  path[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.path[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVFN_MESSAGE_MAKENAVPLANRESPONSE_H
