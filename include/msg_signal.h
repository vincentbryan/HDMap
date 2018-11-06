// Generated by gencpp from file HDMap/msg_signal.msg
// DO NOT EDIT!


#ifndef HDMAP_MESSAGE_MSG_SIGNAL_H
#define HDMAP_MESSAGE_MSG_SIGNAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace HDMap
{
template <class ContainerAllocator>
struct msg_signal_
{
  typedef msg_signal_<ContainerAllocator> Type;

  msg_signal_()
    : header()
    , type()
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , vertical(false)
    , shape()
    , left(false)
    , forward(false)
    , right(false)  {
    }
  msg_signal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , type(_alloc)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , vertical(false)
    , shape(_alloc)
    , left(false)
    , forward(false)
    , right(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef uint8_t _vertical_type;
  _vertical_type vertical;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _shape_type;
  _shape_type shape;

   typedef uint8_t _left_type;
  _left_type left;

   typedef uint8_t _forward_type;
  _forward_type forward;

   typedef uint8_t _right_type;
  _right_type right;





  typedef boost::shared_ptr< ::HDMap::msg_signal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::HDMap::msg_signal_<ContainerAllocator> const> ConstPtr;

}; // struct msg_signal_

typedef ::HDMap::msg_signal_<std::allocator<void> > msg_signal;

typedef boost::shared_ptr< ::HDMap::msg_signal > msg_signalPtr;
typedef boost::shared_ptr< ::HDMap::msg_signal const> msg_signalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::HDMap::msg_signal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::HDMap::msg_signal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace HDMap

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/lunar/share/geometry_msgs/cmake/../msg'], 'HDMap': ['/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/msg'], 'std_msgs': ['/opt/ros/lunar/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::HDMap::msg_signal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::HDMap::msg_signal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::HDMap::msg_signal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::HDMap::msg_signal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::HDMap::msg_signal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::HDMap::msg_signal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::HDMap::msg_signal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d4484975e6b49462dc8458ec6a03dcd7";
  }

  static const char* value(const ::HDMap::msg_signal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd4484975e6b49462ULL;
  static const uint64_t static_value2 = 0xdc8458ec6a03dcd7ULL;
};

template<class ContainerAllocator>
struct DataType< ::HDMap::msg_signal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "HDMap/msg_signal";
  }

  static const char* value(const ::HDMap::msg_signal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::HDMap::msg_signal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
string type\n\
float64 x\n\
float64 y\n\
float64 z\n\
bool vertical\n\
string shape\n\
bool left\n\
bool forward\n\
bool right\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::HDMap::msg_signal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::HDMap::msg_signal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.type);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.vertical);
      stream.next(m.shape);
      stream.next(m.left);
      stream.next(m.forward);
      stream.next(m.right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct msg_signal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::HDMap::msg_signal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::HDMap::msg_signal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "vertical: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vertical);
    s << indent << "shape: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.shape);
    s << indent << "left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left);
    s << indent << "forward: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.forward);
    s << indent << "right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HDMAP_MESSAGE_MSG_SIGNAL_H