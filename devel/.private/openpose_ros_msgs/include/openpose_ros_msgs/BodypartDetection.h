// Generated by gencpp from file openpose_ros_msgs/BodypartDetection.msg
// DO NOT EDIT!


#ifndef OPENPOSE_ROS_MSGS_MESSAGE_BODYPARTDETECTION_H
#define OPENPOSE_ROS_MSGS_MESSAGE_BODYPARTDETECTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace openpose_ros_msgs
{
template <class ContainerAllocator>
struct BodypartDetection_
{
  typedef BodypartDetection_<ContainerAllocator> Type;

  BodypartDetection_()
    : x(0)
    , y(0)
    , confidence(0.0)  {
    }
  BodypartDetection_(const ContainerAllocator& _alloc)
    : x(0)
    , y(0)
    , confidence(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _x_type;
  _x_type x;

   typedef uint32_t _y_type;
  _y_type y;

   typedef float _confidence_type;
  _confidence_type confidence;





  typedef boost::shared_ptr< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> const> ConstPtr;

}; // struct BodypartDetection_

typedef ::openpose_ros_msgs::BodypartDetection_<std::allocator<void> > BodypartDetection;

typedef boost::shared_ptr< ::openpose_ros_msgs::BodypartDetection > BodypartDetectionPtr;
typedef boost::shared_ptr< ::openpose_ros_msgs::BodypartDetection const> BodypartDetectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace openpose_ros_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'openpose_ros_msgs': ['/home/zheng/robot_ws_zheng/src/openpose_ros/openpose_ros_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2edd10e314180ad374d9e70737f3fc60";
  }

  static const char* value(const ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2edd10e314180ad3ULL;
  static const uint64_t static_value2 = 0x74d9e70737f3fc60ULL;
};

template<class ContainerAllocator>
struct DataType< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "openpose_ros_msgs/BodypartDetection";
  }

  static const char* value(const ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 x\n"
"uint32 y\n"
"float32 confidence\n"
;
  }

  static const char* value(const ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.confidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BodypartDetection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.y);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENPOSE_ROS_MSGS_MESSAGE_BODYPARTDETECTION_H