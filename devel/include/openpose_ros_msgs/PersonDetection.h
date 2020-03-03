// Generated by gencpp from file openpose_ros_msgs/PersonDetection.msg
// DO NOT EDIT!


#ifndef OPENPOSE_ROS_MSGS_MESSAGE_PERSONDETECTION_H
#define OPENPOSE_ROS_MSGS_MESSAGE_PERSONDETECTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <openpose_ros_msgs/BodyPartDetection.h>
#include <openpose_ros_msgs/BodyPartDetection.h>

namespace openpose_ros_msgs
{
template <class ContainerAllocator>
struct PersonDetection_
{
  typedef PersonDetection_<ContainerAllocator> Type;

  PersonDetection_()
    : face_landmark()
    , body_part()  {
    }
  PersonDetection_(const ContainerAllocator& _alloc)
    : face_landmark(_alloc)
    , body_part(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::openpose_ros_msgs::BodyPartDetection_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::openpose_ros_msgs::BodyPartDetection_<ContainerAllocator> >::other >  _face_landmark_type;
  _face_landmark_type face_landmark;

   typedef std::vector< ::openpose_ros_msgs::BodyPartDetection_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::openpose_ros_msgs::BodyPartDetection_<ContainerAllocator> >::other >  _body_part_type;
  _body_part_type body_part;





  typedef boost::shared_ptr< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> const> ConstPtr;

}; // struct PersonDetection_

typedef ::openpose_ros_msgs::PersonDetection_<std::allocator<void> > PersonDetection;

typedef boost::shared_ptr< ::openpose_ros_msgs::PersonDetection > PersonDetectionPtr;
typedef boost::shared_ptr< ::openpose_ros_msgs::PersonDetection const> PersonDetectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace openpose_ros_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'openpose_ros_msgs': ['/home/zheng/robot_ws_zheng/src/ros-openpose/openpose_ros_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5cf57469f872bf144c62ed7904d5c159";
  }

  static const char* value(const ::openpose_ros_msgs::PersonDetection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5cf57469f872bf14ULL;
  static const uint64_t static_value2 = 0x4c62ed7904d5c159ULL;
};

template<class ContainerAllocator>
struct DataType< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "openpose_ros_msgs/PersonDetection";
  }

  static const char* value(const ::openpose_ros_msgs::PersonDetection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "BodyPartDetection[] face_landmark\n"
"BodyPartDetection[] body_part\n"
"\n"
"================================================================================\n"
"MSG: openpose_ros_msgs/BodyPartDetection\n"
"uint32 part_id\n"
"uint32 x\n"
"uint32 y\n"
"float32 confidence\n"
;
  }

  static const char* value(const ::openpose_ros_msgs::PersonDetection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.face_landmark);
      stream.next(m.body_part);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PersonDetection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::openpose_ros_msgs::PersonDetection_<ContainerAllocator>& v)
  {
    s << indent << "face_landmark[]" << std::endl;
    for (size_t i = 0; i < v.face_landmark.size(); ++i)
    {
      s << indent << "  face_landmark[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::openpose_ros_msgs::BodyPartDetection_<ContainerAllocator> >::stream(s, indent + "    ", v.face_landmark[i]);
    }
    s << indent << "body_part[]" << std::endl;
    for (size_t i = 0; i < v.body_part.size(); ++i)
    {
      s << indent << "  body_part[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::openpose_ros_msgs::BodyPartDetection_<ContainerAllocator> >::stream(s, indent + "    ", v.body_part[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENPOSE_ROS_MSGS_MESSAGE_PERSONDETECTION_H
