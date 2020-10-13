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

#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/BodypartDetection.h>

namespace openpose_ros_msgs
{
template <class ContainerAllocator>
struct PersonDetection_
{
  typedef PersonDetection_<ContainerAllocator> Type;

  PersonDetection_()
    : num_people_detected(0)
    , person_ID(0)
    , nose()
    , neck()
    , right_shoulder()
    , right_elbow()
    , right_wrist()
    , left_shoulder()
    , left_elbow()
    , left_wrist()
    , right_hip()
    , right_knee()
    , right_ankle()
    , left_hip()
    , left_knee()
    , left_ankle()
    , right_eye()
    , left_eye()
    , right_ear()
    , left_ear()
    , chest()  {
    }
  PersonDetection_(const ContainerAllocator& _alloc)
    : num_people_detected(0)
    , person_ID(0)
    , nose(_alloc)
    , neck(_alloc)
    , right_shoulder(_alloc)
    , right_elbow(_alloc)
    , right_wrist(_alloc)
    , left_shoulder(_alloc)
    , left_elbow(_alloc)
    , left_wrist(_alloc)
    , right_hip(_alloc)
    , right_knee(_alloc)
    , right_ankle(_alloc)
    , left_hip(_alloc)
    , left_knee(_alloc)
    , left_ankle(_alloc)
    , right_eye(_alloc)
    , left_eye(_alloc)
    , right_ear(_alloc)
    , left_ear(_alloc)
    , chest(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _num_people_detected_type;
  _num_people_detected_type num_people_detected;

   typedef uint32_t _person_ID_type;
  _person_ID_type person_ID;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _nose_type;
  _nose_type nose;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _neck_type;
  _neck_type neck;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _right_shoulder_type;
  _right_shoulder_type right_shoulder;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _right_elbow_type;
  _right_elbow_type right_elbow;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _right_wrist_type;
  _right_wrist_type right_wrist;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _left_shoulder_type;
  _left_shoulder_type left_shoulder;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _left_elbow_type;
  _left_elbow_type left_elbow;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _left_wrist_type;
  _left_wrist_type left_wrist;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _right_hip_type;
  _right_hip_type right_hip;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _right_knee_type;
  _right_knee_type right_knee;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _right_ankle_type;
  _right_ankle_type right_ankle;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _left_hip_type;
  _left_hip_type left_hip;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _left_knee_type;
  _left_knee_type left_knee;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _left_ankle_type;
  _left_ankle_type left_ankle;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _right_eye_type;
  _right_eye_type right_eye;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _left_eye_type;
  _left_eye_type left_eye;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _right_ear_type;
  _right_ear_type right_ear;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _left_ear_type;
  _left_ear_type left_ear;

   typedef  ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator>  _chest_type;
  _chest_type chest;





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



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'openpose_ros_msgs': ['/home/zheng/robot_ws_zheng/src/openpose_ros/openpose_ros_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::openpose_ros_msgs::PersonDetection_<ContainerAllocator> const>
  : TrueType
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
    return "d01c2d124a49b2cef7015366ea4e0099";
  }

  static const char* value(const ::openpose_ros_msgs::PersonDetection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd01c2d124a49b2ceULL;
  static const uint64_t static_value2 = 0xf7015366ea4e0099ULL;
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
    return "uint32 num_people_detected\n"
"uint32 person_ID\n"
"BodypartDetection nose\n"
"BodypartDetection neck\n"
"BodypartDetection right_shoulder\n"
"BodypartDetection right_elbow\n"
"BodypartDetection right_wrist\n"
"BodypartDetection left_shoulder\n"
"BodypartDetection left_elbow\n"
"BodypartDetection left_wrist\n"
"BodypartDetection right_hip\n"
"BodypartDetection right_knee\n"
"BodypartDetection right_ankle\n"
"BodypartDetection left_hip\n"
"BodypartDetection left_knee\n"
"BodypartDetection left_ankle\n"
"BodypartDetection right_eye\n"
"BodypartDetection left_eye\n"
"BodypartDetection right_ear\n"
"BodypartDetection left_ear\n"
"BodypartDetection chest\n"
"\n"
"================================================================================\n"
"MSG: openpose_ros_msgs/BodypartDetection\n"
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
      stream.next(m.num_people_detected);
      stream.next(m.person_ID);
      stream.next(m.nose);
      stream.next(m.neck);
      stream.next(m.right_shoulder);
      stream.next(m.right_elbow);
      stream.next(m.right_wrist);
      stream.next(m.left_shoulder);
      stream.next(m.left_elbow);
      stream.next(m.left_wrist);
      stream.next(m.right_hip);
      stream.next(m.right_knee);
      stream.next(m.right_ankle);
      stream.next(m.left_hip);
      stream.next(m.left_knee);
      stream.next(m.left_ankle);
      stream.next(m.right_eye);
      stream.next(m.left_eye);
      stream.next(m.right_ear);
      stream.next(m.left_ear);
      stream.next(m.chest);
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
    s << indent << "num_people_detected: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_people_detected);
    s << indent << "person_ID: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.person_ID);
    s << indent << "nose: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.nose);
    s << indent << "neck: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.neck);
    s << indent << "right_shoulder: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.right_shoulder);
    s << indent << "right_elbow: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.right_elbow);
    s << indent << "right_wrist: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.right_wrist);
    s << indent << "left_shoulder: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.left_shoulder);
    s << indent << "left_elbow: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.left_elbow);
    s << indent << "left_wrist: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.left_wrist);
    s << indent << "right_hip: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.right_hip);
    s << indent << "right_knee: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.right_knee);
    s << indent << "right_ankle: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.right_ankle);
    s << indent << "left_hip: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.left_hip);
    s << indent << "left_knee: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.left_knee);
    s << indent << "left_ankle: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.left_ankle);
    s << indent << "right_eye: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.right_eye);
    s << indent << "left_eye: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.left_eye);
    s << indent << "right_ear: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.right_ear);
    s << indent << "left_ear: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.left_ear);
    s << indent << "chest: ";
    s << std::endl;
    Printer< ::openpose_ros_msgs::BodypartDetection_<ContainerAllocator> >::stream(s, indent + "  ", v.chest);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENPOSE_ROS_MSGS_MESSAGE_PERSONDETECTION_H
