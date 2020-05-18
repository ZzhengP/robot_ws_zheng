// Generated by gencpp from file openpose_ros_msgs/GetPersons.msg
// DO NOT EDIT!


#ifndef OPENPOSE_ROS_MSGS_MESSAGE_GETPERSONS_H
#define OPENPOSE_ROS_MSGS_MESSAGE_GETPERSONS_H

#include <ros/service_traits.h>


#include <openpose_ros_msgs/GetPersonsRequest.h>
#include <openpose_ros_msgs/GetPersonsResponse.h>


namespace openpose_ros_msgs
{

struct GetPersons
{

typedef GetPersonsRequest Request;
typedef GetPersonsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetPersons
} // namespace openpose_ros_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::openpose_ros_msgs::GetPersons > {
  static const char* value()
  {
    return "b372c9a3a4112a64a284d512401cc11f";
  }

  static const char* value(const ::openpose_ros_msgs::GetPersons&) { return value(); }
};

template<>
struct DataType< ::openpose_ros_msgs::GetPersons > {
  static const char* value()
  {
    return "openpose_ros_msgs/GetPersons";
  }

  static const char* value(const ::openpose_ros_msgs::GetPersons&) { return value(); }
};


// service_traits::MD5Sum< ::openpose_ros_msgs::GetPersonsRequest> should match 
// service_traits::MD5Sum< ::openpose_ros_msgs::GetPersons > 
template<>
struct MD5Sum< ::openpose_ros_msgs::GetPersonsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::openpose_ros_msgs::GetPersons >::value();
  }
  static const char* value(const ::openpose_ros_msgs::GetPersonsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::openpose_ros_msgs::GetPersonsRequest> should match 
// service_traits::DataType< ::openpose_ros_msgs::GetPersons > 
template<>
struct DataType< ::openpose_ros_msgs::GetPersonsRequest>
{
  static const char* value()
  {
    return DataType< ::openpose_ros_msgs::GetPersons >::value();
  }
  static const char* value(const ::openpose_ros_msgs::GetPersonsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::openpose_ros_msgs::GetPersonsResponse> should match 
// service_traits::MD5Sum< ::openpose_ros_msgs::GetPersons > 
template<>
struct MD5Sum< ::openpose_ros_msgs::GetPersonsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::openpose_ros_msgs::GetPersons >::value();
  }
  static const char* value(const ::openpose_ros_msgs::GetPersonsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::openpose_ros_msgs::GetPersonsResponse> should match 
// service_traits::DataType< ::openpose_ros_msgs::GetPersons > 
template<>
struct DataType< ::openpose_ros_msgs::GetPersonsResponse>
{
  static const char* value()
  {
    return DataType< ::openpose_ros_msgs::GetPersons >::value();
  }
  static const char* value(const ::openpose_ros_msgs::GetPersonsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OPENPOSE_ROS_MSGS_MESSAGE_GETPERSONS_H
