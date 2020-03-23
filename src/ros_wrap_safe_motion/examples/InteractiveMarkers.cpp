
#ifndef RVIZ_VISUAL_TOOLS_IMARKER_SIMPLE_DEMO_H
#define RVIZ_VISUAL_TOOLS_IMARKER_SIMPLE_DEMO_H

// ROS
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/imarker_simple.h>

namespace rviz_visual_tools
{
class IMarkerSimpleDemo
{
public:
  /** \brief Constructor */
  IMarkerSimpleDemo() : nh_("~")
  {
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("panda_link0", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();

    // Create a random initial pose
    geometry_msgs::Pose init_pose;

    interactive_marker_pub_= nh_.advertise<geometry_msgs::Pose>("/interactivePose", 1000);
    init_pose.position.x = 0.7;
    init_pose.position.y = -0.2;
    init_pose.position.z = 0.1;
    // Create a 6DOF interactive marker
    static const double SCALE = 0.2;
    imarker_simple_.reset(new rviz_visual_tools::IMarkerSimple("imarker", SCALE, init_pose));

    // Add callback to this class
    imarker_simple_->setIMarkerCallback(std::bind(&IMarkerSimpleDemo::processIMarkerPose, this, std::placeholders::_1));

    ROS_INFO_STREAM_NAMED(name_, "IMarkerSimpleDemo Ready.");
  }

  void processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    // Show some spheres for fun
    visual_tools_->publishSphere(feedback->pose, visual_tools_->getRandColor());
    visual_tools_->trigger();
    interactive_marker_pub_.publish(feedback->pose);

  }
    
private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "imarker_simple_demo";
  ros::Publisher interactive_marker_pub_;

  // A shared node handle
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  rviz_visual_tools::IMarkerSimplePtr imarker_simple_;

};  // end class

// Create std pointers for this class
typedef std::shared_ptr<IMarkerSimpleDemo> IMarkerSimpleDemoPtr;
typedef std::shared_ptr<const IMarkerSimpleDemo> IMarkerSimpleDemoConstPtr;

}  // namespace rviz_visual_tools
#endif  // RVIZ_VISUAL_TOOLS_IMARKER_SIMPLE_DEMO_H

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "imarker_simple_demo");
  ROS_INFO_STREAM_NAMED("main", "Starting IMarkerSimpleDemo...");

  // Seed random number generator
  // srand (time(NULL));

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize main class
  rviz_visual_tools::IMarkerSimpleDemo server;

  ros::waitForShutdown();

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}