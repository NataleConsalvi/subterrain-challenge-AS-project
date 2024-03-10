#include "ros/ros.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

class CarrotTrajectoryPublisher
{
public:
  CarrotTrajectoryPublisher()
  {
    // Initialize the ROS node handle
    

    // Subscribe to the exploration/point_reached topic
    point_reached_sub_ = nh_.subscribe("red/exploration/point_reached", 1,
                                       &CarrotTrajectoryPublisher::pointReachedCallback, this);

    // Publish on the /red/carrot/trajectory topic
    carrot_trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("red/carrot/trajectory", 1);
    
    // Subscribe to the current_state_est topic
    current_state_est_sub_ = nh_.subscribe("current_state_est", 1,
                                          &CarrotTrajectoryPublisher::currentStateEstCallback, this);
  }

   void currentStateEstCallback(const nav_msgs::Odometry& msg)
  {
    // Store the current_state_est message for later use
    ROS_INFO("current_state_est received.");
    current_state_est_ = msg;
    
  }

  void pointReachedCallback(const std_msgs::Bool& msg)
  {
    if (msg.data)
    {
      // Flag is true, publish a carrot/trajectory message
      ROS_INFO("Point reached! Publishing carrot/trajectory message...");

      // Create a new carrot/trajectory message
      trajectory_msgs::MultiDOFJointTrajectoryPoint carrot_trajectory_msg;

      // Fill in the position and orientation data from current_state_est
      geometry_msgs::Transform transform;
      transform.translation.x = current_state_est_.pose.pose.position.x;  // Set translation x
      transform.translation.y = current_state_est_.pose.pose.position.y; // Set translation y
      transform.translation.z = current_state_est_.pose.pose.position.z;   // Set translation z
      transform.rotation.x = current_state_est_.pose.pose.orientation.x;      // Set rotation x
      transform.rotation.y = current_state_est_.pose.pose.orientation.y;      // Set rotation y
      transform.rotation.z = current_state_est_.pose.pose.orientation.z;   // Set rotation z
      transform.rotation.w = current_state_est_.pose.pose.orientation.w;    // Set rotation w

      // Add the transform to the trajectory message
      carrot_trajectory_msg.transforms.push_back(transform);

      // Publish the carrot/trajectory message
      carrot_trajectory_pub_.publish(carrot_trajectory_msg);
      ROS_INFO("Carrot/trajectory message published:");
    ROS_INFO_STREAM("Translation: x=" << transform.translation.x << ", y=" << transform.translation.y << ", z=" << transform.translation.z);
    ROS_INFO_STREAM("Rotation: x=" << transform.rotation.x << ", y=" << transform.rotation.y << ", z=" << transform.rotation.z << ", w=" << transform.rotation.w);
    }
    else {
      ROS_INFO("Point not reached.");
    }
  }

 

private:
  ros::NodeHandle nh_;
  ros::Subscriber point_reached_sub_;
  ros::Publisher carrot_trajectory_pub_;
  ros::Subscriber current_state_est_sub_;
  nav_msgs::Odometry current_state_est_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "carrot_trajectory_publisher");
  ROS_INFO("Node initialized.");

  CarrotTrajectoryPublisher carrot_publisher;
  ROS_INFO("CarrotTrajectoryPublisher initialized.");

  ros::Rate loop_rate(0.5);  

  while (ros::ok())
  {
    ros::spinOnce();  
    loop_rate.sleep();  
  }

  return 0;
}
