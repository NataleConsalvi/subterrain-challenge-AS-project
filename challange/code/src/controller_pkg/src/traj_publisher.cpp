#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>
#include <tf/LinearMath/Quaternion.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("red/position_hold/trajectory", 1);
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    int count = 0;    
    tf::Transform desired_pose(tf::Transform::getIdentity());
    geometry_msgs::Twist velocity;
    geometry_msgs::Twist acceleration;
        
        
   
 while (ros::ok())
    {
    
    tf::Vector3 origin(-38.0, 10.0, 6.9); // Starting location of the drone
    
    double t = (ros::Time::now() - start).toSec();

     
    velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
    velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
       
    acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
    acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;


    double takeoff_duration = 5;                  
    double target_height = 15;                     
    double cruise_duration = 40;
        
    tf::Vector3 target_position(-321.0, 10.0, 15); 
    tf::Vector3 finaltakeoff(-38.0, 10.0, 15);
        
    double takeoff_heading_angle = atan2(target_position.y() - origin.y(), target_position.x() - origin.x()) - M_PI;




 if (t < takeoff_duration) 
    {
      
    //ROS_INFO("Phase: take off and rotation");
      
    tf::Quaternion takeoff_rotation;
    takeoff_rotation.setRPY(0, 0, takeoff_heading_angle * (t / takeoff_duration));
           
    desired_pose.setOrigin(tf::Vector3(origin.x(), origin.y(), origin.z() + t / takeoff_duration * (target_height - origin.z())));
    desired_pose.setRotation(takeoff_rotation);
            
    }

 
         
         
 if (t >= takeoff_duration && t <= takeoff_duration + cruise_duration) 
   {
   //ROS_INFO("Phase: Cruise");
   
    // Calculate linear velocity for the cruise phase (from distance and time)
    double cruise_linear_speed = 7.5;  
    tf::Vector3 linear_velocity(cruise_linear_speed, 0.0, 0.0);  // Moving along the x-axis

    // Set desired linear velocity
    velocity.linear.x = linear_velocity.getX();
    velocity.linear.y = linear_velocity.getY();
    velocity.linear.z = linear_velocity.getZ();

    // Set desired acceleration to zero
    acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0.0;


    // Interpolate the current position towards the target position
    double progress = (t - takeoff_duration) / cruise_duration;
    desired_pose.setOrigin(tf::lerp(finaltakeoff, target_position, progress));

    // Set orientation 
    tf::Quaternion cruise_orientation;
    cruise_orientation.setRPY(0, 0, 180);  // Adjust roll, pitch, and yaw as needed
    desired_pose.setRotation(cruise_orientation);
}
 
 
 
 


      // Publish
    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.transforms[0].translation.x = desired_pose.getOrigin().x();
    msg.transforms[0].translation.y = desired_pose.getOrigin().y();
    msg.transforms[0].translation.z = desired_pose.getOrigin().z();
    msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
    msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
    msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
    msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
        
    msg.velocities.resize(1);
    msg.velocities[0] = velocity;
    msg.accelerations.resize(1);
    msg.accelerations[0] = acceleration;
    desired_state_pub.publish(msg);


   /*std::stringstream ss;
    ss << "Trajectory Position"
       << " x:" << desired_pose.getOrigin().x()
       << " y:" << desired_pose.getOrigin().y()
       << " z:" << desired_pose.getOrigin().z();
        ROS_INFO("%s", ss.str().c_str());*/


#if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));
#endif

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    
}

    ros::shutdown(); 
    return 0;
}
