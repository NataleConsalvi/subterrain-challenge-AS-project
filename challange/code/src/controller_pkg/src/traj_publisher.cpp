#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>
#include <nav_msgs/Odometry.h>

#include <math.h>

#define STATIC_POSE 0

#define STRAIGHT_POSE 1

#define PI M_PI

#define TFOUTPUT 1

tf::Vector3 current_state;
double current_yaw;



void currentStateCallback (const nav_msgs::Odometry& cur_state){

        current_state.setX(cur_state.pose.pose.position.x);
    current_state.setY(cur_state.pose.pose.position.y);
    current_state.setZ(cur_state.pose.pose.position.z);
    current_yaw = tf::getYaw(cur_state.pose.pose.orientation);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    ros::Subscriber current_state_est = n.subscribe("current_state_est", 1, currentStateCallback);
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    int count = 0;
    while (ros::ok()) {

        /*double Xstart = current_state.x();
        double Ystart = current_state.y();
        double Zstart = current_state.z();*/
        tf::Vector3 origin(-38.0,10.0,6.9);

        double t = (ros::Time::now()-start).toSec();

        // Quantities to fill in
        tf::Transform desired_pose(tf::Transform::getIdentity());

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;


        //double tTakeoff = 15.0;
        double tHoverEnd = 10;
        double tTakeoffStart = 5.0;
        double rotation_duration = 3;

        

ROS_INFO("Time: %f", t);
         
 //tf:: Vector3 temp_pose;

 #if STATIC_POSE
        // Static Pose
        tf::Vector3 displacement(0,0,2);
        desired_pose.setOrigin(origin+displacement);
        tf::Quaternion q;
        q.setRPY(0,0,PI/4);
        count++;
        std::cout<<"Desired Orientation" << count << std::endl;
        desired_pose.setRotation(q);

        #endif
tf::Vector3 target_pose1 (0.0,0.0,14.1);
tf::Vector3 target_pose2 (-308.0,+1.0, 0);

if (t >= tTakeoffStart && t < tHoverEnd ) {
 ROS_INFO("Phase: Takeoff ");
 ROS_INFO("Current Position: x = %f, y = %f, z = %f, Yaw = %f ", current_state.x(), current_state.y(), current_state.z(),current_yaw);
                
        tf::Quaternion q;
        q.setRPY(0,0,0);
        desired_pose.setRotation(q);
        desired_pose.setOrigin(origin + target_pose1);
}

if (t >=  tHoverEnd && t <= tHoverEnd + rotation_duration){

 ROS_INFO("Phase: Rotation ");
 ROS_INFO("Current Position: x = %f, y = %f, z = %f, Yaw = %f ", current_state.x(), current_state.y(), current_state.z(),current_yaw);

        double  x1 = origin.getX();
        double  y1 = origin.getY();
        double  x2 = target_pose2.getX();
        double y2 = target_pose2.getY();
       

        desired_pose.setOrigin(origin + target_pose1);

        double Yawd = std::atan2(y2 - y1, x2 - x1) ;
        tf::Quaternion qy;
        qy.setRPY(0,0,Yawd);
        desired_pose.setRotation(qy);
}
        

if (t > tHoverEnd + rotation_duration) {
 ROS_INFO("Phase: After Takeoff Duration");
 
                
        double  x1 = origin.getX();
        double  y1 = origin.getY();
        double  x2 = target_pose2.getX();
        double y2 = target_pose2.getY();
         double Yawd = std::atan2(y2 - y1, x2 - x1);
        tf::Quaternion qy;
        qy.setRPY(0,0,Yawd);
        double distance = (origin + target_pose1 + target_pose2 - current_state).length();
ROS_INFO("Current Position: x = %f, y = %f, z = %f, distance %f", current_state.x(), current_state.y(), current_state.z(), distance);
                
                if (distance > 1){

                desired_pose.setRotation(qy);
                
               
                double normalized_time = (t - tHoverEnd + rotation_duration)/100;
                desired_pose.setOrigin(lerp(origin + target_pose1, target_pose1 + target_pose2, normalized_time));
                
                
                
                }
                        else {
                        desired_pose.setOrigin(target_pose1 + target_pose2);
                        }
                 
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

        std::stringstream ss;
        ss << "Trajectory Position"
           << " x:" << desired_pose.getOrigin().x()
           << " y:" << desired_pose.getOrigin().y()
           << " z:" << desired_pose.getOrigin().z()
           << "Yawd:" <<tf::getYaw(desired_pose.getRotation());
        ROS_INFO("%s", ss.str().c_str());

#if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));
#endif

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
