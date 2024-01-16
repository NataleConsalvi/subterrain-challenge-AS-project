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

void currentStateCallback (const nav_msgs::Odometry& cur_state){

        current_state.setX(cur_state.pose.pose.position.x);
    current_state.setY(cur_state.pose.pose.position.y);
    current_state.setZ(cur_state.pose.pose.position.z);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    ros::Subscriber current_state_est = n.subscribe("current_state", 1, currentStateCallback);
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    int count = 0;
    while (ros::ok()) {
        tf::Vector3 origin(-38,10,6.5);

        double t = (ros::Time::now()-start).toSec();

        // Quantities to fill in
        tf::Transform desired_pose(tf::Transform::getIdentity());

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;


        double takeoff_duration = 30.0;
        double takeoff_instant = 20.0;
        double transition_duration = 5.0;

ROS_INFO("Time: %f", t);
         if (t < takeoff_instant) {
 ROS_INFO("Phase: Takeoff Instant");
                tf::Vector3 target_pose (0.0,0.0,0.0);
                desired_pose.setOrigin(origin + target_pose);
                desired_pose.setOrigin(lerp(origin, origin + target_pose, t / takeoff_instant));

        }


        if (t < takeoff_duration && t >= takeoff_instant) {
ROS_INFO("Phase: Takeoff Duration");
                tf::Vector3 target_pose (0.0,0.0,10.0);
                 double normalized_time = (t - takeoff_instant) / (takeoff_duration - takeoff_instant);
               desired_pose.setOrigin(lerp(origin, origin + target_pose, normalized_time));

        }



        if (t > takeoff_duration) {
ROS_INFO("Phase: After Takeoff Duration");
                tf::Vector3 target_pose (+50,20, 0.0);
                desired_pose.setOrigin(origin + target_pose);
                double distance = (target_pose - current_state).length();
                double normalized_time = (t - takeoff_duration)/10;
                desired_pose.setOrigin(lerp(origin, origin + target_pose, normalized_time));
                ROS_INFO("Current Position: x = %f, y = %f, z = %f", current_state.x(), current_state.y(), current_state.z());


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
           << " z:" << desired_pose.getOrigin().z();
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