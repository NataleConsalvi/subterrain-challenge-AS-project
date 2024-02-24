#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <iostream>

class WaypointCheckerNode {
public:
    WaypointCheckerNode() {
        desirePositionSub = nh.subscribe("airsim_ros_node/desired_state2", 1, &WaypointCheckerNode::desirePositionCallback, this);
        currentStateSub = nh.subscribe("current_state_est", 1, &WaypointCheckerNode::currentStateCallback, this);
        pointReachedPub = nh.advertise<std_msgs::Bool>("/airsim_ros_node/exploration/point_reached", 1);
    }

    void desirePositionCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state) {
        // Store the desired position
        geometry_msgs::Vector3 t = des_state.transforms[0].translation;
        xd << t.x, t.y, t.z;
         ROS_INFO("Received Desired Position (xd): [%.3f, %.3f, %.3f]", xd(0), xd(1), xd(2));
    }

    void currentStateCallback(const nav_msgs::Odometry& cur_state) {
        // Calculate the distance between the current position and the desired position
        Eigen::Vector3d x;
        x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
         Eigen::Vector3d diff= x-xd;
        double distance = diff.norm();
         ROS_INFO("Current Position (x): [%.3f, %.3f, %.3f]", x(0), x(1), x(2));
ROS_INFO("Desired Position (xd): [%.3f, %.3f, %.3f]", xd(0), xd(1), xd(2));


ROS_INFO("Distance to desired position: %f", distance);
        // Check if the distance is within the threshold
        bool pointReached = (distance <= thresholdDistance);
         ROS_INFO("Point reached: %s", pointReached ? "true" : "false");

        // Publish the result
        std_msgs::Bool reachedMsg;
        reachedMsg.data = pointReached;
        pointReachedPub.publish(reachedMsg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber desirePositionSub;
    ros::Subscriber currentStateSub;
    ros::Publisher pointReachedPub;
    Eigen::Vector3d xd;
    double thresholdDistance = 5; // Set your desired threshold here
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_checker_node");

    WaypointCheckerNode node;

    ros::spin();

    return 0;
}
