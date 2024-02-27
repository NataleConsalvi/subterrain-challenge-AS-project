#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>
#include <tf/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#define FLY_TOWARDS_TARGET 1

#define PI M_PI

#define TFOUTPUT 0

class TrajPublisher
{
private:
	ros::NodeHandle n;
	ros::Publisher desired_state_pub;
	
	// Topics from state machine
    ros::Subscriber received_state;
    int actual_state;   //Represents de actual state

    geometry_msgs::Twist velocity;
    geometry_msgs::Twist acceleration;
 
    tf::Transform desired_pose;
    
    ros::Subscriber current_state;
    
    tf::Vector3 actual_coordinates;
    tf::Vector3 previous_coordinates;

public:
    //Constructor
    TrajPublisher()
    {
        ROS_INFO("TRAJ_PUBLISHER INITIALIZED.");
        desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("airsim_ros_node/desired_state2", 1);
        
        int count = 0;
        ros::Time start(ros::Time::now());

        actual_state = 0;

        desired_pose = tf::Transform::getIdentity();

        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

        received_state  = n.subscribe("/state", 1, &TrajPublisher::State_callback, this);
        current_state = n.subscribe("current_state_est", 1, &TrajPublisher::onCurrentState, this);
    }


    void State_callback(const std_msgs::Int64& state_msg)
    {
        int new_state = state_msg.data;
        if(new_state != actual_state)
        { 
            actual_state = new_state;
            tf::TransformBroadcaster br;
            switch(actual_state)
            {
                case 1:
                //CODE
                    break;

                case 2:
                {
                    bool reachedcave = false;
                    ros::Rate loop_rate(500);
                    tf::Vector3 origin(-38.0, 10.0, 6.9); // Starting location of the drone
                
                    double takeoff_duration = 5;                  
                    double target_height = 15;                     
                    double cruise_duration = 40;
                    
                    tf::Vector3 target_position(-321.0, 10.0, 15); 
                    tf::Vector3 finaltakeoff(-38.0, 10.0, 15);
                    ros::Time start(ros::Time::now());
                    
                    while(!reachedcave)
                    {
                        double t = (ros::Time::now() - start).toSec();
                        double takeoff_heading_angle = atan2(target_position.y() - origin.y(), target_position.x() - origin.x()) - M_PI;
                        
                        if (t < takeoff_duration) 
                        {
                            //ROS_INFO("Phase: take off and rotation");
            
                            tf::Quaternion takeoff_rotation;
                            takeoff_rotation.setRPY(0, 0, takeoff_heading_angle * (t / takeoff_duration));
                            desired_pose.setOrigin(tf::Vector3(origin.x(), origin.y(), origin.z() + t / takeoff_duration * (target_height - origin.z())));
                            desired_pose.setRotation(takeoff_rotation);
                            publishDesiredState(desired_pose, velocity, acceleration, desired_state_pub, br);        
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
                            publishDesiredState(desired_pose, velocity, acceleration, desired_state_pub, br); 
                        }
                        if ( t > takeoff_duration + cruise_duration)
                        {
                            reachedcave = true;
                        }
                        
                        
                        
                        ros::spinOnce();
                        loop_rate.sleep();
                        //++count;
                    }
                }
                break;

                case 3:
                //CODE
                break;



                case 4:
                {
                   	velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        		velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        
       	        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        		acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;
        		
                	bool landed = false;
                	bool first = true; 
                       
                       while (!landed){  
                            if(first){
                               desired_pose.setOrigin(tf::Vector3(actual_coordinates.x(), actual_coordinates.y(), actual_coordinates.z() - 0.2));

                    		publishDesiredState(desired_pose, velocity, acceleration, desired_state_pub, br);
                    		first = false;
                    		previous_coordinates = actual_coordinates;
                            }
                            else{
                                if(previous_coordinates.x() != actual_coordinates.x() || 
           		            previous_coordinates.y() != actual_coordinates.y() || 
            		            previous_coordinates.z() != actual_coordinates.z()) {
            		            
                               desired_pose.setOrigin(tf::Vector3(actual_coordinates.x(), actual_coordinates.y(), actual_coordinates.z() - 0.2));

                    		 publishDesiredState(desired_pose, velocity, acceleration, desired_state_pub, br);
                    		 previous_coordinates = actual_coordinates;
                    		 }
                    		 else{
                    		 	landed = true;
                    		 	
                    		 		
                    		 }
                    		
                            }

                       }
                         }
                   break;

                case 5:
                //CODE
                break;
            }
        }
    }



  void onCurrentState(const nav_msgs::Odometry& cur_state){  
     tf::Vector3 position(cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z);
     actual_coordinates = position;

       
  }
  
  
 
    void publishDesiredState(const tf::Transform& desired_pose,
                            const geometry_msgs::Twist& velocity,
                            const geometry_msgs::Twist& acceleration,
                            ros::Publisher& desired_state_pub,
                            tf::TransformBroadcaster& br)
    {
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

        //std::stringstream ss;
        //ss << "Trajectory Position"
        //<< " x:" << desired_pose.getOrigin().x()
        //<< " y:" << desired_pose.getOrigin().y()
        //<< " z:" << desired_pose.getOrigin().z();
        //ROS_INFO("%s", ss.str().c_str());

    #if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(), "world", "av-desired"));
    #endif
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    TrajPublisher Traj_publisher;
    ros::spin(); 
    return 0;
}
