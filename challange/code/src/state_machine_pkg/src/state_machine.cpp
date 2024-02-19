#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include <service_pkg/stop_service.h>
#include <service_pkg/switch_autonomous_state.h>
class State_Machine{
    // Node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_relative_;

    //Timer controll the main node
    ros::Timer timermission;

    // Mission States
    // 1- Initial state
    // 2- Predefined route to cave (TakeOff and Predefined route to cave entrance)
    // 3- Autonomous exploration of the cave
    // 4- Landing drone
    // 5- Landed
    int mission_state;

    // True when quadrotor reaches expexted_height
    bool autonomous_check;

    // True when server send the massage
    bool landing_check;

    // For checking landed
    bool landed;

    bool call_autonomous_state_service;
    bool success;
    // Specified in State Machine Launch file
    double expected_height_;
    double landed_height_;

    ros::Publisher drone_state_pub; 
    std_msgs::Int64 mission_state_msgs;
    
    
    ros::Subscriber current_state_subscriber;
    
    ros::ServiceClient switch_client;
    service_pkg::switch_autonomous_state srv;
    ros::ServiceServer land_server;


public:
    State_Machine
    {   
        // set initial state flag
        mission_state = 1;
        launch_flag = false;
        autonomous_check = false;
        landing_check = false;
        landed = false;

        expected_height_ = expected_height;
        landed_height_ = landed_height;

        call_autonomous_state_service = true;
        success = false;
        // publish to controller
        drone_state_pub = nh_.advertise<std_msgs::Int64>("state", 10);

        // subscribing the landing service from move base
        switch_client = nh_relative_.serviceClient<service_pkg::switch_autonomous_state>("/autonomous_state/switch_autonomous_state");
        
        // subscribing the landing service from move base
        land_server = nh_.advertiseService("stop_service", &State_Machine::stop_flag, this);
        
        // use timer to schedule events
        timermission = nh_.createTimer(ros::Duration(0.1), &State_Machine::state_machine_loop, this);
    }

    void state_machine_loop(const ros::TimerEvent& t){
        if(mission_state == 1){initial_state();}
        if(mission_state == 2){predefined_state();}
        if(mission_state == 3){autonomous_state();}
        if(mission_state == 4){landing_state();}
        if(mission_state == 5){landed_state();}
    }

    void initial_state()
    {
        mission_state_msgs.data = 1;
        drone_state_pub.publish(mission_state_msgs);

        if (launch_flag)
        {
            mission_state = 2;
        }
    }

    void predefined_state()
    {
        mission_state_msgs.data = 2;
        drone_state_pub.publish(mission_state_msgs);

        if (autonomous_check)
        {
            mission_state = 3;
        }
    }

    void autonomous_state()
    {

        mission_state_msgs.data = 3;
        drone_state_pub.publish(mission_state_msgs);

        // land_server = nh_.advertiseService("stop_service", &State_Machine::stop_flag, this);
        if (landing_check)
        {
            mission_state = 4;
        }
    }

    void landing_state()
    {
        mission_state_msgs.data = 4;
        drone_state_pub.publish(mission_state_msgs);
        ROS_INFO_STREAM_ONCE("autonomous_state end, landing");
        if (landed)
        {
            mission_state = 5;
        }
    }

    void landed_state()
    {
        mission_state_msgs.data = 5;
        drone_state_pub.publish(mission_state_msgs);
    }

    bool stop_flag(service_pkg::stop_service::Request &req, service_pkg::stop_service::Response &res)
    {
        
        landing_check = true;
        return true;
    }

    void current_height_check_callback(const nav_msgs::Odometry& cur_state)
    {
        current_height_ = cur_state.pose.pose.position.z;

        if ((current_height_ - expected_height_ < 0.3) && (current_height_ - expected_height_ > -0.3))
        {
            ROS_INFO_STREAM_ONCE("Quadrotor has reached expected height, ready for exploring");
            if (call_autonomous_state_service)
            {
                srv.request.switch_on = true;
                // switch_client.call(srv);
                success = switch_client.call(srv);
                if (success)
                {
                    ROS_INFO("call success");
                    call_autonomous_state_service = false;
                }
                else
                {
                    ROS_INFO("Try to call again");
                }
            }
            // if (success)
            //     {
            //         ROS_INFO("call success");
            //     }
            // else
            //     {
            //         ROS_INFO("Try to call again");
            //     }
            autonomous_check = true;
        }

        if ((current_height_ - landed_height_ < 0.3) && (current_height_ - landed_height_ > -0.3) && (mission_state == 4))
        {
            ROS_INFO_STREAM_ONCE("Quadrotor has reached landed height, landed");
            landed = true;
        }
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh_private("~");
    bool rotation_first;
    bool rotation_while_autonomous_state;
    State_Machine state_machine;

    ros::spin();
}