#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include <cstdlib>
#include <fstream>
#include <ros/package.h>
#include "visualization_msgs/MarkerArray.h"



class State_Machine{
    // Node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_relative_;

    ros::Subscriber desired_state_subscriber;

    ros::Subscriber detected_lights;

    //Timer controll the main node
    ros::Timer timermission;

    
    ros::Publisher carrot_trajectory_pub;

    ros::Subscriber Ready_Trajectory;
    ros::Subscriber Point_reached;
    

    // Mission States
    // 1- Predefined route to cave (TakeOff and Predefined route to cave entrance)
    // 2- Autonomous exploration of the cave
    // 3- Landing drone
    // 4- Landed
    
    int mission_state;

    bool init;
    
    // True when quadrotor reaches expexted_height
    bool autonomous_check;

    // True when server send the massage
    bool landing_check;

    // For checking landed
    bool landed;

    bool call_autonomous_state_service;
    bool success;

    ros::Publisher drone_state_pub; 
    std_msgs::Int64 mission_state_msgs;

  
public:
    State_Machine()
    {   
        // set initial state flag
        ROS_INFO("State machine initialized.");
        mission_state = 1;
        init = false;

        //launch_flag = false;
        autonomous_check = false;
        landing_check = false;
        landed = false;

        call_autonomous_state_service = true;
        success = false;
        // publish to controller
        drone_state_pub = nh_.advertise<std_msgs::Int64>("state", 10);

        carrot_trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("red/carrot/trajectory", 1);
        
        // use timer to schedule events
        timermission = nh_.createTimer(ros::Duration(0.1), &State_Machine::state_machine_loop, this);

        // Subscribe to the desired state topic
        desired_state_subscriber = nh_.subscribe("red/position_hold/trajectory", 1, &State_Machine::desired_state_callback, this);

        detected_lights = nh_.subscribe("/perception_params/object_vis_out_topic", 10, &State_Machine::autonomous_finished, this);
        
        Ready_Trajectory = nh_.subscribe("red/ready_trajectory",1, &State_Machine::TrajReadyCallBack, this);

        Point_reached = nh_.subscribe("red/exploration/point_reached",1, &State_Machine::PointReachedCallBack, this);  
        
    }

    void PointReachedCallBack(const std_msgs::Bool& msg){

        if (mission_state == 2 && msg.data) {
            
            std::string intoworkspace = "cd subterrain-challenge-AS-project/challange/code/";
            std::string source = "source devel/setup.bash";
            

            std::string command1 = "export UAV_NAMESPACE=red";
            std::string command2 = "rosservice call /red/exploration/toggle \"data: true\"";

            const char* homeDir = std::getenv("HOME");
            if (homeDir == nullptr) {
                std::cerr << "Error: Unable to determine the home directory." << std::endl;
            }

            // Combine commands to be executed in a new terminal with changed working directory
            std::string fullCommand = "gnome-terminal --working-directory=" + std::string(homeDir) + " -- bash -c '"
                                    + intoworkspace + " && " + source + " && " + command1 + " && " + command2 + "; exit";
            
            
            system(fullCommand.c_str());
        }
    }


    void TrajReadyCallBack(const std_msgs::Bool& msg){

        if (mission_state == 2 && msg.data) {
            
            std::string intoworkspace = "cd subterrain-challenge-AS-project/challange/code/";
            std::string source = "source devel/setup.bash";
            

            std::string command1 = "export UAV_NAMESPACE=red";
            std::string command2 = "rosservice call /red/confirm_trajectory \"data: true\"";

            const char* homeDir = std::getenv("HOME");
            if (homeDir == nullptr) {
                std::cerr << "Error: Unable to determine the home directory." << std::endl;
            }

            // Combine commands to be executed in a new terminal with changed working directory
            std::string fullCommand = "gnome-terminal --working-directory=" + std::string(homeDir) + " -- bash -c '"
                                    + intoworkspace + " && " + source + " && " + command1 + " && " + command2 + "; exec bash'; exit";
            
            
            system(fullCommand.c_str());
        }
    }

    void state_machine_loop(const ros::TimerEvent& t){
        if (!init) {
            ros::Duration(30.0).sleep();  // Sleep for 20 seconds
            init = true;
        }
        if(mission_state == 1){predefined_state();}
        if(mission_state == 2){autonomous_state();}
        if(mission_state == 3){landing_state();}
        if(mission_state == 4){landed_state();}
    }

    void desired_state_callback(const trajectory_msgs::MultiDOFJointTrajectoryPoint& desired_state)
    {
        // Assuming the cave entrance position is (cave_x, cave_y, cave_z)
        double cave_x = -321.0;
        double cave_y = 10.0;
        double cave_z = 15.0;

        // Extract the x, y, z coordinates of the desired state
        double x = desired_state.transforms[0].translation.x;
        double y = desired_state.transforms[0].translation.y;
        double z = desired_state.transforms[0].translation.z;

        // Calculate the distance between the desired state and the cave entrance
        double distance_to_cave = std::sqrt(std::pow(x - cave_x, 2) + std::pow(y - cave_y, 2) + std::pow(z - cave_z, 2));

        // Set autonomous_check to true if the distance is less than or equal to 0.2
        autonomous_check = (distance_to_cave <= 0.2);

    }

    void autonomous_finished(const visualization_msgs::MarkerArray::ConstPtr& msg)
    {
        if (msg->markers.size() == 4)
        {
            landing_check = true;
        }
    }

     void predefined_state()
    {
        mission_state_msgs.data = 1;
        drone_state_pub.publish(mission_state_msgs);

        if (autonomous_check)
        {
            mission_state = 2;

            std::string intoworkspace = "cd subterrain-challenge-AS-project/challange/code/";
            std::string source = "source devel/setup.bash";
            std::string command11 = "export UAV_NAMESPACE=red";
            std::string command22 = "roslaunch state_machine_pkg predefined2autonomous.launch";

            // Dynamically obtain the home directory of the user
            const char* homeDir = std::getenv("HOME");
            if (homeDir == nullptr) {
                std::cerr << "Error: Unable to determine the home directory." << std::endl;
            }

            // Combine commands to be executed in a new terminal with changed working directory
            std::string fullCommand = "gnome-terminal --working-directory=" + std::string(homeDir) + " -- bash -c '"
                                    + intoworkspace + " && " + source + " && " + command11 + " && " + command22 + "; exec bash'";
            
            system(fullCommand.c_str());
            ROS_INFO("Opened new terminal");

            ros::Duration(20.0).sleep();

            trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
            msg.transforms.resize(1);
            msg.transforms[0].translation.x = -320.983182101615;
            msg.transforms[0].translation.y = 9.996668873146382;
            msg.transforms[0].translation.z = 14.978115666412771;
            msg.transforms[0].rotation.x = 0.0017930719768628478;
            msg.transforms[0].rotation.y = -0.00036625718348659575;
            msg.transforms[0].rotation.z = 0.8939921259880066;
            msg.transforms[0].rotation.w = -0.4480791985988617;
                
            msg.velocities.resize(1);
            msg.velocities[0].linear.x = 0.0;
            msg.velocities[0].linear.y = 0.0;
            msg.velocities[0].linear.z = 0.0;

            msg.accelerations.resize(1);
            msg.accelerations[0].linear.x = 0.0;
            msg.accelerations[0].linear.y = 0.0;
            msg.accelerations[0].linear.z = 0.0;

            carrot_trajectory_pub.publish(msg);

            std::string command111 = "export UAV_NAMESPACE=red";
            std::string command222 = "rosservice call /red/exploration/toggle \"data: true\"";

            // Combine commands to be executed in a new terminal with changed working directory
            std::string fullCommand2 = "gnome-terminal --working-directory=" + std::string(homeDir) + " -- bash -c '"
                                    + intoworkspace + " && " + source + " && " + command111 + " && " + command222 + "; exec bash' & disown; exit";
            
            system(fullCommand2.c_str());
        }
    }

     void autonomous_state()
    {
        mission_state_msgs.data = 2;
        drone_state_pub.publish(mission_state_msgs);
        if (landing_check)
        {
            mission_state = 3;

            const char* roslaunchProcessName = "roslaunch state_machine_pkg predefined2autonomous.launch";

            // Construct the command to find and kill the roslaunch process
            std::string killCommand = "pkill -f \"" + std::string(roslaunchProcessName) + "\"";

            // Execute the command
            system(killCommand.c_str());
        }
    }

    void landing_state()
    {
        mission_state_msgs.data = 3;
        drone_state_pub.publish(mission_state_msgs);
        ROS_INFO_STREAM_ONCE("autonomous_state end, landing");
        if (landed)
        {
            mission_state = 4;
        }
    }

    void landed_state()
    {
        mission_state_msgs.data = 4;
        drone_state_pub.publish(mission_state_msgs);
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh_private("~");
    State_Machine state_machine;

    ros::spin();
}