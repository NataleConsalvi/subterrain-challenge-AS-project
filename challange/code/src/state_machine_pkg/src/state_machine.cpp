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



class State_Machine{
    // Node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_relative_;

    ros::Subscriber desired_state_subscriber;

    //Timer controll the main node
    ros::Timer timermission;

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
    int founded_lights;

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
        founded_lights = 0;
        landed = false;

        call_autonomous_state_service = true;
        success = false;
        // publish to controller
        drone_state_pub = nh_.advertise<std_msgs::Int64>("state", 10);
        
        // use timer to schedule events
        timermission = nh_.createTimer(ros::Duration(0.1), &State_Machine::state_machine_loop, this);

        // Subscribe to the desired state topic
        desired_state_subscriber = nh_.subscribe("/airsim_ros_node/desired_state2", 1, &State_Machine::desired_state_callback, this);
    }

    void state_machine_loop(const ros::TimerEvent& t){
        if (!init) {
            ros::Duration(20.0).sleep();  // Sleep for 20 seconds
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

    void autonomous_finished(const trajectory_msgs::MultiDOFJointTrajectoryPoint& desired_state)
    {
        founded_lights++;
        if(founded_lights>=4)
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
            std::string command = "roslaunch state_machine_pkg predefined2autonomous.launch";

            // Dynamically obtain the home directory of the user
            const char* homeDir = std::getenv("HOME");
            if (homeDir == nullptr) {
                std::cerr << "Error: Unable to determine the home directory." << std::endl;
            }

            // Combine commands to be executed in a new terminal with changed working directory
            std::string fullCommand = "gnome-terminal --working-directory=" + std::string(homeDir) + " -- bash -c '"
                                    + intoworkspace + " && " + source + " && " + command + "; exec bash'";
            
            system(fullCommand.c_str());
            ROS_INFO("Opened new terminal");
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

