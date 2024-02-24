#include <ros/ros.h>
#include <state_machine_pkg/Transition.h>
#include <roslaunch/Process.h>
#include <roslaunch/ProcessLaunchInfo.h>
#include <roslaunch/ProcessManager.h>

roslaunch::ProcessManager process_manager;

bool handleTransition(state_transition::Transition::Request& req,
                      state_transition::Transition::Response& res) {
    if (req.new_state == "autonomous_exploration") {
        // Launch autonomous exploration nodes only once
        if (!process_manager.isStarted("autonomous_exploration_nodes")) {
            process_manager.startProcess("roslaunch", "state_machine_pkg predefined2autonomous.launch");
        }
        res.success = true;
    } else if (req.new_state == "land") {
        // Terminate autonomous exploration nodes if running
        process_manager.shutdownProcess("roslaunch", "state_machine_pkg predefined2autonomous.launch");

        // Add logic to land the drone here
        res.success = true;
    } else {
        res.success = false;
    }
    return true;
}