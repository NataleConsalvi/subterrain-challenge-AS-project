#include "rrt_planner_3d.h"

void RRTPlanner3D::expandTree() {
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        // Generate a random node
        geometry_msgs::PoseStamped random_node = getRandomNode();

        // Find the nearest node in the tree to the random node
        std::pair<int, double> nearest_node = findNearestNode(random_node);

        // Expand the tree towards the random node
        geometry_msgs::PoseStamped new_node = expandTowards(nearest_node, random_node);

        // Check if the new node is close enough to the goal
        if (isGoalReached(new_node)) {
            // Publish the path and break the loop
            publishPath();
            break;
        }
    }
}

std::pair<int, double> RRTPlanner3D::findNearestNode(const geometry_msgs::PoseStamped& random_node) {
    double min_distance = std::numeric_limits<double>::max();
    int nearest_index = -1;

    for (int i = 0; i < tree.size(); ++i) {
        double distance = calculateDistance(tree[i], random_node);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_index = i;
        }
    }

    return std::make_pair(nearest_index, min_distance);
}

geometry_msgs::PoseStamped RRTPlanner3D::expandTowards(const std::pair<int, double>& nearest_node, 
                                                       const geometry_msgs::PoseStamped& random_node) {
    // Get the nearest node and calculate the unit vector towards the random node
    geometry_msgs::PoseStamped nearest_pose = tree[nearest_node.first];
    geometry_msgs::Vector3 direction;
    direction.x = random_node.pose.position.x - nearest_pose.pose.position.x;
    direction.y = random_node.pose.position.y - nearest_pose.pose.position.y;
    direction.z = random_node.pose.position.z - nearest_pose.pose.position.z;
    double length = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    direction.x /= length;
    direction.y /= length;
    direction.z /= length;

    // Extend the tree towards the random node
    geometry_msgs::PoseStamped new_node = nearest_pose;
    new_node.pose.position.x += direction.x * step_size;
    new_node.pose.position.y += direction.y * step_size;
    new_node.pose.position.z += direction.z * step_size;

    // Check if the new node is in an occupied space, if so, reset to nearest node
    octomap::point3d query(new_node.pose.position.x, new_node.pose.position.y, new_node.pose.position.z);
    octomap::OcTreeNode* result = octree->search(query);
    if (result && octree->isNodeOccupied(result)) {
        new_node = nearest_pose;
    }

    // Add the new node to the tree
    tree.push_back(new_node);

    // Publish the new node for visualization
    publishNode(new_node);

    return new_node;
}

bool RRTPlanner3D::isGoalReached(const geometry_msgs::PoseStamped& node) {
    double distance = calculateDistance(node, goal_pose);
    return distance < goal_tolerance;
}

double RRTPlanner3D::calculateDistance(const geometry_msgs::PoseStamped& pose1, 
                                       const geometry_msgs::PoseStamped& pose2) {
    // Euclidean distance between two poses
    return sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                pow(pose1.pose.position.y - pose2.pose.position.y, 2) +
                pow(pose1.pose.position.z - pose2.pose.position.z, 2));
}

void RRTPlanner3D::publishNode(const geometry_msgs::PoseStamped& node) {
    // Publish the node for visualization
    visualization_msgs::Marker marker;
    marker.header = node.header;
    marker.ns = "rrt_tree";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = node.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    pose_pub.publish(marker);
}

void RRTPlanner3D::publishPath() {
    // Publish the final path for visualization
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    path.poses = tree;
    path_pub.publish(path);
}
