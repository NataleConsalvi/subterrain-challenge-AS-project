
#include <random>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <limits>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>





double MAX_ITERATION = 2000;// The maximum number of iterations for the RRT algorithm.
double MAX_DISTANCE = 2; // The maximum distance a node can move in a single iteration.
double GOAL= 0; //A flag to indicate whether the goal has been reached.


tf::Vector3 current_state;
//------------------------------------------------------------------------------
void currentStateCallback (const nav_msgs::Odometry& cur_state){

        current_state.setX(cur_state.pose.pose.position.x);
    current_state.setY(cur_state.pose.pose.position.y);
    current_state.setZ(cur_state.pose.pose.position.z);
}
struct Node
{

    double posX;
    double posY;
    double posZ;
    Node *prev;
    Node *next;
};
//Definition of a Node structure to represent a point in 3D space. 
//Each node has X, Y, Z coordinates, and pointers to the previous and 
//next nodes in the path.

//------------------------------------------------------------------------------

/*Definition of the RRT class, which encapsulates the 
Rapidly-exploring Random Tree (RRT) algorithm for path planning in a 3D space.
 It includes private members for the start and goal nodes, 
as well as a vector to store nodes in the tree.*/
class RRT
{

private:
    Node *start;
    Node *goal;
    std::vector<Node *> rrtNodes;

public:
    RRT(const tf::Vector3 &initialPosition, const tf::Vector3 &goalPosition)
    {

        Node *node = new Node;
        this->start = node;
        start->posX = initialPosition.getX();
        start->posY = initialPosition.getY();
        start->posZ = initialPosition.getZ();
        start->prev = nullptr;
        rrtNodes.push_back(node);

        node = new Node;
        this->goal = node;
        goal->posX = goalPosition.getX();
        goal->posY = goalPosition.getY();
        goal->posZ = goalPosition.getZ();
    }

    //------------------------------------------------------------------------------

/*Implementation of a callback function to handle OctoMap messages. 
It converts the OctoMap message to an OctoMap tree, updating the octree
 variable.
 NB: an octomap::OcTree represents a 3D occupancy grid*/
void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    octomap::AbstractOcTree *tree = octomap_msgs::fullMsgToMap(*msg); //convert the OctoMap message into an octomap::AbstractOcTree
    octree = dynamic_cast<octomap::OcTree *>(tree); //dynamic_cast in order to specifically cast it to an OcTree. 
}

    bool checkObstacles(double x1, double y1, double x2, double y2, std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> obs)
    {
           // Convert coordinates to octomap::point3d
    octomap::point3d start(x1, y1, z1);
    octomap::point3d end(x2, y2, z2);

    // Check for obstacles in the path
    octomap::KeyRay key_ray;
    if (octree->computeRayKeys(start, end, key_ray)) // calculate the keys of the octomap 
    //nodes intersected by the ray between points start and end.
    //(each node in the tree corresponds to a voxel in 3D space
    //(keys are unique identifiers assigned to these nodes)).
    //The resulting key_ray contain a sequence of keys, 
    //each corresponding to a node in the Octomap
    {
        for (auto &key : key_ray)
        {
            //for each key in the ray, it searches for the corresponding OctoMap node
            octomap::OcTreeNode *node = octree->search(key);
            //if the node exists and is occupied, it indicates a collision.
            if (node && octree->isNodeOccupied(node))
            {
                // Collision detected
                return true;
            }
        }
    }

    // No collision
    return false;
}
    }

    //------------------------------------------------------------------------------

    Node *checkNearestNode(Node *new_node)
    {

        std::vector<double> tempX;
        std::vector<double> tempY;
        std::vector<double> tempZ;

        Node *near_node = new Node;
        double minDistance = std::numeric_limits<double>::max();
        double corrX = 0.0;
        double corrY = 0.0;
        double corrZ = 0.0;
        bool check_obstacle;

        for (auto &ii : rrtNodes)
        {

            double distance = std::sqrt(std::pow((new_node->posX - ii->posX), 2) + std::pow((new_node->posY - ii->posY), 2), std::pow((new_node->posZ - ii->posZ), 2));
            if (distance < minDistance)
            {

                minDistance = distance;
                near_node = ii;
            }
        }

        double dx = new_node->posX - near_node->posX;
        double dy = new_node->posY - near_node->posY;
        double dz = new_node->posZ - near_node->posZ;

        double angleXY = std::atan2(dy, dx) * 180 / M_PI;
        double angleXZ = std::atan2(dz, dx) * 180 / M_PI;

    if (minDistance > MAX_DISTANCE)
    {
        // Adjust new node position if it's too far from the nearest node
        corrX = near_node->posX + std::cos(angleXY) * MAX_DISTANCE;
        corrY = near_node->posY + std::sin(angleXY) * MAX_DISTANCE;
        corrZ = near_node->posZ + std::sin(angleXZ) * MAX_DISTANCE;
    }

    if (minDistance <= MAX_DISTANCE)
    {
        // Keep the new node position if it's within the maximum distance
        corrX = new_node->posX;
        corrY = new_node->posY;
        corrZ = new_node->posZ;
    }

    // Step 5: Check for obstacles and adjust position accordingly
    if (rrtNodes.size() > 0)
    {
        check_obstacle = checkObstacles(near_node->posX, near_node->posY, near_node->posZ, corrX, corrY, corrZ, std::make_tuple(obsXmin, obsYmin, obsXmax, obsYmax));
    }

    new_node->posX = corrX;//equivalent of (*new_node).posX = corrX; the operator-> is used to access the members (posX,Y,Z) of the object pointed to by "new_node".
    new_node->posY = corrY;
    new_node->posZ = corrZ;

    // Step 6: Update the RRT with the new_node and return it
    near_node->next = new_node;
    new_node->prev = near_node;

    if (rrtNodes.size() == 0)
    {
        new_node->prev = start;
    }

    if (check_obstacle == 0)
    {
        rrtNodes.push_back(new_node);
    }

    // Step 7: If the new node is the goal, print the path and set the GOAL flag
    if (((double)new_node->posX == (double)this->goal->posX) &&
        ((double)new_node->posY == (double)this->goal->posY) &&
        ((double)new_node->posZ == (double)this->goal->posZ))
    {
        
        GOAL = 1;

        while (new_node->prev != nullptr)
        {
            
            new_node = new_node->prev;
            tempX.push_back(new_node->posX);
            tempY.push_back(new_node->posY);
            tempZ.push_back(new_node->posZ);
        }

        
    }

    return new_node;
}
    //------------------------------------------------------------------------------

//Purpose: perform iterations of the RRT algorithm by generating random nodes,
//connecting them to the nearest nodes in the existing RRT, and checking if
//the goal is reached.
    double lookForPath()
    {

        //Initialize a random number generator.
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist100(0, 20); 

        for (double i = 0; i < MAX_ITERATION; i++)
        {
            //Generate a random node within the specified bounds
            Node *random_node = new Node;
            Node *last_node = new Node;

            double randX = dist100(rng);
            double randY = dist100(rng);
            double randZ = dist(rng);

            random_node->posX = randX;
            random_node->posY = randY;
            random_node->posZ = randZ;
        //Call to "checkNearestNode to find the nearest node in the RRT to the randomly generated node"
            last_node = checkNearestNode(random_node);

            if (GOAL == 1)
            {
                goal->prev = last_node;
                //std::cout << last_node->posX << " :: " << last_node->posY << std::endl;

                return -1;
            }
        }
        return 1;
    }

    //------------------------------------------------------------------------------

    void printRRT(RRT &rrt)
    {

        Node *node = rrt.start;
        std::cout << node->posX << " : " << node->posY << std::endl;
        node = rrt.goal;
        std::cout << node->posX << " : " << node->posY << std::endl;
    }


//------------------------------------------------------------------------------

int main()
{
    ros::init(argc, argv, "rrt_planner_node");
    ros::NodeHandle nh;
 ros::Subscriber current_state_est = n.subscribe("current_state_est", 1, currentStateCallback);

     ros::spinOnce();  // Allow one spin to get the initial Odometry message
     tf::Vector3 goalPosition(-45,11,10);
     RRT rrt(current_state, goalPosition);
    //rrt.printRRT(rrt);
    double checkpath = rrt.lookForPath();

    ros::Rate rate(10); 

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}