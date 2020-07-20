#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>


// initialize variables

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction>
MoveBaseClient; //create a type definition for a client called MoveBaseClient

std::vector <std::pair<double, double>> waypointVect;
std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
geometry_msgs::PointStamped map_point, map_next;

int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
double x_Goal, y_Goal, xNext, yNext;
std::string utm_zone;
std::string path_local, path_abs;

int countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("link_waypoint_nav") + path_local;
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double pos_x = 0;
        while(!fileCount.eof())
        {
            fileCount >> pos_x;
            ++count;
        }
        count = count - 1;
        numWaypoints = count / 2;
        ROS_INFO("%.0f Simulation waypoints were read", numWaypoints);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
    return numWaypoints;
}

std::vector <std::pair<double, double>> getWaypoints(std::string path_local)
{
    double pos_x = 0, pos_y = 0;

    path_abs = ros::package::getPath("link_waypoint_nav") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < numWaypoints; i++)
    {
        fileRead >> pos_x;
        fileRead >> pos_y;
        waypointVect.push_back(std::make_pair(pos_x, pos_y));
    }
    fileRead.close();

    //Outputting vector
    ROS_INFO("The following Simulation Waypoints have been set:");
    for(std::vector < std::pair < double, double >> ::iterator iterDisp = waypointVect.begin(); iterDisp != waypointVect.end();
    iterDisp++)
    {
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    }
    return waypointVect;
}

move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next, bool last_point)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    if(last_point == false)
    {
        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;

        // Calculate quaternion
        float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
        float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
        rot_euler.getRotation(rot_quat);

        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }
    else
    {
        goal.target_pose.pose.orientation.w = 1.0;
    }

    return goal;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_waypoint_sim"); //initiate node called send_waypoint_sim
    ros::NodeHandle nh;
    ROS_INFO("Initiated send_waypoint_sim node");
    MoveBaseClient ac("/move_base", true);
    //construct an action client that we use to communication with the action named move_base.
    //Setting true is telling the constructor to start ros::spin()

    // Initiate publisher to send end of node message
    ros::Publisher pubWaypointNodeEnded = nh.advertise<std_msgs::Bool>("/link_waypoint_nav/waypoint_following_status", 100);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(100.0)))
    {
        wait_count++;
        if(wait_count > 3)
        {
            ROS_ERROR("move_base action server did not come up, killing send_waypoint_sim node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Get Longitude and Latitude goals from text file

    //Count number of waypoints
    ros::param::get("/link_waypoint_nav/coordinates_file", path_local);
    numWaypoints = countWaypointsInFile(path_local);

    //Reading waypoints from text file and output results
    waypointVect = getWaypoints(path_local);


    // Iterate through vector of waypoints for setting goals
    for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
    {
        //Setting goal:
        x_Goal = iter->first;
        y_Goal = iter->second;
        bool final_point = false;

        //set next goal point if not at last waypoint
        if(iter < (waypointVect.end() - 1))
        {
            iter++;
            xNext = iter->first;
            yNext = iter->second;
            iter--;
        }
        else //set to current
        {
            xNext = iter->first;
            yNext = iter->second;
            final_point = true;
        }

        ROS_INFO("Received pose_x goal:%.8f", x_Goal);
        ROS_INFO("Received pose_y goal:%.8f", y_Goal);
        
        //Transform UTM to map point in odom frame
        map_point.point.x = x_Goal;
        map_point.point.y = y_Goal;
        map_next.point.x = xNext;
        map_next.point.y = yNext;

        //Build goal to send to move_base
        move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_next, final_point); //initiate a move_base_msg called goal

        // Send Goal
        ROS_INFO("Sending goal");
        ac.sendGoal(goal); //push goal to move_base node

        //Wait for result
        ac.waitForResult(); //waiting to see if move_base was able to reach goal

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Link has reached its goal!");
            //switch to next waypoint and repeat
        }
        else
        {
            ROS_ERROR("Link was unable to reach its goal. GPS Waypoint unreachable.");
            ROS_INFO("Exiting node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
    } // End for loop iterating through waypoint vector

    ROS_INFO("Link has reached all of its goals!!!\n");
    ROS_INFO("Ending node...");

    // Notify joy_launch_control that waypoint following is complete
    std_msgs::Bool node_ended;
    node_ended.data = true;
    pubWaypointNodeEnded.publish(node_ended);

    ros::shutdown();
    ros::spin();
    return 0;
}
