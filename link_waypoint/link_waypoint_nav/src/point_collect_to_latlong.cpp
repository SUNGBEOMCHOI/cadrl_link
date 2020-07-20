#include <ros/ros.h>
#include <ros/package.h>
#include <robot_localization/navsat_conversions.h>
#include <utility>
#include <fstream>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <math.h>
#include <pthread.h>



bool collect_request;
static volatile bool continue_collection = true;
bool rviz_point_request = false;
double pose_x=0, pose_y=0, pose_x_last=0, pose_y_last=0;
double rviz_x=0, rviz_y=0, utm_x=0, utm_y=0;
double min_coord_change = 10 * pow(10,-6);
double utm_trans_x, utm_trans_y;
std::string utm_zone = "52S";

static void* keyboardInterrupt(void*)
{
    while (continue_collection){
		if (std::cin.get()=='q'){
			continue_collection = false;
		}
	}
}

void get_pose_goal(const nav_msgs::Odometry::ConstPtr &pose_goal)
{
    utm_x = (pose_goal->pose.pose.position.x)+utm_trans_x;
    utm_y = (pose_goal->pose.pose.position.y)+utm_trans_y;
	//convert utm to latlong
    RobotLocalization::NavsatConversions::UTMtoLL(utm_y, utm_x, utm_zone, pose_x, pose_y);
}

void get_rviz_goal(const geometry_msgs::PoseStamped::ConstPtr &rviz_goal)
{
    rviz_x = rviz_goal->pose.position.x;
    rviz_y = rviz_goal->pose.position.y;
    rviz_point_request = true;
}

int main(int argc, char** argv)
{
	//Initialize variables
		int numWaypoints = 0;
		std::string path_local;

    // Initialize node and time
		ros::init(argc, argv, "point_collect_latlong"); //initiate node called collect_gps_waypoints
		ros::NodeHandle n;
		ros::Time::init();
		ros::Time time_last;
		ros::Time time_current;
		ros::Duration duration_min(5);

	//init pthread for keyboard interrupt
		pthread_t thId;
		(void) pthread_create(&thId, 0, keyboardInterrupt, 0);

    //Initiate subscribers
        ros::Subscriber sub_odometry = n.subscribe("/odometry/filtered", 100, get_pose_goal);
        //ros::Subscriber sub_simple_goal = n.subscribe("/move_base_simple/goal", 100, get_rviz_goal);
		ROS_INFO("Initiated point_collect_latlong node");

    //Read file path and create/open file
    	ros::param::get("/link_waypoint_nav/coordinates_file", path_local);
		ros::param::get("/link_waypoint_nav/utm_x_trans", utm_trans_x);
		ros::param::get("/link_waypoint_nav/utm_y_trans", utm_trans_y);

		std::string path_abs =  ros::package::getPath("link_waypoint_nav") + path_local;	
		std::ofstream coordFile (path_abs.c_str());
		ROS_INFO("Saving coordinates to: %s", path_abs.c_str());


	if(coordFile.is_open())
	{
		while(continue_collection)
		{
			ros::spinOnce();
			time_current = ros::Time::now();
			if((time_current - time_last > duration_min))
			{	
				// Check that there was sufficient change in position between points
				// This makes the move_base navigation smoother and stops points from being collected twice
				double difference_lat = abs((pose_x - pose_x_last)*pow(10,6))*pow(10,-6);
				double difference_long = abs((pose_y - pose_y_last)*pow(10,6))*pow(10,-6);

				if( (difference_lat > min_coord_change) || (difference_long > min_coord_change))
				{
					//write waypoint
					ROS_INFO("You have collected another waypoint!");
					std::cout << std::endl;
					numWaypoints++;
					coordFile << std::fixed << std::setprecision(8) << pose_x << " " << pose_y << std::endl;
					pose_x_last = pose_x;
					pose_y_last = pose_y;
				}
                
                else if (rviz_point_request){
                    //write rviz 2D Nav goal to coord
                    ROS_INFO("You have requested a rviz waypoint!");
					std::cout << std::endl;
					numWaypoints++;
					coordFile << std::fixed << std::setprecision(8) << rviz_x << " " << rviz_y << std::endl;
					pose_x_last = rviz_x;
					pose_y_last = rviz_y;
                    rviz_point_request = false;
                }

				else
				{//do not write waypoint
					ROS_WARN("Waypoint not saved, you have not moved enough");
					ROS_WARN("New pose_x (Latitude): %f   Last Latitude: %f \n", pose_x, pose_x_last );
					ROS_WARN("New pose_y (Longitude): %f   Last Longitude: %f \n", pose_y, pose_y_last );
				}
				time_last = time_current;
			}
			else{}
			ros::spinOnce();
		}

		(void) pthread_join(thId, NULL);
		coordFile.close();
		ROS_INFO("End request registered.");
	}
	else
	{
		ROS_ERROR("Unable to open file.");
		ROS_INFO("Exiting..");
	}

	ROS_INFO("Closed waypoint file, you have collected %d waypoints.", numWaypoints);
	ROS_INFO("Ending node...");

	ros::shutdown();
	return 0;
}
