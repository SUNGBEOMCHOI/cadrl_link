#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <fstream>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <math.h>
#include <pthread.h>



bool collect_request;
static volatile bool continue_collection = true;
double lati_point=0, longi_point=0, lati_last=0, longi_last=0;
double min_coord_change = 10 * pow(10,-6);

static void* keyboardInterrupt(void*)
{
    while (continue_collection){
		if (std::cin.get()=='q'){
			continue_collection = false;
		}
	}
}

void filtered_gps(const sensor_msgs::NavSatFix gps_msg)
{
		lati_point = gps_msg.latitude;
		longi_point = gps_msg.longitude;
}

int main(int argc, char** argv)
{
	//Initialize variables
		int numWaypoints = 0;
		std::string path_local;

    // Initialize node and time
		ros::init(argc, argv, "link_collect_gps"); //initiate node called collect_gps_waypoints
		ros::NodeHandle n;
		ros::Time::init();
		ros::Time time_last;
		ros::Time time_current;
		ros::Duration duration_min(5);
    
	//init pthread for keyboard interrupt
		pthread_t thId;
		(void) pthread_create(&thId, 0, keyboardInterrupt, 0);

    //Initiate subscribers
		ros::Subscriber sub_gps = n.subscribe("/navsat/fix", 100, filtered_gps);
		ROS_INFO("Initiated collect_gps_waypoints node");

    //Read file path and create/open file
    	ros::param::get("/link_waypoint_nav/coordinates_file", path_local);
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
				double difference_lat = abs((lati_point - lati_last)*pow(10,6))*pow(10,-6);
				double difference_long = abs((longi_point - longi_last)*pow(10,6))*pow(10,-6);

				if( (difference_lat > min_coord_change) || (difference_long > min_coord_change))
				{
					//write waypoint
					ROS_INFO("You have collected another waypoint!");
					std::cout << std::endl;
					numWaypoints++;
					coordFile << std::fixed << std::setprecision(8) << lati_point << " " << longi_point << std::endl;
					lati_last = lati_point;
					longi_last = longi_point;
				}

				else
				{//do not write waypoint
					ROS_WARN("Waypoint not saved, you have not moved enough");
					ROS_WARN("New Latitude: %f   Last Latitude: %f \n", lati_point, lati_last );
					ROS_WARN("New Longitude: %f   Last Longitude: %f \n", longi_point, longi_last );
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
