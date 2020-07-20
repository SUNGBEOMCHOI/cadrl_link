#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <robot_localization/navsat_conversions.h>
#include <math.h>

double utm_x = 0, utm_y = 0;
int count = 0;
bool trans_init = false, convergeDone = false;
float lat_list [10],long_list [10], yaw_offset, sum_lat, sum_long, average_lat, average_long;

void gpsConverge(float lat_val, float long_val){
  if (count<10){
      ROS_INFO("Received lat : %f, long : %f \n",lat_val, long_val);
      lat_list[count] = lat_val;
      long_list[count] = long_val;
      count++;
      if (count==10){
        ROS_INFO("Average calculation started!");
        for (int i=0; i<10; i++){
          sum_lat+=lat_list[i];
          sum_long+=long_list[i];
        }
        average_lat += sum_lat/10;
        average_long += sum_long/10;
        ROS_INFO("Average lat : %f, Average long : %f \n\nGPS Pose estimate Done.\n",average_lat, average_long);
        convergeDone=true;
      }
  }
}

void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg){

  ros::param::get("/link_waypoint_nav/navsat_transform/yaw_offset", yaw_offset);
  static tf::TransformBroadcaster br;
  std::string utm_zone;
  tf::Transform transform;
  if (!isnan(msg->latitude)){

    gpsConverge(msg->latitude, msg->longitude);

    if (!trans_init && convergeDone){
      RobotLocalization::NavsatConversions::LLtoUTM(average_lat, average_long, utm_y, utm_x, utm_zone);
      trans_init = true;
    }

    transform.setOrigin( tf::Vector3(utm_x, utm_y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_offset);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "local_map"));
  }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::NavSatFix>("/navsat/fix", 10, &gpsCallback);
    ros::spin();
    return 0;
}
