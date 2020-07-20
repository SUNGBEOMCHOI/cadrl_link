#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

double _PreviousLeftEncoderCounts = 0.0;
double _PreviousRightEncoderCounts = 0.0;
ros::Time current_time, last_time;
double DistancePerCount = (3.14159265 * 0.2032); //the wheel diameter is 0.2032m
//final odometric datas
double x;
double y;
double th;
double v_left;//left motor speed
double v_right;//right motor speed
double vth;//angular velocity of robot
double deltaLeft;//no of ticks in left encoder since last update
double deltaRight;//no of ticks in right encoder since last update
double dt;
double delta_distance;//distance moved by robot since last update
double delta_th;//corresponging change in heading
double delta_x ;//corresponding change in x direction
double delta_y;//corresponding change in y direction
bool trans_init = false;
#define PI 3.14159265
#define TwoPI 6.28318531


void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{
 current_time = ros::Time::now();
 deltaLeft = ticks->x - _PreviousLeftEncoderCounts;
 deltaRight = ticks->y - _PreviousRightEncoderCounts;
 dt = (current_time - last_time).toSec();
 v_left = deltaLeft * DistancePerCount/dt;
 v_right = deltaRight * DistancePerCount/dt;
 delta_distance=0.5f*(double)(deltaLeft+deltaRight)*DistancePerCount;
 //delta_th = (double)(deltaRight-deltaLeft)*DistancePerCount/0.5f; //Distance between the two wheels is 0.5m
 delta_x = delta_distance*(double)cos(th);
 delta_y = delta_distance*(double)sin(th);
 x += delta_x;
 y += delta_y;
 //th += delta_th;
 //if (th > PI)
 //th -= TwoPI;
 //else
 //if ( th <= -PI)
 //th += TwoPI;
 _PreviousLeftEncoderCounts = ticks->x;
 _PreviousRightEncoderCounts = ticks->y;
 last_time = current_time;
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
 tf::Quaternion q(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w);
 tf::Matrix3x3 m(q);
 double roll, pitch, yaw;
 m.getRPY(roll, pitch, yaw);
 th=yaw;
 if(th <= -PI){
  th += TwoPI;
 }
 //if (!trans_init){
 // th=yaw;
 // trans_init=true;
 //}
 
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "odometry_publisher");
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("pos", 100, WheelCallback);
 ros::Subscriber subi = n.subscribe("/imu/data", 100, ImuCallback);
 ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
 tf::TransformBroadcaster odom_broadcaster;
 ros::Rate r(1);

 while(n.ok()){
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    //odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x=delta_x/dt;
    odom.twist.twist.linear.y=delta_y/dt;
    odom.twist.twist.angular.z = delta_th/dt;

    ROS_INFO("Position x = %lf and Position y = %lf ",x,y);

    //publish the message
    odom_pub.publish(odom);
    last_time = current_time;
    ros::spinOnce();
    r.sleep();
    }
}
