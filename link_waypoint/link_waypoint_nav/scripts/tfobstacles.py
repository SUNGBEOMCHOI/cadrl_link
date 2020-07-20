#!/usr/bin/env python

import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from tf.transformations import quaternion_from_euler
from obstacle_detector.msg import Obstacles, CircleObstacle

def callback(data):
    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs
    frame_id = data.header.frame_id
    ob_list = data.circles
    new_ob_list = []
    for i in range(len(ob_list)):
        x = ob_list[i].center.x
        y = ob_list[i].center.y
        z = ob_list[i].center.z
        vx = ob_list[i].velocity.x
        vy = ob_list[i].velocity.y
        vz = ob_list[i].velocity.z
        r = ob_list[i].radius
        new_ob_list.append([x, y, z, vx, vy, vz, r])


    obstacle_msg = ObstacleArrayMsg()
    obstacle_msg.header.stamp.secs = secs
    obstacle_msg.header.stamp.nsecs = nsecs
    frame_id = "odom"
    obstacle_msg.header.frame_id = frame_id

    for i in range(len(new_ob_list)):
        obstacle_msg.obstacles.append(ObstacleMsg())
        obstacle_msg.obstacles[i].id = 1
        obstacle_msg.obstacles[i].header.stamp.secs = secs 
        obstacle_msg.obstacles[i].header.stamp.nsecs = nsecs
        obstacle_msg.obstacles[i].header.frame_id = frame_id
        obstacle_msg.obstacles[i].radius = new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points = [Point32(), Point32(), Point32(), Point32()]
        obstacle_msg.obstacles[i].polygon.points[0].x = new_ob_list[i][0] + new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points[0].y = new_ob_list[i][1] + new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points[0].z = 0
        obstacle_msg.obstacles[i].polygon.points[1].x = new_ob_list[i][0] + new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points[1].y = new_ob_list[i][1] - new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points[1].z = 0
        obstacle_msg.obstacles[i].polygon.points[2].x = new_ob_list[i][0] - new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points[2].y = new_ob_list[i][1] - new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points[2].z = 0
        obstacle_msg.obstacles[i].polygon.points[3].x = new_ob_list[i][0] - new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points[3].y = new_ob_list[i][0] + new_ob_list[i][6]
        obstacle_msg.obstacles[i].polygon.points[3].z = 0
        yaw = math.atan2(new_ob_list[i][4], new_ob_list[i][3])
        q = tf.transformations.quaternion_from_euler(0,0,yaw)
        obstacle_msg.obstacles[i].orientation = Quaternion(*q)
        obstacle_msg.obstacles[i].velocities.twist.linear.x = new_ob_list[i][3]
        obstacle_msg.obstacles[i].velocities.twist.linear.y = new_ob_list[i][4]
        obstacle_msg.obstacles[i].velocities.twist.linear.z = 0
        obstacle_msg.obstacles[i].velocities.twist.angular.x = 0
        obstacle_msg.obstacles[i].velocities.twist.angular.y = 0
        obstacle_msg.obstacles[i].velocities.twist.angular.z = 0
    pub.publish(obstacle_msg)

 
if __name__ == '__main__':
    try:
        rospy.init_node('tf_obstacle')
        pub = rospy.Publisher('/new_obstacles', ObstacleArrayMsg, queue_size=10)
	rospy.Subscriber("/tracked_obstacles", Obstacles, callback)
	r = rospy.Rate(10)        
	rospy.spin()
        r.sleep()
	  
    except rospy.ROSInterruptException:
        pass
