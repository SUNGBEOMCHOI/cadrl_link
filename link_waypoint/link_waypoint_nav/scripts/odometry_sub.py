#!/usr/bin/python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import numpy as np

data_path = "/home/link/"
odometry_stamp = open(data_path + "odometry_filtered.txt", 'w+')
fourbyfour_matrix = open(data_path + "fourbyfour_matrix.txt", 'w+')
odometry_stamp.write("x y z qx qy qz qw\n")



def odometryCb(msg):
    global odometry_stamp, fourbyfour_matrix, count, rate
    t = tf.TransformerROS()
    #logging odometry
    rospy.loginfo("\nPose: {}\nQuaternion: {}\n".format(msg.pose.pose.position, msg.pose.pose.orientation))
    
    rot = (str(msg.pose.pose.orientation.x),str(msg.pose.pose.orientation.y),
    str(msg.pose.pose.orientation.z),str(msg.pose.pose.orientation.w))
    trans = (str(msg.pose.pose.position.x),str(msg.pose.pose.position.y),str(msg.pose.pose.position.x))
    #logging 4x4 matrix
    rot_trans_flat = t.fromTranslationRotation(trans, rot)
    rot_trans_flat = (rot_trans_flat.flatten()).tolist()
    rot_trans_str = "{} {} {} {} {} {} {} {} {} {} {} {}\n".format(rot_trans_flat[0],
    rot_trans_flat[1], rot_trans_flat[2], rot_trans_flat[3], rot_trans_flat[4], rot_trans_flat[5], rot_trans_flat[6],
    rot_trans_flat[7], rot_trans_flat[8], rot_trans_flat[9], rot_trans_flat[10], rot_trans_flat[11]
    )
    fourbyfour_matrix.write(rot_trans_str)
    odometry_stamp.write("{} {} {} {} {} {} {}\n".format(
        msg.pose.pose.position.x, msg.pose.pose.position.y,
        msg.pose.pose.position.z, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w))
  

if __name__ =='__main__':
    try:
      rospy.init_node('odometry_logger')
      rospy.Subscriber("/link_waypoint_nav/odometry/filtered", Odometry, odometryCb)
      rospy.spin()
    except rospy.ROSInterruptException:
      odometry_stamp.close()
      fourbyfour_matrix.close()
