#!/usr/bin/python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+ "distance: {}".format(data.data))

def listener():
    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/usonic_data_0", String, callback)
    rospy.Subscriber("/usonic_data_1", String, callback)
    rospy.spin()


if __name__ == "__main__":

  listener()
