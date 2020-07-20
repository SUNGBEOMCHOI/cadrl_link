#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print yaw

def q2e():
    rospy.init_node('q2e')
    #sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    sub = rospy.Subscriber('/imu/data', Imu, get_rotation)
    pub = rospy.Publisher('yaw', Float32, queue_size=10)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(yaw)
        r.sleep()


if __name__ == '__main__':
    try:
        yaw=0.0
        q2e()
    except rospy.ROSInterruptException:
        pass
