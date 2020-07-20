#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <link_waypoint> environment

Feel free to edit this template as you like!
"""
#import rospy
import math
from morse.builder import *
from link_waypoint.builder.robots import LINK

robot = LINK()
robot.add_interface('ros')
#robot.translate(-31.5,96.22,0)
robot.rotate(0,0,0)

odom = Odometry()
#odom.alter('Noise', pos_std = {'x': 0.01, 'y': 0.01, 'z': 0.01},
#                    rot_std = {'roll': 0.01, 'pitch': 0.01, 'yaw': 0.01})
robot.append(odom)
odom.add_interface('ros', topic='/odometry/filtered')

gps = GPS()
gps.alter('UTM')
gps.translate(-0.25,0.0,0.03)
robot.append(gps)
gps.add_interface('ros', topic='/navsat/fix')

imu = IMU()
#imu.alter('IMUNoise', gyro_std={'x':0.1,'y':0.1,'z':0.1})
imu.translate(0,0,0.05)
imu.rotate(0,0,0)
robot.append(imu)
imu.add_interface('ros', topic='imu')

depthcamera = DepthVideoCamera()
depthcamera.translate(0.25,-0.05,0.23)
robot.append(depthcamera)
depthcamera.add_interface('ros', topic='/camera/depth')

cam = VideoCamera()
cam.translate(0.25,0.05,0.23)
robot.append(cam)
cam.add_interface('ros', topic='camera')

pointcloud = DepthCamera()
pointcloud.translate(0.25,0,0.23)
robot.append(pointcloud)
pointcloud.add_interface('ros', topic='pc')

laserscan = Sick()
laserscan.translate(0.25,0.0,0.1)
laserscan.properties(scan_window=360,laser_range=5.0)
robot.append(laserscan)
laserscan.add_interface('ros', topic='scan')

motion = MotionVWDiff()
motion.properties(ControlType = 'Position')
#motion.set_speed(3.0,0.3)
robot.append(motion)
motion.add_interface('ros', topic='/cmd_vel')

#-17.0,12.0
trans_list = [[4.0,0.0,-30.0,-5.0],[-45.0,-12.0,-34.0,0.0],[-48.0,-24.0,-36.0,-34.0],[-45.0,-35.0,-55.0,-45.0],[-30.0,-38.0,-38.0,-49.0]]
rot_list = [[-1.57,0.0],[0.0,3.14],[-1.57,1.57],[3.14,0.0],[3.14,0.0]]

human0 = Human()
human0.translate(x=trans_list[0][0],y=trans_list[0][1])
human0.rotate(0,0,rot_list[0][0])
pose0 = Pose()
human0.append(pose0)
pose0.add_stream('socket')
motion0 = Waypoint()
motion0.properties(ControlType='Position')
human0.append(motion0)
motion0.add_stream('socket')

human1 = Human()
human1.translate(x=trans_list[1][0],y=trans_list[1][1])
human1.rotate(0,0,rot_list[1][0])
pose1 = Pose()
human1.append(pose1)
pose1.add_stream('socket')
motion1 = Waypoint()
motion1.properties(ControlType='Position')
human1.append(motion1)
motion1.add_stream('socket')

human2 = Human()
human2.translate(x=trans_list[2][0],y=trans_list[2][1])
human2.rotate(0,0,rot_list[2][0])
pose2 = Pose()
human2.append(pose2)
pose2.add_stream('socket')
motion2 = Waypoint()
motion2.properties(ControlType='Position')
human2.append(motion2)
motion2.add_stream('socket')

human3 = Human()
human3.translate(x=trans_list[3][0],y=trans_list[3][1])
human3.rotate(0,0,rot_list[3][0])
pose3 = Pose()
human3.append(pose3)
pose3.add_stream('socket')
motion3 = Waypoint()
motion3.properties(ControlType='Position')
human3.append(motion3)
motion3.add_stream('socket')

human4 = Human()
human4.translate(x=trans_list[4][0],y=trans_list[4][1])
human4.rotate(0,0,rot_list[4][0])
pose4 = Pose()
human4.append(pose4)
pose4.add_stream('socket')
motion4 = Waypoint()
motion4.properties(ControlType='Position')
human4.append(motion4)
motion4.add_stream('socket')

env = Environment('empty')
#env.fullscreen(True)
env.show_framerate(True)
env.simulator_frequency(60,1,1)
#env.set_camera_location([-55.0, -2.6, 44.0])
#env.set_camera_rotation([1.20, 0, -1.20])
env.properties(UTMXOffset='293835.5', UTMYOffset='4139818.5', UTMZOffset=0.0)
#env.set_time_scale(0.2)
#env.use_internal_syncer()
