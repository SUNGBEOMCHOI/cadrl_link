#!/usr/bin/env python
import Jetson.GPIO as GPIO
import rospy
from geometry_msgs.msg import Vector3

count = 0
rot = 0
state = 1

pub = rospy.Publisher('pos', Vector3, queue_size=100)
rospy.init_node('w_pub', anonymous=True)
rate = rospy.Rate(100)

data = Vector3()

GPIO.setmode(GPIO.BOARD)

outA = 36
outB = 37
outZ = 31

GPIO.cleanup([outA, outB, outZ])
GPIO.setup(outA, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(outB, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(outZ, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def encoderCountA(channel):
    global count
    global state
    if GPIO.input(outA) != GPIO.input(outB):
        count+=1
        state = 1 # CW
    else:
        count+=-1
        state = -1 # CCW
"""
def encoderCountB(channel):
    global count
    global state
    if GPIO.input(outA) == GPIO.input(outB):
        count+=1
        state = 'CW'
    else:
        count+=-1
        state = 'CCW'
"""
def encoderReset(channel):
    global count
    global rot
    count=0
    if state == 1:
        rot+=1
    else:
        rot+=-1

GPIO.add_event_detect(outA, GPIO.FALLING, callback = encoderCountA)
#GPIO.add_event_detect(outB, GPIO.FALLING, callback = encoderCountB)
GPIO.add_event_detect(outZ, GPIO.FALLING, callback = encoderReset)
#main loop
"""
while not rospy.is_shutdown():
    data.x = rot
    rospy.loginfo(data)
    pub.publish(data)
    print(count)
    rate.sleep()
"""

try:
    while True:
	print('{},{},{}'.format(GPIO.input(outA),GPIO.input(outB),GPIO.input(outZ)))
	print('n: {},{},{}'.format(count,rot,state))

except KeyboardInterrupt: GPIO.cleanup() # frees GPIO driver from usage
