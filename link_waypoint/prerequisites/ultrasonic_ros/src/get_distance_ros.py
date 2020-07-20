#!/usr/bin/python

'''
  # demo_get_distance.py
  #
  # Connect board with raspberryPi.
  # Run this demo.
  #
  # Connect A02 to UART
  # get the distance value
  #
  # Copyright   [DFRobot](http://www.dfrobot.com), 2016
  # Copyright   GNU Lesser General Public License
  #
  # version  V1.0
  # date  2019-8-31
'''

import time
import rospy
import serial
from std_msgs.msg import Int32
#from std_msgs.msg import String
from DFRobot_RaspberryPi_A02YYUW import DFRobot_A02_Distance as Board

class UltraSonicSensor:

  def __init__(self, port, channel):
    self.port = port
    self.channel = channel
    self.board = Board(self.port)
    self.min_dis = 0     #Minimum ranging threshold: 0mm
    self.max_dis = 4500  #Highest ranging threshold: 4500mm
    self.usonic_publisher = rospy.Publisher("/usonic_data_{}".format(self.channel), Int32, queue_size=1)
    self.distance_msg = 0
    self.board.set_dis_range(self.min_dis, self.max_dis)

  def get_usonic_distance(self):  

    if self.board.last_operate_status == self.board.STA_OK:
      self.distance_msg = self.board.getDistance()
      return
    elif self.board.last_operate_status == self.board.STA_ERR_CHECKSUM:
      self.distance_msg = "Usonic : ERROR"
    elif self.board.last_operate_status == self.board.STA_ERR_SERIAL:
      self.distance_msg ="Usonic : Serial open failed!"
    elif self.board.last_operate_status == self.board.STA_ERR_CHECK_OUT_LIMIT:
      self.distance_msg ="Usonic : Above the upper limit: " + self.distance_msg
    elif self.board.last_operate_status == self.board.STA_ERR_CHECK_LOW_LIMIT:
      self.distance_msg = "Usonic : Below the lower limit: "+ self.distance_msg
    elif self.board.last_operate_status == self.board.STA_ERR_DATA:
      self.distance_msg ="Usonic : No data!"
    raise Exception(self.distance_msg)

  def publish_usonic_data(self):

    msg = Int32()
    msg.data = self.distance_msg
    self.usonic_publisher.publish(msg)

    # else:
    #   self.usonic_publisher_1.publish(msg)

        

if __name__ == "__main__":
  
  rospy.init_node('send_distance')
  port_0 = rospy.get_param("~port_0")
  port_1 = rospy.get_param("~port_1")
  rospy.set_param('~port_0', "/dev/us1")
  rospy.set_param('~port_1', "/dev/us2")
  rate = rospy.Rate(5)
  us_0 = UltraSonicSensor(port_0, 0)
  us_1 = UltraSonicSensor(port_1, 1)
  error_dict = {}
  error_cnt = 0
  try:
    while not rospy.is_shutdown():
      try:
        us_0.get_usonic_distance()
        us_0.publish_usonic_data()
      except Exception as e:
        #print(e)
        if e.__str__() in error_dict.keys():
          error_dict[e.__str__()] += 1
        else:
          error_dict[e.__str__()] = 1
        us_0.board = Board(us_0.port)
      try:
        us_1.get_usonic_distance()
        us_1.publish_usonic_data()
      except Exception as e:
        #print(e)
        if e.__str__() in error_dict.keys():
          error_dict[e.__str__()] += 1
        else:
          error_dict[e.__str__()] = 1
        us_1.board = Board(us_1.port)
      # rospy.Time(.5)
      #rospy.loginfo("First usensor distance is "+str(us_0.distance_msg))
      #rospy.loginfo("Second usensor distance is "+str(us_1.distance_msg))
      #rospy.loginfo(error_dict.__str__() + "\n")
      rate.sleep()
    
  except rospy.ROSInterruptException as e:
    pass
  #rospy.Timer(rospy.Duration(1.0/10.0), us.get_usonic_distance)
  #rospy.Timer(rospy.Duration(1.0/10.0), us.publish_usonic_data)
 
 
  
