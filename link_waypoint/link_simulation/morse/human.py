from pymorse import Morse
import random
import time
import threading

trans_list = [[-18.0,16.0,-28.0,5.0],[-54.0,-12.0,-42.0,-22.0],[-35.0,-5.0,-40.0,-18.0],[-48.5,-24.5,-38.5,-34.5],[-30.0,-38.0,-38.0,-49.0]]
rot_list = [[3.14,0.0],[0.0,3.14],[-1.57,1.57],[3.14,0.0],[3.14,0.0]]

#-18 16 t -28 5
#-54 -12 t -43 -22
#-35 -5 t -40 -18
#-48.5 -24.5 t -38.5 -34.5
moving_order_1= [{"x":trans_list[0][2],"y":trans_list[0][3],'z':0, 'tolerance':0.05, 'speed':1},
                {"x":trans_list[1][2],"y":trans_list[1][3],'z':0, 'tolerance':0.05, 'speed':1},
                {"x":trans_list[2][2],"y":trans_list[2][3],'z':0, 'tolerance':0.05, 'speed':1},
                {"x":trans_list[3][2],"y":trans_list[3][3],'z':0, 'tolerance':0.05, 'speed':1},
                {"x":trans_list[4][2],"y":trans_list[4][3],'z':0, 'tolerance':0.05, 'speed':1}]

moving_order_2= [{"x":trans_list[0][0],"y":trans_list[0][1],'z':0, 'tolerance':0.05, 'speed':1},
                {"x":trans_list[1][0],"y":trans_list[1][1],'z':0, 'tolerance':0.05, 'speed':1},
                {"x":trans_list[2][0],"y":trans_list[2][1],'z':0, 'tolerance':0.05, 'speed':1},
                {"x":trans_list[3][0],"y":trans_list[3][1],'z':0, 'tolerance':0.05, 'speed':1},
                {"x":trans_list[4][0],"y":trans_list[4][1],'z':0, 'tolerance':0.05, 'speed':1}]

def pubA(i):
    motions[i].publish(moving_order_1[i])

def pubB(i):
    motions[i].publish(moving_order_2[i])

with Morse() as morse:
    pose1 = morse.human1.pose1
    pose2 = morse.human2.pose2
    pose3 = morse.human3.pose3
    pose4 = morse.human4.pose4
    pose0 = morse.human0.pose0
    motion1 = morse.human1.motion1
    motion2 = morse.human2.motion2
    motion3 = morse.human3.motion3
    motion4 = morse.human4.motion4
    motion0 = morse.human0.motion0 

    poses = [pose0, pose1, pose2, pose3, pose4]
    motions = [motion0, motion1, motion2, motion3, motion4]
    while True:
        for i in range(5):
            pubA(i)
        time.sleep(25)
        for i in range(5):
            pubB(i)
        time.sleep(25)
