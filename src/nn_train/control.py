#!/usr/bin/env python
import rospy
import numpy as np
import time
import pandas as pd
import os
import csv
from std_msgs.msg import Float64MultiArray, Int32, Float64
import math
from sensor_msgs.msg import JointState
# pos = []
# pos_136 = '/home/robot/workspaces/ur5_mpc_vrep/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_1.xsens.bvh.csv'
# pos_136 = pd.read_csv(pos_136, quoting=csv.QUOTE_NONNUMERIC)
# pos_136 = pos_136.to_numpy()
# pos.append(pos_136)

pub = rospy.Publisher('/info', Float64MultiArray, queue_size=1)
position = [0]*6
velocity = [0]*6
human_data = [2.5]*43
# i=0
def callback(data):
    global position, velocity
    position = data.position[0:6]
    velocity = data.velocity[0:6]
check = 0

def h_callback(data):
    global human_data
    for i in range(43):
        human_data[i] = data.data[i]

def main():
    global position, velocity, human_data
    rospy.init_node('control', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.Subscriber("/human", Float64MultiArray, h_callback)
    # sleep_time = 0
    rate = rospy.Rate(125)
    while not rospy.is_shutdown():
        point_array = [0]*55
        point_array[0:6] = position[0:6]
        point_array[6:48] = human_data[0:42]
        point_array[48:54] = velocity[0:6]
        point_array[54] = human_data[42]
        infodata = Float64MultiArray()
        infodata.data = point_array
        rospy.loginfo(infodata)
        pub.publish(infodata)
        rate.sleep()
        # rospy.spin() 

if __name__ == '__main__': main()
