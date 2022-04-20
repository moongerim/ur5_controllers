#!/usr/bin/env python
import rospy
import numpy as np
import time
import pandas as pd
import os
import csv
from std_msgs.msg import Float64MultiArray
import math


pub = rospy.Publisher('/HighController/mpc_high_positions', Float64MultiArray, queue_size=1)
# test_goal = [0,0,0,0,0,0,0,0,0,0,0,0,0,-1.57,0,-1.57,0,0]
test_goal = [0,0,0,0,0,0,0,0,0,0,0,0,1,-1.57,0,-1.57,0,0]

def main():
    global test_goal
    rospy.init_node('test_control', anonymous=True)
    while not rospy.is_shutdown():
        pub_data = Float64MultiArray()
        pub_data.data = test_goal
        pub.publish(pub_data)
        time.sleep(0.01)
    rospy.spin()
    print("exit")    

if __name__ == '__main__': main()
