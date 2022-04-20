#!/usr/bin/env python
import rospy
import time
import getch

import sys
import select
import tty
import termios

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from kortex_driver.srv import *
from kortex_driver.msg import *

from numpy import genfromtxt
my_data = genfromtxt('/home/robot/October/Bench17/SSM_cont/src/gen3_p/scripts/trajectory.txt', delimiter=' ')

position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def callback(data):
    global position
    global velocity
    #rospy.loginfo(data.position)
    for i in range(7):
        position[i]=data.position[i]
        velocity[i]=data.velocity[i] 
    
def listener():
    global position
    global velocity
    global my_data
    P = 1.0 # 12 is too much
    goal_pose = [0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0]
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/my_gen3/joint_states", JointState, callback)
    print('connecting')
    pub = rospy.Publisher('/my_gen3/in/joint_velocity', Base_JointSpeeds, queue_size=1)
    while (pub.get_num_connections() < 1):
        print('waiting')
    rate = rospy.Rate(20) # 50ms
    vells_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal_iterator = 0
    k='-'
    old_settings = termios.tcgetattr(sys.stdin)
    while not rospy.is_shutdown():
        tty.setcbreak(sys.stdin.fileno())
        # k=str(ord(getch.getch()))
        # goal_pose = [my_data[i][0], my_data[i][1], my_data[i][2], my_data[i][3], my_data[i][4], my_data[i][5], my_data[i][6]]
        goal_pose = [my_data[goal_iterator][0], my_data[goal_iterator][1], my_data[goal_iterator][2], my_data[goal_iterator][3], my_data[goal_iterator][4], my_data[goal_iterator][5], my_data[goal_iterator][6]]
        position_str = "goal %i Pos %.3f %.3f %.3f %.3f %.3f %.3f %.3f" % (goal_iterator, position[0],position[1],position[2],position[3],position[4],position[5],position[6])
        rospy.loginfo(position_str)
        rospy.loginfo(k)# to print on  terminal 
        vells_array[6] = P*(goal_pose[6] - position[6])
        vells_array[5] = P*(goal_pose[5] - position[5])
        vells_array[4] = P*(goal_pose[4] - position[4])
        vells_array[3] = P*(goal_pose[3] - position[3])
        vells_array[2] = P*(goal_pose[2] - position[2])
        vells_array[1] = P*(goal_pose[1] - position[1])
        vells_array[0] = P*(goal_pose[0] - position[0])
        hello_speed = Base_JointSpeeds()
        hello_speed.joint_speeds = []
        for i in range(len(vells_array)):
            hell_joint = JointSpeed()
            hell_joint.joint_identifier = i
            hell_joint.value = vells_array[i]
            hell_joint.duration = 0
            hello_speed.joint_speeds.append(hell_joint)
        pub.publish(hello_speed)
        goal_iterator=(goal_iterator+1)%len(my_data)
        if isData():
            c = sys.stdin.read(1)
            if c == '\x1b':         # x1b is ESC
                break
        rate.sleep()
    print('well done, I am out')
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    vells_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    hello_speed = Base_JointSpeeds()
    hello_speed.joint_speeds = []
    for i in range(len(vells_array)):
        hell_joint = JointSpeed()
        hell_joint.joint_identifier = i
        hell_joint.value = vells_array[i]
        hell_joint.duration = 0
        hello_speed.joint_speeds.append(hell_joint)
    pub.publish(hello_speed)
    pub.publish(hello_speed)
    pub.publish(hello_speed)
    time.sleep(0.5) # wait for 5 seconds
    rospy.loginfo(hello_speed)

if __name__ == '__main__':
    listener()
