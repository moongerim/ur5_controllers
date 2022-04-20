#!/usr/bin/env python
import rospy
import time
import getch

import sys
import select
import tty
import termios

from threading import Timer

class Watchdog:
    def __init__(self, timeout, userHandler=None):  # timeout in seconds
        self.timeout = timeout
        self.handler = userHandler if userHandler is not None else self.defaultHandler
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()

    def reset(self):
        self.timer.cancel()
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()

    def stop(self):
        self.timer.cancel()

    def defaultHandler(self):
        raise self

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from kortex_driver.srv import *
from kortex_driver.msg import *

position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
barc_count = 0

pub = rospy.Publisher('/my_gen3/in/joint_velocity', Base_JointSpeeds, queue_size=1)

def myHandler():
    global watchdog
    global pub
    global barc_count
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
    rospy.loginfo('bark bark')
    barc_count = barc_count + 1
    watchdog.reset() 

watchdog = Watchdog(0.150, myHandler) # run watchdog handler if no messages in 100ms

def callback(data):
    global position
    global velocity
    for i in range(7):
        position[i]=data.position[i]
        velocity[i]=data.velocity[i]

def jointcommand(data):
    global watchdog
    global pub
    vells_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #vells_array = [-data.data[0]/10, -data.data[1]/10, -data.data[2]/10, -data.data[3]/10, 
    #               -data.data[4]/10, -data.data[5]/10, -data.data[6]/10]
    vells_array = [data.data[0], data.data[1], data.data[2], data.data[3], 
                   data.data[4], data.data[5], data.data[6]]
    hello_speed = Base_JointSpeeds()
    hello_speed.joint_speeds = []
    for i in range(len(vells_array)):
        hell_joint = JointSpeed()
        hell_joint.joint_identifier = i
        hell_joint.value = vells_array[i]
        hell_joint.duration = 0
        hello_speed.joint_speeds.append(hell_joint)
    pub.publish(hello_speed)
    rospy.loginfo(data.data)
    watchdog.reset() 
    
def main():
    global position
    global velocity
    global pub
    global barc_count
    rospy.init_node('emulator', anonymous=True)
    #rospy.Subscriber("/my_gen3/joint_states", JointState, callback)
    rospy.Subscriber("/my_gen3/joint_group_velocity_controller/command", Float64MultiArray, jointcommand)
    print('connecting')
    while (pub.get_num_connections() < 1):
        print('waiting')
    print('started')
    rate = rospy.Rate(20) # 50ms
    vells_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print('dog is out')
    old_settings = termios.tcgetattr(sys.stdin)
    while not rospy.is_shutdown():
        tty.setcbreak(sys.stdin.fileno())
        # Do something just for fun
        print('barks',barc_count)
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
    main()
