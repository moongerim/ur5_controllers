#!/usr/bin/env python
#
# Copyright 2015, 2016 Thomas Timm Andersen (original version)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import Float64MultiArray
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Q1 = [0,-1.5708,0,-1.5708,0,0]
Q2 = [1.5708,-1.5708,1.5708,0,1.5708,0]
Q_angle = [74*pi/180, -75*pi/180, 99*pi/180, -200*pi/180, -79*pi/180, -180*pi/180]
Q_angle2 = [-96*pi/180, -75*pi/180, 55*pi/180, -347*pi/180, 76*pi/180, 6*pi/180]
Q_angle3 = [-103*pi/180, -78*pi/180, 60*pi/180, 0.0*pi/180, 67*pi/180, 7*pi/180]
Q_angle4 = [102*pi/180, -99*pi/180, -73*pi/180, -186*pi/180, 276*pi/180, 3*pi/180]

Q_angle5 = [-253*pi/180, -100*pi/180, -71*pi/180, -189*pi/180, -64*pi/180, 11*pi/180]
client = None
flag = 0
def talker(data):
    Q1 = data.data[0:6]
    time = 5
    flag = data.data[6]
    if flag>1:
        print("inside python file")
        set_init_pose(Q1, time)

def move1(Q1,time):
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(time))]
            #JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(4.0)),
            #JointTrajectoryPoint(positions=Q_angle4, velocities=[0]*6, time_from_start=rospy.Duration(6.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def set_init_pose(Q1,time):
    global client
    try:
        # rospy.init_node("init_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print("This program makes the robot go to initial position")
        move1(Q1,time)
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

def main():
    rospy.init_node("init_move", anonymous=True)
    # rate = rospy.Rate(20) #hz
    while not rospy.is_shutdown():
        rospy.Subscriber("/LowController/init", Float64MultiArray, talker)
        rospy.spin()

if __name__ == '__main__': main()