#!/usr/bin/env python
import __main__ as main
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
from math import sin,cos,sqrt
from initialization import set_init_pose
from std_msgs.msg import String
import stream_tee as stream_tee
from stream_tee import write_mat
whole_start_time = time. time()
start_time = time.time()
class ENV:
    def __init__(self,run_name):
        rospy.Subscriber('/data', Float64MultiArray, self.callback)
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self.pub_2 = rospy.Publisher('/given_jv', Float64MultiArray,queue_size=1)
        self.obs = [0]*62
        self.run_name = run_name
        self.iter_n = 0
        self.init_log_variables()

    def callback(self, data):
        self.obs[0:6] = data.data[0:6]
        self.obs[6:12] = data.data[6:12]
        self.obs[12:22] = data.data[12:22]
        self.obs[22:32] = data.data[22:32]
        self.obs[32:62] = data.data[32:62]

    def step(self,vel,joint):
        hello_str = "speedj(["+str(vel[0])+","+str(vel[1])+","+str(vel[2])+","+str(vel[3])+","+str(vel[4])+","+str(vel[5])+"],"+"5.0"+",0.1)" 
        # rospy.loginfo(hello_str)
        self.pub.publish(hello_str)
        temp_data = [vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], joint]
        vel_values = Float64MultiArray()
        vel_values.data = temp_data
        self.pub_2.publish(vel_values)
        joint_states = self.observation()
        end_time = time.time()
        time_elapsed = (end_time - start_time)
        self.poses.append(joint_states[0:6])
        self.joint_velocities.append(joint_states[6:12])
        self.real_lin_vel.append(joint_states[12:22])
        self.given_lin_vel.append(joint_states[22:32])
        self.tp_table.append(joint_states[32:62])
        self.real_vel.append(vel)
        self.time_t.append(time_elapsed)
        self.joint_n.append(joint)

    def observation(self):
        return self.obs

    def reset(self, init):  
        self.save_log()
        self.iter_n+=1 
        set_init_pose(init)
        self.init_log_variables()
        
    def init_log_variables(self):
        self.poses=[]
        self.joint_velocities=[]
        self.real_lin_vel=[]
        self.given_lin_vel=[]
        self.tp_table=[]
        self.real_vel=[]
        self.time_t=[]
        self.joint_n=[]

    def save_log(self):
        write_mat('Network_log/' + self.run_name,
                        {'poses': self.poses,
                        'joint_velocities': self.joint_velocities,
                        'real_lin_vel': self.real_lin_vel,
                        'given_lin_vel':self.given_lin_vel,
                        'tp_table': self.tp_table,
                        'real_vel': self.real_vel,
                        'time_t': self.time_t,
                        'joint_n':self.joint_n},
                        str(self.iter_n))

if __name__ == '__main__':
    rospy.init_node("accel_test", anonymous=True)
    run_name = stream_tee.generate_timestamp()
    env = ENV(run_name)
    t = time.time()
    # env.reset()
    i = 0
    iteration = 0
    max_vel = 0.5
    rate = rospy.Rate(200) #hz
    joint = 2
    zero_vel = [0,0,0,0,0,0]
    init = [0.7853, -3.14, 0, -1.5708, 0, 0]
    while not rospy.is_shutdown():
        if joint == 1:
            vel = [max_vel,0,0,0,0,0]
        if joint == 2:
            vel = [0, max_vel,0,0,0,0]
        if joint == 3:
            vel = [0, 0, max_vel,0,0,0]
            init = [0.7853, -3.14, -1.57, -1.5708, 0, 0]
        if joint == 4:
            vel = [0, 0, 0, max_vel,0,0]
        if joint == 5:
            vel = [0, 0, 0, 0, max_vel,0]
        if joint == 6:
            vel = [0, 0, 0, 0, 0, max_vel]
        obs = env.observation()

        print(joint, max_vel, i, obs[6:12])
        
        if obs[6]==0.0 and obs[7]==0.0 and obs[8]==0.0 and obs[9]==0.0 and obs[10]==0.0 and obs[11]==0.0:
            start = 1

        if max_vel<1.5:
            if i>270/max_vel:
                if start==0 and i<600/max_vel:
                    env.step(zero_vel, joint)
                    print(i, zero_vel)
                else:
                    env.reset(init)
                    time_to_sleep = max_vel*20
                    time.sleep(time_to_sleep)
                    max_vel+=0.01
                    start_time = time.time()
                    i = 0
            else:
                env.step(vel, joint)
                print(i,vel)
                start=0
            i+=1
                
        else:
            joint+=1
            i=0
            max_vel = 0.4

        rate.sleep()
    
        