#!/usr/bin/env python
import os
import stream_tee as stream_tee
import __main__ as main
import rospy
from std_msgs.msg import Float64MultiArray, String
import time
from stream_tee import write_mat
from initialization import set_init_pose

class ENV:
    def __init__(self,run_name):
        rospy.Subscriber('/info', Float64MultiArray, self.callback)
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self.flag_pub = rospy.Publisher('/flag', Float64MultiArray, queue_size=1)
        self.run_name = run_name
        self.A = [0.0, -2.3, -1.1, -1.2, -1.2, 0.5]
        self.B = [3.0, -1.6, -1.7, -1.7, -1.7, 1.0]
        self.start = self.A
        self.goal = self.B
        self.i = 0
        self.first = 0
        self.hello_str=[-1,0]
        self.init_log_variables()
        self.total = 0
        
    def callback(self, data):
        self.observation = data.data[0:169]

    def done(self):
        arrive = False
        # print(self.max_diff)
        if self.max_diff<self.threshold:
            print("-----Arrived------")
            arrive = True
        return arrive


    def step(self):
        # print("step")
        vel = [self.observation[81],self.observation[82],self.observation[83],self.observation[84],self.observation[85],self.observation[86]]
        hello_str = "speedj(["+str(vel[0])+","+str(vel[1])+","+str(vel[2])+","+str(vel[3])+","+str(vel[4])+","+str(vel[5])+"],"+"5.0"+",0.1)" 
        # print(vel)
        self.pub.publish(hello_str)
        elapsed_time = time.time() - self.t_total
        self.ctp.append(self.observation[0:21])
        self.human_poses.append(self.observation[21:63])
        self.joint_poses.append(self.observation[63:69])
        # self.current = self.observation[63:69]
        self.goals.append(self.observation[69:75])
        
        self.real_vels.append(self.observation[75:81])
        self.low_controller_solutions.append(self.observation[81:90])
        self.minimum_dist.append(self.observation[90:97])
        self.smallest_dist.append(self.observation[97])
        self.lin_vel_scale.append(self.observation[98])
        self.from_high_controller.append(self.observation[99:129])
        self.goal = self.observation[117:123]
        self.ctv.append(self.observation[129:150])
        self.lin_vel_limit.append(self.observation[150:157])
        self.file_n.append(self.observation[157])
        # self.time.append(self.observation[158])
        self.ctv_linear.append(self.observation[159:166])
        self.max_diff = self.observation[166]
        self.file_start.append(self.observation[167])
        self.mpc_solve_time.append(self.observation[168])
        self.time.append(elapsed_time)
    
    def reset(self): 
        print("reset")
        if self.first<1:
            self.first+=1
            self.threshold = 0.02
            self.init_log_variables()
            self.t_total = time.time()
        else:
            time.sleep(0.5)
            if self.i%2==0:
                self.hello_str[1] = 1
                pub_data = Float64MultiArray()
                pub_data.data = self.hello_str
                self.flag_pub.publish(pub_data)
                time.sleep(1)
                self.save_log(self.i)
                self.init_log_variables()
                self.hello_str[0]+= 1
                self.hello_str[1] = 0
                pub_data.data = self.hello_str
                self.flag_pub.publish(pub_data)
                time.sleep(1)
            self.i+=1
        self.step()
    
    def init_log_variables(self):
        self.observation = [1]*169
        self.ctp = []
        self.human_poses = []
        self.joint_poses = []
        self.goals = []
        self.real_vels = []
        self.low_controller_solutions = []
        self.minimum_dist = []
        self.smallest_dist = []
        self.lin_vel_scale = []
        self.from_high_controller = []
        self.ctv = []
        self.lin_vel_limit = []
        self.file_n=[]
        self.file_start = []
        self.time = []
        self.ctv_linear = []
        self.arrive = False
        self.diff = 10
        self.mpc_solve_time = []
        
    def save_log(self,save_iter):
        rec_dir = '/home/robot/workspaces/Big_Data/'
        os.chdir(rec_dir)
        print("***saving***")
        write_mat('mpc_log/' + self.run_name,
                        {'ctp':self.ctp,
                        'human_poses':self.human_poses,
                        'joint_positions': self.joint_poses,
                        'goal':self.goals,
                        'real_vels': self.real_vels,
                        'low_controller': self.low_controller_solutions,
                        'minimum_distance': self.minimum_dist,
                        'smallest_dist': self.smallest_dist,
                        'lin_vel_scale':self.lin_vel_scale,
                        'from_high_controller':self.from_high_controller,
                        'ctv': self.ctv,
                        'lin_vel_limit': self.lin_vel_limit,
                        'file_n':self.file_n,
                        'file_start':self.file_start,
                        'time':self.time,
                        'mpc_solve_time':self.mpc_solve_time,
                        'ctv_linear': self.ctv_linear},
                        str(save_iter))    

if __name__ == '__main__':
    rospy.init_node("mpc_test", anonymous=True)
    run_name = stream_tee.generate_timestamp()
    env = ENV(run_name)
    t = time.time()
    env.reset()
    i = 0
    save_iter = 0
    rate = rospy.Rate(20) #hz
    while not rospy.is_shutdown():
        # save_iter+=1
        # if save_iter%2000==0:
        #     env.save_log(save_iter)
        done = env.done()
        if done==True:
            elapsed = time.time() - t
            print("Episode ", i, ' time = ', elapsed)
            i+=1
            print("Episode", i, " is started")
            env.reset()
            t = time.time()
        else:
            env.step()
        rate.sleep()

    
        
