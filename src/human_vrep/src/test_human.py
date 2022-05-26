#!/usr/bin/env python
import rospy
import time
import pandas as pd
import csv
from std_msgs.msg import Float64MultiArray
pos = []
# Load test data:
pos_181 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_1.xsens.bvh.csv'
pos_182 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_2.xsens.bvh.csv'
pos_183 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_3.xsens.bvh.csv'
pos_184 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_4.xsens.bvh.csv'
pos_185 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_5.xsens.bvh.csv'
pos_186 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_1.xsens.bvh.csv'
pos_187 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_2.xsens.bvh.csv'
pos_188 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_3.xsens.bvh.csv'
pos_189 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_4.xsens.bvh.csv'
pos_190 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_5.xsens.bvh.csv'
pos_191 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_1.xsens.bvh.csv'
pos_192 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_2.xsens.bvh.csv'
pos_193 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_3.xsens.bvh.csv'
pos_194 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_4.xsens.bvh.csv'
pos_195 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_5.xsens.bvh.csv'

# TEST
pos_181 = pd.read_csv(pos_181, quoting=csv.QUOTE_NONNUMERIC)
pos_181 = pos_181.to_numpy()
pos.append(pos_181)
pos_182 = pd.read_csv(pos_182, quoting=csv.QUOTE_NONNUMERIC)
pos_182 = pos_182.to_numpy()
pos.append(pos_182)
pos_183 = pd.read_csv(pos_183, quoting=csv.QUOTE_NONNUMERIC)
pos_183 = pos_183.to_numpy()
pos.append(pos_183)
pos_184 = pd.read_csv(pos_184, quoting=csv.QUOTE_NONNUMERIC)
pos_184 = pos_184.to_numpy()
pos.append(pos_184)
pos_185 = pd.read_csv(pos_185, quoting=csv.QUOTE_NONNUMERIC)
pos_185 = pos_185.to_numpy()
pos.append(pos_185)
pos_186 = pd.read_csv(pos_186, quoting=csv.QUOTE_NONNUMERIC)
pos_186 = pos_186.to_numpy()
pos.append(pos_186)
pos_187 = pd.read_csv(pos_187, quoting=csv.QUOTE_NONNUMERIC)
pos_187 = pos_187.to_numpy()
pos.append(pos_187)
pos_188 = pd.read_csv(pos_188, quoting=csv.QUOTE_NONNUMERIC)
pos_188 = pos_188.to_numpy()
pos.append(pos_188)
pos_189 = pd.read_csv(pos_189, quoting=csv.QUOTE_NONNUMERIC)
pos_189 = pos_189.to_numpy()
pos.append(pos_189)
pos_190 = pd.read_csv(pos_190, quoting=csv.QUOTE_NONNUMERIC)
pos_190 = pos_190.to_numpy()
pos.append(pos_190)
pos_191 = pd.read_csv(pos_191, quoting=csv.QUOTE_NONNUMERIC)
pos_191 = pos_191.to_numpy()
pos.append(pos_191)
pos_192 = pd.read_csv(pos_192, quoting=csv.QUOTE_NONNUMERIC)
pos_192 = pos_192.to_numpy()
pos.append(pos_192)
pos_193 = pd.read_csv(pos_193, quoting=csv.QUOTE_NONNUMERIC)
pos_193 = pos_193.to_numpy()
pos.append(pos_193)
pos_194 = pd.read_csv(pos_194, quoting=csv.QUOTE_NONNUMERIC)
pos_194 = pos_194.to_numpy()
pos.append(pos_194)
pos_195 = pd.read_csv(pos_195, quoting=csv.QUOTE_NONNUMERIC)
pos_195 = pos_195.to_numpy()
pos.append(pos_195)

# split data to the equal parts:
splited_data=[]
for k in range (len(pos)):
    temp = pos[k]
    rep=int(len(temp)/10000)
    for i in range(rep):
        splited_data.append([temp[i*10000:i*10000+10000],k,i*10000])

# for k in range (len(pos)):
#     temp = pos[k]
#     for i in range(7):
#         splited_data.append([temp[i*1000:i*1000+10000],k,i*1000])

print(len(pos),len(splited_data))
human_spheres = rospy.Publisher('/Obstacle/human_spheres', Float64MultiArray, queue_size=1)

print("start the high level controller!")

class ENV:
    def __init__(self):
        rospy.Subscriber('/flag', Float64MultiArray, self.callback)
        self.pub = rospy.Publisher('/Obstacle/human_spheres', Float64MultiArray, queue_size=1)
        self.iter = 0
        self.condition_h = [0]*2

    def callback(self, data):
        self.condition_h = data.data[0:2]

    def check_condition(self):
        return [int(self.condition_h[0]),self.condition_h[1]]

    def step(self,i,data_part,file_number,distance,file_n,file_start):
        # print(data_part,file_n,file_start)
        k=file_number
        point_array = [0]*465
        for a in range(14):
            point_array[3*a] = (data_part[file_number+i][3*a])+distance
            point_array[3*a+1] = (data_part[file_number+i][3*a+1])+distance
            point_array[3*a+2] = (data_part[file_number+i][3*a+2])
        point_array[42] = k+1
        for f in range(10):
            for a in range(14):    
                point_array[f*42+43+3*a] = (data_part[file_number+i][3*a])+distance
                point_array[f*42+43+3*a+1] = (data_part[file_number+i][3*a+1])+distance
                point_array[f*42+43+3*a+2] = (data_part[file_number+i][3*a+2])
        point_array[463] = file_n
        point_array[464] = file_start
        obstacle_data = Float64MultiArray()
        obstacle_data.data = point_array
        self.pub.publish(obstacle_data)

if __name__ == '__main__':
    rospy.init_node("human_poses_provider", anonymous=True)
    env = ENV()
    i = 0
    cond_temp=0
    rate = rospy.Rate(250) #hz
    msg = rospy.wait_for_message("/flag", Float64MultiArray)
    # msg = True
    if(msg):
        while not rospy.is_shutdown():
            condition_h = env.check_condition()
            if condition_h[1]==1:
                time.sleep(0.02)
                i=0
            else:
                temp = splited_data[condition_h[0]][0]
                file_n = splited_data[condition_h[0]][1]
                file_start = splited_data[condition_h[0]][2]
                env.step(i,temp,condition_h[0],1.0,file_n,file_start)
                print(condition_h,i)
                i+=1
            rate.sleep()
