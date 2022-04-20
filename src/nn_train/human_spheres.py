#!/usr/bin/env python
import rospy
import time
import pandas as pd
import csv
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
pos = []
pos_136 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_1.xsens.bvh.csv'
pos_137 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_2.xsens.bvh.csv'
pos_138 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_3.xsens.bvh.csv'
pos_139 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_4.xsens.bvh.csv'
pos_140 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_1_Trial_5.xsens.bvh.csv'
pos_141 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_1.xsens.bvh.csv'
pos_142 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_2.xsens.bvh.csv'
pos_143 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_3.xsens.bvh.csv'
pos_144 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_4.xsens.bvh.csv'
pos_145 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_2_Trial_5.xsens.bvh.csv'
pos_146 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_1.xsens.bvh.csv'
pos_147 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_2.xsens.bvh.csv'
pos_148 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_3.xsens.bvh.csv'
pos_149 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_4.xsens.bvh.csv'
pos_150 = '/home/robot/workspaces/human_data/Participant_8410_csv/Participant_8410_Setup_A_Seq_6_Trial_5.xsens.bvh.csv'

pos_136 = pd.read_csv(pos_136, quoting=csv.QUOTE_NONNUMERIC)
pos_136 = pos_136.to_numpy()
pos.append(pos_136)
pos_137 = pd.read_csv(pos_137, quoting=csv.QUOTE_NONNUMERIC)
pos_137 = pos_137.to_numpy()
pos.append(pos_137)
pos_138 = pd.read_csv(pos_138, quoting=csv.QUOTE_NONNUMERIC)
pos_138 = pos_138.to_numpy()
pos.append(pos_138)
pos_139 = pd.read_csv(pos_139, quoting=csv.QUOTE_NONNUMERIC)
pos_139 = pos_139.to_numpy()
pos.append(pos_139)
pos_140 = pd.read_csv(pos_140, quoting=csv.QUOTE_NONNUMERIC)
pos_140 = pos_140.to_numpy()
pos.append(pos_140)
pos_141 = pd.read_csv(pos_141, quoting=csv.QUOTE_NONNUMERIC)
pos_141 = pos_141.to_numpy()
pos.append(pos_141)
pos_142 = pd.read_csv(pos_142, quoting=csv.QUOTE_NONNUMERIC)
pos_142 = pos_142.to_numpy()
pos.append(pos_142)
pos_143 = pd.read_csv(pos_143, quoting=csv.QUOTE_NONNUMERIC)
pos_143 = pos_143.to_numpy()
pos.append(pos_143)
pos_144 = pd.read_csv(pos_144, quoting=csv.QUOTE_NONNUMERIC)
pos_144 = pos_144.to_numpy()
pos.append(pos_144)
pos_145 = pd.read_csv(pos_145, quoting=csv.QUOTE_NONNUMERIC)
pos_145 = pos_145.to_numpy()
pos.append(pos_145)
pos_146 = pd.read_csv(pos_146, quoting=csv.QUOTE_NONNUMERIC)
pos_146 = pos_146.to_numpy()
pos.append(pos_146)
pos_147 = pd.read_csv(pos_147, quoting=csv.QUOTE_NONNUMERIC)
pos_147 = pos_147.to_numpy()
pos.append(pos_147)
pos_148 = pd.read_csv(pos_148, quoting=csv.QUOTE_NONNUMERIC)
pos_148 = pos_148.to_numpy()
pos.append(pos_148)
pos_149 = pd.read_csv(pos_149, quoting=csv.QUOTE_NONNUMERIC)
pos_149 = pos_149.to_numpy()
pos.append(pos_149)
pos_150 = pd.read_csv(pos_150, quoting=csv.QUOTE_NONNUMERIC)
pos_150 = pos_150.to_numpy()
pos.append(pos_150)

human_spheres = rospy.Publisher('/human', Float64MultiArray, queue_size=1)

print("start")
condition_h = 1
def human_condition(data):
    global condition_h
    if data.data == 'stop_human':
        condition_h = 1
    else:
        condition_h = 0

def main():
    global pos, position, velocity, condition_h
    rospy.init_node('human_control', anonymous=True)
    # sleep_time = 0
    msg = rospy.wait_for_message("/flag", String)
    rospy.Subscriber("/flag", String, human_condition)
    # msg = True
    if(msg):
        # time.sleep(2)
        for k in range (len(pos)):
            print(k)
            temp = pos[k]
            for i in range (len(temp)):
                point_array = [0]*43
                for a in range(14):
                    point_array[3*a] = (temp[i][3*a])+1.0
                    point_array[3*a+1] = (temp[i][3*a+1])+1.0
                    point_array[3*a+2] = (temp[i][3*a+2])-1.2
                point_array[42] = k+1
                obstacle_data = Float64MultiArray()
                obstacle_data.data = point_array
                if condition_h==0: 
                    human_spheres.publish(obstacle_data)
                else:
                    time.sleep(0.05)
                    # print("sleep human")
                # rospy.loginfo(obstacle_data)
                time.sleep(0.004)
        rospy.spin()
        print("the end")
    print("exit")    

if __name__ == '__main__': main()
