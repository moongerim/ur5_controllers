#!/usr/bin/env python
import rospy
import numpy as np
import time
import pandas as pd
import os
import csv
from std_msgs.msg import Float64MultiArray, String
import math
pos = []
# Train
pos_1 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_3_Trial_1.xsens.bvh.csv'
pos_2 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_3_Trial_2.xsens.bvh.csv'
pos_3 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_3_Trial_3.xsens.bvh.csv'
pos_4 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_3_Trial_4.xsens.bvh.csv'
pos_5 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_3_Trial_5.xsens.bvh.csv'
pos_6 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_4_Trial_1.xsens.bvh.csv'
pos_7 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_4_Trial_2.xsens.bvh.csv'
pos_8 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_4_Trial_3.xsens.bvh.csv'
pos_9 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_4_Trial_4.xsens.bvh.csv'
pos_10 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_4_Trial_5.xsens.bvh.csv'
pos_11 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_5_Trial_1.xsens.bvh.csv'
pos_12 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_5_Trial_2.xsens.bvh.csv'
pos_13 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_5_Trial_3.xsens.bvh.csv'
pos_14 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_5_Trial_4.xsens.bvh.csv'
pos_15 = '/home/robot/workspaces/human_data/Participant_541_csv/Participant_541_Setup_A_Seq_5_Trial_5.xsens.bvh.csv'
pos_16 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_1_Trial_1.xsens.bvh.csv'
pos_17 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_1_Trial_2.xsens.bvh.csv'
pos_18 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_1_Trial_3.xsens.bvh.csv'
pos_19 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_1_Trial_4.xsens.bvh.csv'
pos_20 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_1_Trial_5.xsens.bvh.csv'
pos_21 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_3_Trial_1.xsens.bvh.csv'
pos_22 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_3_Trial_2.xsens.bvh.csv'
pos_23 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_3_Trial_3.xsens.bvh.csv'
pos_24 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_3_Trial_4.xsens.bvh.csv'
pos_25 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_3_Trial_5.xsens.bvh.csv'
pos_26 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_5_Trial_1.xsens.bvh.csv'
pos_27 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_5_Trial_2.xsens.bvh.csv'
pos_28 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_5_Trial_3.xsens.bvh.csv'
pos_29 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_5_Trial_4.xsens.bvh.csv'
pos_30 = '/home/robot/workspaces/human_data/Participant_909_csv/Participant_909_Setup_A_Seq_5_Trial_5.xsens.bvh.csv'
pos_31 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_3_Trial_1.xsens.bvh.csv'
pos_32 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_3_Trial_2.xsens.bvh.csv'
pos_33 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_3_Trial_3.xsens.bvh.csv'
pos_34 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_3_Trial_4.xsens.bvh.csv'
pos_35 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_3_Trial_5.xsens.bvh.csv'
pos_36 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_5_Trial_1.xsens.bvh.csv'
pos_37 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_5_Trial_2.xsens.bvh.csv'
pos_38 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_5_Trial_3.xsens.bvh.csv'
pos_39 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_5_Trial_4.xsens.bvh.csv'
pos_40 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_5_Trial_5.xsens.bvh.csv'
pos_41 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_6_Trial_1.xsens.bvh.csv'
pos_42 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_6_Trial_2.xsens.bvh.csv'
pos_43 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_6_Trial_3.xsens.bvh.csv'
pos_44 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_6_Trial_4.xsens.bvh.csv'
pos_45 = '/home/robot/workspaces/human_data/Participant_2193_csv/Participant_2193_Setup_B_Seq_6_Trial_5.xsens.bvh.csv'
pos_46 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_1_Trial_1.xsens.bvh.csv'
pos_47 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_1_Trial_2.xsens.bvh.csv'
pos_48 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_1_Trial_3.xsens.bvh.csv'
pos_49 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_1_Trial_4.xsens.bvh.csv'
pos_50 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_1_Trial_5.xsens.bvh.csv'
pos_51 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_3_Trial_1.xsens.bvh.csv'
pos_52 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_3_Trial_2.xsens.bvh.csv'
pos_53 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_3_Trial_3.xsens.bvh.csv'
pos_54 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_3_Trial_4.xsens.bvh.csv'
pos_55 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_3_Trial_5.xsens.bvh.csv'
pos_56 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_5_Trial_1.xsens.bvh.csv'
pos_57 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_5_Trial_2.xsens.bvh.csv'
pos_58 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_5_Trial_3.xsens.bvh.csv'
pos_59 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_5_Trial_4.xsens.bvh.csv'
pos_60 = '/home/robot/workspaces/human_data/Participant_2274_csv/Participant_2274_Setup_B_Seq_5_Trial_5.xsens.bvh.csv'
pos_61 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_3_Trial_1.xsens.bvh.csv'
pos_62 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_3_Trial_2.xsens.bvh.csv'
pos_63 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_3_Trial_3.xsens.bvh.csv'
pos_64 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_3_Trial_4.xsens.bvh.csv'
pos_65 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_3_Trial_5.xsens.bvh.csv'
pos_66 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_4_Trial_1.xsens.bvh.csv'
pos_67 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_4_Trial_2.xsens.bvh.csv'
pos_68 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_4_Trial_3.xsens.bvh.csv'
pos_69 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_4_Trial_4.xsens.bvh.csv'
pos_70 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_4_Trial_5.xsens.bvh.csv'
pos_71 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_5_Trial_1.xsens.bvh.csv'
pos_72 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_5_Trial_2.xsens.bvh.csv'
pos_73 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_5_Trial_3.xsens.bvh.csv'
pos_74 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_5_Trial_4.xsens.bvh.csv'
pos_75 = '/home/robot/workspaces/human_data/Participant_3327_csv/Participant_3327_csv/Participant_3327_Setup_A_Seq_5_Trial_5.xsens.bvh.csv'
pos_76 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_1_Trial_1.xsens.bvh.csv'
pos_77 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_1_Trial_2.xsens.bvh.csv'
pos_78 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_1_Trial_3.xsens.bvh.csv'
pos_79 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_1_Trial_4.xsens.bvh.csv'
pos_80 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_1_Trial_5.xsens.bvh.csv'
pos_81 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_4_Trial_1.xsens.bvh.csv'
pos_82 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_4_Trial_2.xsens.bvh.csv'
pos_83 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_4_Trial_3.xsens.bvh.csv'
pos_84 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_4_Trial_4.xsens.bvh.csv'
pos_85 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_4_Trial_5.xsens.bvh.csv'
pos_86 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_5_Trial_1.xsens.bvh.csv'
pos_87 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_5_Trial_2.xsens.bvh.csv'
pos_88 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_5_Trial_3.xsens.bvh.csv'
pos_89 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_5_Trial_4.xsens.bvh.csv'
pos_90 = '/home/robot/workspaces/human_data/Participant_5124_csv/Participant_5124_Setup_B_Seq_5_Trial_5.xsens.bvh.csv'
pos_91 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_2_Trial_1.xsens.bvh.csv'
pos_92 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_2_Trial_2.xsens.bvh.csv'
pos_93 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_2_Trial_3.xsens.bvh.csv'
pos_94 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_2_Trial_4.xsens.bvh.csv'
pos_95 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_2_Trial_5.xsens.bvh.csv'
pos_96 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_4_Trial_1.xsens.bvh.csv'
pos_97 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_4_Trial_2.xsens.bvh.csv'
pos_98 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_4_Trial_3.xsens.bvh.csv'
pos_99 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_4_Trial_4.xsens.bvh.csv'
pos_100= '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_4_Trial_5.xsens.bvh.csv'
pos_101 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_6_Trial_1.xsens.bvh.csv'
pos_102 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_6_Trial_2.xsens.bvh.csv'
pos_103 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_6_Trial_3.xsens.bvh.csv'
pos_104 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_6_Trial_4.xsens.bvh.csv'
pos_105 = '/home/robot/workspaces/human_data/Participant_5319_csv/Participant_5319_csv/Participant_5319_Setup_B_Seq_6_Trial_5.xsens.bvh.csv'
pos_106 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_3_Trial_1.xsens.bvh.csv'
pos_107 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_3_Trial_2.xsens.bvh.csv'
pos_108 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_3_Trial_3.xsens.bvh.csv'
pos_109 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_3_Trial_4.xsens.bvh.csv'
pos_110 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_3_Trial_5.xsens.bvh.csv'
pos_111 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_4_Trial_1.xsens.bvh.csv'
pos_112 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_4_Trial_2.xsens.bvh.csv'
pos_113 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_4_Trial_3.xsens.bvh.csv'
pos_114 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_4_Trial_4.xsens.bvh.csv'
pos_115 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_4_Trial_5.xsens.bvh.csv'
pos_116 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_6_Trial_1.xsens.bvh.csv'
pos_117 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_6_Trial_2.xsens.bvh.csv'
pos_118 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_6_Trial_3.xsens.bvh.csv'
pos_119 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_6_Trial_4.xsens.bvh.csv'
pos_120 = '/home/robot/workspaces/human_data/Participant_5521_csv/Participant_5521_Setup_A_Seq_6_Trial_5.xsens.bvh.csv'
pos_121 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_1_Trial_1.xsens.bvh.csv'
pos_122 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_1_Trial_2.xsens.bvh.csv'
pos_123 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_1_Trial_3.xsens.bvh.csv'
pos_124 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_1_Trial_4.xsens.bvh.csv'
pos_125 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_1_Trial_5.xsens.bvh.csv'
pos_126 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_2_Trial_1.xsens.bvh.csv'
pos_127 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_2_Trial_2.xsens.bvh.csv'
pos_128 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_2_Trial_3.xsens.bvh.csv'
pos_129 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_2_Trial_4.xsens.bvh.csv'
pos_130 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_2_Trial_5.xsens.bvh.csv'
pos_131 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_5_Trial_1.xsens.bvh.csv'
pos_132 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_5_Trial_2.xsens.bvh.csv'
pos_133 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_5_Trial_3.xsens.bvh.csv'
pos_134 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_5_Trial_4.xsens.bvh.csv'
pos_135 = '/home/robot/workspaces/human_data/Participant_5535_csv/Participant_5535_Setup_A_Seq_5_Trial_5.xsens.bvh.csv'
pos_136 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_2_Trial_1.xsens.bvh.csv'
pos_137 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_2_Trial_2.xsens.bvh.csv'
pos_138 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_2_Trial_3.xsens.bvh.csv'
pos_139 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_2_Trial_4.xsens.bvh.csv'
pos_140 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_2_Trial_5.xsens.bvh.csv'
pos_141 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_4_Trial_1.xsens.bvh.csv'
pos_142 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_4_Trial_2.xsens.bvh.csv'
pos_143 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_4_Trial_3.xsens.bvh.csv'
pos_144 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_4_Trial_4.xsens.bvh.csv'
pos_145 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_4_Trial_5.xsens.bvh.csv'
pos_146 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_6_Trial_1.xsens.bvh.csv'
pos_147 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_6_Trial_2.xsens.bvh.csv'
pos_148 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_6_Trial_3.xsens.bvh.csv'
pos_149 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_6_Trial_4.xsens.bvh.csv'
pos_150 = '/home/robot/workspaces/human_data/Participant_8524_csv/Participant_8524_Setup_B_Seq_6_Trial_5.xsens.bvh.csv'

# For validation
pos_151 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_1_Trial_1.xsens.bvh.csv'
pos_152 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_1_Trial_2.xsens.bvh.csv'
pos_153 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_1_Trial_3.xsens.bvh.csv'
pos_154 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_1_Trial_4.xsens.bvh.csv'
pos_155 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_1_Trial_5.xsens.bvh.csv'
pos_156 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_4_Trial_1.xsens.bvh.csv'
pos_157 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_4_Trial_2.xsens.bvh.csv'
pos_158 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_4_Trial_3.xsens.bvh.csv'
pos_159 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_4_Trial_4.xsens.bvh.csv'
pos_160 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_4_Trial_5.xsens.bvh.csv'
pos_161 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_5_Trial_1.xsens.bvh.csv'
pos_162 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_5_Trial_2.xsens.bvh.csv'
pos_163 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_5_Trial_3.xsens.bvh.csv'
pos_164 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_5_Trial_4.xsens.bvh.csv'
pos_165 = '/home/robot/workspaces/human_data/Participant_9266_csv/Participant_9266_Setup_B_Seq_5_Trial_5.xsens.bvh.csv'
pos_166 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_1_Trial_1.xsens.bvh.csv'
pos_167 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_1_Trial_2.xsens.bvh.csv'
pos_168 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_1_Trial_3.xsens.bvh.csv'
pos_169 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_1_Trial_4.xsens.bvh.csv'
pos_170 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_1_Trial_5.xsens.bvh.csv'
pos_171 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_3_Trial_1.xsens.bvh.csv'
pos_172 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_3_Trial_2.xsens.bvh.csv'
pos_173 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_3_Trial_3.xsens.bvh.csv'
pos_174 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_3_Trial_4.xsens.bvh.csv'
pos_175 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_3_Trial_5.xsens.bvh.csv'
pos_176 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_4_Trial_1.xsens.bvh.csv'
pos_177 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_4_Trial_2.xsens.bvh.csv'
pos_178 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_4_Trial_3.xsens.bvh.csv'
pos_179 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_4_Trial_4.xsens.bvh.csv'
pos_180 = '/home/robot/workspaces/human_data/Participant_9875_csv/Participant_9875_Setup_B_Seq_4_Trial_5.xsens.bvh.csv'


# For testing
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

pos_1 = pd.read_csv(pos_1, quoting=csv.QUOTE_NONNUMERIC)
pos_1 = pos_1.to_numpy()
pos.append(pos_1)
pos_2 = pd.read_csv(pos_2, quoting=csv.QUOTE_NONNUMERIC)
pos_2 = pos_2.to_numpy()
pos.append(pos_2)
pos_3 = pd.read_csv(pos_3, quoting=csv.QUOTE_NONNUMERIC)
pos_3 = pos_3.to_numpy()
pos.append(pos_3)
pos_4 = pd.read_csv(pos_4, quoting=csv.QUOTE_NONNUMERIC)
pos_4 = pos_4.to_numpy()
pos.append(pos_4)
pos_5 = pd.read_csv(pos_5, quoting=csv.QUOTE_NONNUMERIC)
pos_5 = pos_5.to_numpy()
pos.append(pos_5)
pos_6 = pd.read_csv(pos_6, quoting=csv.QUOTE_NONNUMERIC)
pos_6 = pos_6.to_numpy()
pos.append(pos_6)
pos_7 = pd.read_csv(pos_7, quoting=csv.QUOTE_NONNUMERIC)
pos_7 = pos_7.to_numpy()
pos.append(pos_7)
pos_8 = pd.read_csv(pos_8, quoting=csv.QUOTE_NONNUMERIC)
pos_8 = pos_8.to_numpy()
pos.append(pos_8)
pos_9 = pd.read_csv(pos_9, quoting=csv.QUOTE_NONNUMERIC)
pos_9 = pos_9.to_numpy()
pos.append(pos_9)
pos_10 = pd.read_csv(pos_10, quoting=csv.QUOTE_NONNUMERIC)
pos_10 = pos_10.to_numpy()
pos.append(pos_10)
pos_11 = pd.read_csv(pos_11, quoting=csv.QUOTE_NONNUMERIC)
pos_11 = pos_11.to_numpy()
pos.append(pos_11)
pos_12 = pd.read_csv(pos_12, quoting=csv.QUOTE_NONNUMERIC)
pos_12 = pos_12.to_numpy()
pos.append(pos_12)
pos_13 = pd.read_csv(pos_13, quoting=csv.QUOTE_NONNUMERIC)
pos_13 = pos_13.to_numpy()
pos.append(pos_13)
pos_14 = pd.read_csv(pos_14, quoting=csv.QUOTE_NONNUMERIC)
pos_14 = pos_14.to_numpy()
pos.append(pos_14)
pos_15 = pd.read_csv(pos_15, quoting=csv.QUOTE_NONNUMERIC)
pos_15 = pos_15.to_numpy()
pos.append(pos_15)
pos_16 = pd.read_csv(pos_16, quoting=csv.QUOTE_NONNUMERIC)
pos_16 = pos_16.to_numpy()
pos.append(pos_16)
pos_17 = pd.read_csv(pos_17, quoting=csv.QUOTE_NONNUMERIC)
pos_17 = pos_17.to_numpy()
pos.append(pos_17)
pos_18 = pd.read_csv(pos_18, quoting=csv.QUOTE_NONNUMERIC)
pos_18 = pos_18.to_numpy()
pos.append(pos_18)
pos_19 = pd.read_csv(pos_19, quoting=csv.QUOTE_NONNUMERIC)
pos_19 = pos_19.to_numpy()
pos.append(pos_19)
pos_20 = pd.read_csv(pos_20, quoting=csv.QUOTE_NONNUMERIC)
pos_20 = pos_20.to_numpy()
pos.append(pos_20)
pos_21 = pd.read_csv(pos_21, quoting=csv.QUOTE_NONNUMERIC)
pos_21 = pos_21.to_numpy()
pos.append(pos_21)
pos_22 = pd.read_csv(pos_22, quoting=csv.QUOTE_NONNUMERIC)
pos_22 = pos_22.to_numpy()
pos.append(pos_22)
pos_23 = pd.read_csv(pos_23, quoting=csv.QUOTE_NONNUMERIC)
pos_23 = pos_23.to_numpy()
pos.append(pos_23)
pos_24 = pd.read_csv(pos_24, quoting=csv.QUOTE_NONNUMERIC)
pos_24 = pos_24.to_numpy()
pos.append(pos_24)
pos_25 = pd.read_csv(pos_25, quoting=csv.QUOTE_NONNUMERIC)
pos_25 = pos_25.to_numpy()
pos.append(pos_25)
pos_26 = pd.read_csv(pos_26, quoting=csv.QUOTE_NONNUMERIC)
pos_26 = pos_26.to_numpy()
pos.append(pos_26)
pos_27 = pd.read_csv(pos_27, quoting=csv.QUOTE_NONNUMERIC)
pos_27 = pos_27.to_numpy()
pos.append(pos_27)
pos_28 = pd.read_csv(pos_28, quoting=csv.QUOTE_NONNUMERIC)
pos_28 = pos_28.to_numpy()
pos.append(pos_28)
pos_29 = pd.read_csv(pos_29, quoting=csv.QUOTE_NONNUMERIC)
pos_29 = pos_29.to_numpy()
pos.append(pos_29)
pos_30 = pd.read_csv(pos_30, quoting=csv.QUOTE_NONNUMERIC)
pos_30 = pos_30.to_numpy()
pos.append(pos_30)
pos_31 = pd.read_csv(pos_31, quoting=csv.QUOTE_NONNUMERIC)
pos_31 = pos_31.to_numpy()
pos.append(pos_31)
pos_32 = pd.read_csv(pos_32, quoting=csv.QUOTE_NONNUMERIC)
pos_32 = pos_32.to_numpy()
pos.append(pos_32)
pos_33 = pd.read_csv(pos_33, quoting=csv.QUOTE_NONNUMERIC)
pos_33 = pos_33.to_numpy()
pos.append(pos_33)
pos_34 = pd.read_csv(pos_34, quoting=csv.QUOTE_NONNUMERIC)
pos_34 = pos_34.to_numpy()
pos.append(pos_34)
pos_35 = pd.read_csv(pos_35, quoting=csv.QUOTE_NONNUMERIC)
pos_35 = pos_35.to_numpy()
pos.append(pos_35)
pos_36 = pd.read_csv(pos_36, quoting=csv.QUOTE_NONNUMERIC)
pos_36 = pos_36.to_numpy()
pos.append(pos_36)
pos_37 = pd.read_csv(pos_37, quoting=csv.QUOTE_NONNUMERIC)
pos_37 = pos_37.to_numpy()
pos.append(pos_37)
pos_38 = pd.read_csv(pos_38, quoting=csv.QUOTE_NONNUMERIC)
pos_38 = pos_38.to_numpy()
pos.append(pos_38)
pos_39 = pd.read_csv(pos_39, quoting=csv.QUOTE_NONNUMERIC)
pos_39 = pos_39.to_numpy()
pos.append(pos_39)
pos_40 = pd.read_csv(pos_40, quoting=csv.QUOTE_NONNUMERIC)
pos_40 = pos_40.to_numpy()
pos.append(pos_40)
pos_41 = pd.read_csv(pos_41, quoting=csv.QUOTE_NONNUMERIC)
pos_41 = pos_41.to_numpy()
pos.append(pos_41)
pos_42 = pd.read_csv(pos_42, quoting=csv.QUOTE_NONNUMERIC)
pos_42 = pos_42.to_numpy()
pos.append(pos_42)
pos_43 = pd.read_csv(pos_43, quoting=csv.QUOTE_NONNUMERIC)
pos_43 = pos_43.to_numpy()
pos.append(pos_43)
pos_44 = pd.read_csv(pos_44, quoting=csv.QUOTE_NONNUMERIC)
pos_44 = pos_44.to_numpy()
pos.append(pos_44)
pos_45 = pd.read_csv(pos_45, quoting=csv.QUOTE_NONNUMERIC)
pos_45 = pos_45.to_numpy()
pos.append(pos_45)
pos_46 = pd.read_csv(pos_46, quoting=csv.QUOTE_NONNUMERIC)
pos_46 = pos_46.to_numpy()
pos.append(pos_46)
pos_47 = pd.read_csv(pos_47, quoting=csv.QUOTE_NONNUMERIC)
pos_47 = pos_47.to_numpy()
pos.append(pos_47)
pos_48 = pd.read_csv(pos_48, quoting=csv.QUOTE_NONNUMERIC)
pos_48 = pos_48.to_numpy()
pos.append(pos_48)
pos_49 = pd.read_csv(pos_49, quoting=csv.QUOTE_NONNUMERIC)
pos_49 = pos_49.to_numpy()
pos.append(pos_49)
pos_50 = pd.read_csv(pos_50, quoting=csv.QUOTE_NONNUMERIC)
pos_50 = pos_50.to_numpy()
pos.append(pos_50)
pos_51 = pd.read_csv(pos_51, quoting=csv.QUOTE_NONNUMERIC)
pos_51 = pos_51.to_numpy()
pos.append(pos_51)
pos_52 = pd.read_csv(pos_52, quoting=csv.QUOTE_NONNUMERIC)
pos_52 = pos_52.to_numpy()
pos.append(pos_52)
pos_53 = pd.read_csv(pos_53, quoting=csv.QUOTE_NONNUMERIC)
pos_53 = pos_53.to_numpy()
pos.append(pos_53)
pos_54 = pd.read_csv(pos_54, quoting=csv.QUOTE_NONNUMERIC)
pos_54 = pos_54.to_numpy()
pos.append(pos_54)
pos_55 = pd.read_csv(pos_55, quoting=csv.QUOTE_NONNUMERIC)
pos_55 = pos_55.to_numpy()
pos.append(pos_55)
pos_56 = pd.read_csv(pos_56, quoting=csv.QUOTE_NONNUMERIC)
pos_56 = pos_56.to_numpy()
pos.append(pos_56)
pos_57 = pd.read_csv(pos_57, quoting=csv.QUOTE_NONNUMERIC)
pos_57 = pos_57.to_numpy()
pos.append(pos_57)
pos_58 = pd.read_csv(pos_58, quoting=csv.QUOTE_NONNUMERIC)
pos_58 = pos_58.to_numpy()
pos.append(pos_58)
pos_59 = pd.read_csv(pos_59, quoting=csv.QUOTE_NONNUMERIC)
pos_59 = pos_59.to_numpy()
pos.append(pos_59)
pos_60 = pd.read_csv(pos_60, quoting=csv.QUOTE_NONNUMERIC)
pos_60 = pos_60.to_numpy()
pos.append(pos_60)
pos_61 = pd.read_csv(pos_61, quoting=csv.QUOTE_NONNUMERIC)
pos_61 = pos_61.to_numpy()
pos.append(pos_61)
pos_62 = pd.read_csv(pos_62, quoting=csv.QUOTE_NONNUMERIC)
pos_62 = pos_62.to_numpy()
pos.append(pos_62)
pos_63 = pd.read_csv(pos_63, quoting=csv.QUOTE_NONNUMERIC)
pos_63 = pos_63.to_numpy()
pos.append(pos_63)
pos_64 = pd.read_csv(pos_64, quoting=csv.QUOTE_NONNUMERIC)
pos_64 = pos_64.to_numpy()
pos.append(pos_64)
pos_65 = pd.read_csv(pos_65, quoting=csv.QUOTE_NONNUMERIC)
pos_65 = pos_65.to_numpy()
pos.append(pos_65)
pos_66 = pd.read_csv(pos_66, quoting=csv.QUOTE_NONNUMERIC)
pos_66 = pos_66.to_numpy()
pos.append(pos_66)
pos_67 = pd.read_csv(pos_67, quoting=csv.QUOTE_NONNUMERIC)
pos_67 = pos_67.to_numpy()
pos.append(pos_67)
pos_68 = pd.read_csv(pos_68, quoting=csv.QUOTE_NONNUMERIC)
pos_68 = pos_68.to_numpy()
pos.append(pos_68)
pos_69 = pd.read_csv(pos_69, quoting=csv.QUOTE_NONNUMERIC)
pos_69 = pos_69.to_numpy()
pos.append(pos_69)
pos_70 = pd.read_csv(pos_70, quoting=csv.QUOTE_NONNUMERIC)
pos_70 = pos_70.to_numpy()
pos.append(pos_70)
pos_71 = pd.read_csv(pos_71, quoting=csv.QUOTE_NONNUMERIC)
pos_71 = pos_71.to_numpy()
pos.append(pos_71)
pos_72 = pd.read_csv(pos_72, quoting=csv.QUOTE_NONNUMERIC)
pos_72 = pos_72.to_numpy()
pos.append(pos_72)
pos_73 = pd.read_csv(pos_73, quoting=csv.QUOTE_NONNUMERIC)
pos_73 = pos_73.to_numpy()
pos.append(pos_73)
pos_74 = pd.read_csv(pos_74, quoting=csv.QUOTE_NONNUMERIC)
pos_74 = pos_74.to_numpy()
pos.append(pos_74)
pos_75 = pd.read_csv(pos_75, quoting=csv.QUOTE_NONNUMERIC)
pos_75 = pos_75.to_numpy()
pos.append(pos_75)
pos_76 = pd.read_csv(pos_76, quoting=csv.QUOTE_NONNUMERIC)
pos_76 = pos_76.to_numpy()
pos.append(pos_76)
pos_77 = pd.read_csv(pos_77, quoting=csv.QUOTE_NONNUMERIC)
pos_77 = pos_77.to_numpy()
pos.append(pos_77)
pos_78 = pd.read_csv(pos_78, quoting=csv.QUOTE_NONNUMERIC)
pos_78 = pos_78.to_numpy()
pos.append(pos_78)
pos_79 = pd.read_csv(pos_79, quoting=csv.QUOTE_NONNUMERIC)
pos_79 = pos_79.to_numpy()
pos.append(pos_79)
pos_80 = pd.read_csv(pos_80, quoting=csv.QUOTE_NONNUMERIC)
pos_80 = pos_80.to_numpy()
pos.append(pos_80)
pos_81 = pd.read_csv(pos_81, quoting=csv.QUOTE_NONNUMERIC)
pos_81 = pos_81.to_numpy()
pos.append(pos_81)
pos_82 = pd.read_csv(pos_82, quoting=csv.QUOTE_NONNUMERIC)
pos_82 = pos_82.to_numpy()
pos.append(pos_82)
pos_83 = pd.read_csv(pos_83, quoting=csv.QUOTE_NONNUMERIC)
pos_83 = pos_83.to_numpy()
pos.append(pos_83)
pos_84 = pd.read_csv(pos_84, quoting=csv.QUOTE_NONNUMERIC)
pos_84 = pos_84.to_numpy()
pos.append(pos_84)
pos_85 = pd.read_csv(pos_85, quoting=csv.QUOTE_NONNUMERIC)
pos_85 = pos_85.to_numpy()
pos.append(pos_85)
pos_86 = pd.read_csv(pos_86, quoting=csv.QUOTE_NONNUMERIC)
pos_86 = pos_86.to_numpy()
pos.append(pos_86)
pos_87 = pd.read_csv(pos_87, quoting=csv.QUOTE_NONNUMERIC)
pos_87 = pos_87.to_numpy()
pos.append(pos_87)
pos_88 = pd.read_csv(pos_88, quoting=csv.QUOTE_NONNUMERIC)
pos_88 = pos_88.to_numpy()
pos.append(pos_88)
pos_89 = pd.read_csv(pos_89, quoting=csv.QUOTE_NONNUMERIC)
pos_89 = pos_89.to_numpy()
pos.append(pos_89)
pos_90 = pd.read_csv(pos_90, quoting=csv.QUOTE_NONNUMERIC)
pos_90 = pos_90.to_numpy()
pos.append(pos_90)
pos_91 = pd.read_csv(pos_91, quoting=csv.QUOTE_NONNUMERIC)
pos_91 = pos_91.to_numpy()
pos.append(pos_91)
pos_92 = pd.read_csv(pos_92, quoting=csv.QUOTE_NONNUMERIC)
pos_92 = pos_92.to_numpy()
pos.append(pos_92)
pos_93 = pd.read_csv(pos_93, quoting=csv.QUOTE_NONNUMERIC)
pos_93 = pos_93.to_numpy()
pos.append(pos_93)
pos_94 = pd.read_csv(pos_94, quoting=csv.QUOTE_NONNUMERIC)
pos_94 = pos_94.to_numpy()
pos.append(pos_94)
pos_95 = pd.read_csv(pos_95, quoting=csv.QUOTE_NONNUMERIC)
pos_95 = pos_95.to_numpy()
pos.append(pos_95)
pos_96 = pd.read_csv(pos_96, quoting=csv.QUOTE_NONNUMERIC)
pos_96 = pos_96.to_numpy()
pos.append(pos_96)
pos_97 = pd.read_csv(pos_97, quoting=csv.QUOTE_NONNUMERIC)
pos_97 = pos_97.to_numpy()
pos.append(pos_97)
pos_98 = pd.read_csv(pos_98, quoting=csv.QUOTE_NONNUMERIC)
pos_98 = pos_98.to_numpy()
pos.append(pos_98)
pos_99 = pd.read_csv(pos_99, quoting=csv.QUOTE_NONNUMERIC)
pos_99 = pos_99.to_numpy()
pos.append(pos_99)
pos_100 = pd.read_csv(pos_100, quoting=csv.QUOTE_NONNUMERIC)
pos_100 = pos_100.to_numpy()
pos.append(pos_100)
pos_101 = pd.read_csv(pos_101, quoting=csv.QUOTE_NONNUMERIC)
pos_101 = pos_101.to_numpy()
pos.append(pos_101)
pos_102 = pd.read_csv(pos_102, quoting=csv.QUOTE_NONNUMERIC)
pos_102 = pos_102.to_numpy()
pos.append(pos_102)
pos_103 = pd.read_csv(pos_103, quoting=csv.QUOTE_NONNUMERIC)
pos_103 = pos_103.to_numpy()
pos.append(pos_103)
pos_104 = pd.read_csv(pos_104, quoting=csv.QUOTE_NONNUMERIC)
pos_104 = pos_104.to_numpy()
pos.append(pos_104)
pos_105 = pd.read_csv(pos_105, quoting=csv.QUOTE_NONNUMERIC)
pos_105 = pos_105.to_numpy()
pos.append(pos_105)
pos_106 = pd.read_csv(pos_106, quoting=csv.QUOTE_NONNUMERIC)
pos_106 = pos_106.to_numpy()
pos.append(pos_106)
pos_107 = pd.read_csv(pos_107, quoting=csv.QUOTE_NONNUMERIC)
pos_107 = pos_107.to_numpy()
pos.append(pos_107)
pos_108 = pd.read_csv(pos_108, quoting=csv.QUOTE_NONNUMERIC)
pos_108 = pos_108.to_numpy()
pos.append(pos_108)
pos_109 = pd.read_csv(pos_109, quoting=csv.QUOTE_NONNUMERIC)
pos_109 = pos_109.to_numpy()
pos.append(pos_109)
pos_110 = pd.read_csv(pos_110, quoting=csv.QUOTE_NONNUMERIC)
pos_110 = pos_110.to_numpy()
pos.append(pos_110)
pos_111 = pd.read_csv(pos_111, quoting=csv.QUOTE_NONNUMERIC)
pos_111 = pos_111.to_numpy()
pos.append(pos_111)
pos_112 = pd.read_csv(pos_112, quoting=csv.QUOTE_NONNUMERIC)
pos_112 = pos_112.to_numpy()
pos.append(pos_112)
pos_113 = pd.read_csv(pos_113, quoting=csv.QUOTE_NONNUMERIC)
pos_113 = pos_113.to_numpy()
pos.append(pos_113)
pos_114 = pd.read_csv(pos_114, quoting=csv.QUOTE_NONNUMERIC)
pos_114 = pos_114.to_numpy()
pos.append(pos_114)
pos_115 = pd.read_csv(pos_115, quoting=csv.QUOTE_NONNUMERIC)
pos_115 = pos_115.to_numpy()
pos.append(pos_115)
pos_116 = pd.read_csv(pos_116, quoting=csv.QUOTE_NONNUMERIC)
pos_116 = pos_116.to_numpy()
pos.append(pos_116)
pos_117 = pd.read_csv(pos_117, quoting=csv.QUOTE_NONNUMERIC)
pos_117 = pos_117.to_numpy()
pos.append(pos_117)
pos_118 = pd.read_csv(pos_118, quoting=csv.QUOTE_NONNUMERIC)
pos_118 = pos_118.to_numpy()
pos.append(pos_118)
pos_119 = pd.read_csv(pos_119, quoting=csv.QUOTE_NONNUMERIC)
pos_119 = pos_119.to_numpy()
pos.append(pos_119)
pos_120 = pd.read_csv(pos_120, quoting=csv.QUOTE_NONNUMERIC)
pos_120 = pos_120.to_numpy()
pos.append(pos_120)
pos_121 = pd.read_csv(pos_121, quoting=csv.QUOTE_NONNUMERIC)
pos_121 = pos_121.to_numpy()
pos.append(pos_121)
pos_122 = pd.read_csv(pos_122, quoting=csv.QUOTE_NONNUMERIC)
pos_122 = pos_122.to_numpy()
pos.append(pos_122)
pos_123 = pd.read_csv(pos_123, quoting=csv.QUOTE_NONNUMERIC)
pos_123 = pos_123.to_numpy()
pos.append(pos_123)
pos_124 = pd.read_csv(pos_124, quoting=csv.QUOTE_NONNUMERIC)
pos_124 = pos_124.to_numpy()
pos.append(pos_124)
pos_125 = pd.read_csv(pos_125, quoting=csv.QUOTE_NONNUMERIC)
pos_125 = pos_125.to_numpy()
pos.append(pos_125)
pos_126 = pd.read_csv(pos_126, quoting=csv.QUOTE_NONNUMERIC)
pos_126 = pos_126.to_numpy()
pos.append(pos_126)
pos_127 = pd.read_csv(pos_127, quoting=csv.QUOTE_NONNUMERIC)
pos_127 = pos_127.to_numpy()
pos.append(pos_127)
pos_128 = pd.read_csv(pos_128, quoting=csv.QUOTE_NONNUMERIC)
pos_128 = pos_128.to_numpy()
pos.append(pos_128)
pos_129 = pd.read_csv(pos_129, quoting=csv.QUOTE_NONNUMERIC)
pos_129 = pos_129.to_numpy()
pos.append(pos_129)
pos_130 = pd.read_csv(pos_130, quoting=csv.QUOTE_NONNUMERIC)
pos_130 = pos_130.to_numpy()
pos.append(pos_130)
pos_131 = pd.read_csv(pos_131, quoting=csv.QUOTE_NONNUMERIC)
pos_131 = pos_131.to_numpy()
pos.append(pos_131)
pos_132 = pd.read_csv(pos_132, quoting=csv.QUOTE_NONNUMERIC)
pos_132 = pos_132.to_numpy()
pos.append(pos_132)
pos_133 = pd.read_csv(pos_133, quoting=csv.QUOTE_NONNUMERIC)
pos_133 = pos_133.to_numpy()
pos.append(pos_133)
pos_134 = pd.read_csv(pos_134, quoting=csv.QUOTE_NONNUMERIC)
pos_134 = pos_134.to_numpy()
pos.append(pos_134)
pos_135 = pd.read_csv(pos_135, quoting=csv.QUOTE_NONNUMERIC)
pos_135 = pos_135.to_numpy()
pos.append(pos_135)
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
# EVAL
pos_151 = pd.read_csv(pos_151, quoting=csv.QUOTE_NONNUMERIC)
pos_151 = pos_151.to_numpy()
pos.append(pos_151)
pos_152 = pd.read_csv(pos_152, quoting=csv.QUOTE_NONNUMERIC)
pos_152 = pos_152.to_numpy()
pos.append(pos_152)
pos_153 = pd.read_csv(pos_153, quoting=csv.QUOTE_NONNUMERIC)
pos_153 = pos_153.to_numpy()
pos.append(pos_153)
pos_154 = pd.read_csv(pos_154, quoting=csv.QUOTE_NONNUMERIC)
pos_154 = pos_154.to_numpy()
pos.append(pos_154)
pos_155 = pd.read_csv(pos_155, quoting=csv.QUOTE_NONNUMERIC)
pos_155 = pos_155.to_numpy()
pos.append(pos_155)
pos_156 = pd.read_csv(pos_156, quoting=csv.QUOTE_NONNUMERIC)
pos_156 = pos_156.to_numpy()
pos.append(pos_156)
pos_157 = pd.read_csv(pos_157, quoting=csv.QUOTE_NONNUMERIC)
pos_157 = pos_157.to_numpy()
pos.append(pos_157)
pos_158 = pd.read_csv(pos_158, quoting=csv.QUOTE_NONNUMERIC)
pos_158 = pos_158.to_numpy()
pos.append(pos_158)
pos_159 = pd.read_csv(pos_159, quoting=csv.QUOTE_NONNUMERIC)
pos_159 = pos_159.to_numpy()
pos.append(pos_159)
pos_160 = pd.read_csv(pos_160, quoting=csv.QUOTE_NONNUMERIC)
pos_160 = pos_160.to_numpy()
pos.append(pos_160)
pos_161 = pd.read_csv(pos_161, quoting=csv.QUOTE_NONNUMERIC)
pos_161 = pos_161.to_numpy()
pos.append(pos_161)
pos_162 = pd.read_csv(pos_162, quoting=csv.QUOTE_NONNUMERIC)
pos_162 = pos_162.to_numpy()
pos.append(pos_162)
pos_163 = pd.read_csv(pos_163, quoting=csv.QUOTE_NONNUMERIC)
pos_163 = pos_163.to_numpy()
pos.append(pos_163)
pos_164 = pd.read_csv(pos_164, quoting=csv.QUOTE_NONNUMERIC)
pos_164 = pos_164.to_numpy()
pos.append(pos_164)
pos_165 = pd.read_csv(pos_165, quoting=csv.QUOTE_NONNUMERIC)
pos_165 = pos_165.to_numpy()
pos.append(pos_165)
pos_166 = pd.read_csv(pos_166, quoting=csv.QUOTE_NONNUMERIC)
pos_166 = pos_166.to_numpy()
pos.append(pos_166)
pos_167 = pd.read_csv(pos_167, quoting=csv.QUOTE_NONNUMERIC)
pos_167 = pos_167.to_numpy()
pos.append(pos_167)
pos_168 = pd.read_csv(pos_168, quoting=csv.QUOTE_NONNUMERIC)
pos_168 = pos_168.to_numpy()
pos.append(pos_168)
pos_169 = pd.read_csv(pos_169, quoting=csv.QUOTE_NONNUMERIC)
pos_169 = pos_169.to_numpy()
pos.append(pos_169)
pos_170 = pd.read_csv(pos_170, quoting=csv.QUOTE_NONNUMERIC)
pos_170 = pos_170.to_numpy()
pos.append(pos_170)
pos_171 = pd.read_csv(pos_171, quoting=csv.QUOTE_NONNUMERIC)
pos_171 = pos_171.to_numpy()
pos.append(pos_171)
pos_172 = pd.read_csv(pos_172, quoting=csv.QUOTE_NONNUMERIC)
pos_172 = pos_172.to_numpy()
pos.append(pos_172)
pos_173 = pd.read_csv(pos_173, quoting=csv.QUOTE_NONNUMERIC)
pos_173 = pos_173.to_numpy()
pos.append(pos_173)
pos_174 = pd.read_csv(pos_174, quoting=csv.QUOTE_NONNUMERIC)
pos_174 = pos_174.to_numpy()
pos.append(pos_174)
pos_175 = pd.read_csv(pos_175, quoting=csv.QUOTE_NONNUMERIC)
pos_175 = pos_175.to_numpy()
pos.append(pos_175)
pos_176 = pd.read_csv(pos_176, quoting=csv.QUOTE_NONNUMERIC)
pos_176 = pos_176.to_numpy()
pos.append(pos_176)
pos_177 = pd.read_csv(pos_177, quoting=csv.QUOTE_NONNUMERIC)
pos_177 = pos_177.to_numpy()
pos.append(pos_177)
pos_178 = pd.read_csv(pos_178, quoting=csv.QUOTE_NONNUMERIC)
pos_178 = pos_178.to_numpy()
pos.append(pos_178)
pos_179 = pd.read_csv(pos_179, quoting=csv.QUOTE_NONNUMERIC)
pos_179 = pos_179.to_numpy()
pos.append(pos_179)
pos_180 = pd.read_csv(pos_180, quoting=csv.QUOTE_NONNUMERIC)
pos_180 = pos_180.to_numpy()
pos.append(pos_180)
# TEST
pos_181 = pd.read_csv(pos_181, quoting=csv.QUOTE_NONNUMERIC)
pos_181 = pos_181.to_numpy()
pos.append(pos_181)
# pos_182 = pd.read_csv(pos_182, quoting=csv.QUOTE_NONNUMERIC)
# pos_182 = pos_182.to_numpy()
# pos.append(pos_182)
# pos_183 = pd.read_csv(pos_183, quoting=csv.QUOTE_NONNUMERIC)
# pos_183 = pos_183.to_numpy()
# pos.append(pos_183)
# pos_184 = pd.read_csv(pos_184, quoting=csv.QUOTE_NONNUMERIC)
# pos_184 = pos_184.to_numpy()
# pos.append(pos_184)
# pos_185 = pd.read_csv(pos_185, quoting=csv.QUOTE_NONNUMERIC)
# pos_185 = pos_185.to_numpy()
# pos.append(pos_185)
# pos_186 = pd.read_csv(pos_186, quoting=csv.QUOTE_NONNUMERIC)
# pos_186 = pos_186.to_numpy()
# pos.append(pos_186)
# pos_187 = pd.read_csv(pos_187, quoting=csv.QUOTE_NONNUMERIC)
# pos_187 = pos_187.to_numpy()
# pos.append(pos_187)
# pos_188 = pd.read_csv(pos_188, quoting=csv.QUOTE_NONNUMERIC)
# pos_188 = pos_188.to_numpy()
# pos.append(pos_188)
# pos_189 = pd.read_csv(pos_189, quoting=csv.QUOTE_NONNUMERIC)
# pos_189 = pos_189.to_numpy()
# pos.append(pos_189)
# pos_190 = pd.read_csv(pos_190, quoting=csv.QUOTE_NONNUMERIC)
# pos_190 = pos_190.to_numpy()
# pos.append(pos_190)
# pos_191 = pd.read_csv(pos_191, quoting=csv.QUOTE_NONNUMERIC)
# pos_191 = pos_191.to_numpy()
# pos.append(pos_191)
# pos_192 = pd.read_csv(pos_192, quoting=csv.QUOTE_NONNUMERIC)
# pos_192 = pos_192.to_numpy()
# pos.append(pos_192)
# pos_193 = pd.read_csv(pos_193, quoting=csv.QUOTE_NONNUMERIC)
# pos_193 = pos_193.to_numpy()
# pos.append(pos_193)
# pos_194 = pd.read_csv(pos_194, quoting=csv.QUOTE_NONNUMERIC)
# pos_194 = pos_194.to_numpy()
# pos.append(pos_194)
# pos_195 = pd.read_csv(pos_195, quoting=csv.QUOTE_NONNUMERIC)
# pos_195 = pos_195.to_numpy()
# pos.append(pos_195)

human_spheres = rospy.Publisher('/Obstacle/human_spheres', Float64MultiArray, queue_size=1)
# future_temp = [0,0,0,0,0,0,0,0,0,0]   #0s
# future_temp = [125,125,125,125,125,125,125,125,125,125] #0.5s
# future_temp = [125,250,250,250,250,250,250,250,250,250] #1s
# future_temp = [125,250,375,375,375,375,375,375,375,375] #1.5s
# future_temp = [125,250,375,500,500,500,500,500,500,500] #2s
# future_temp = [125,250,375,500,625,625,625,625,625,625] #2.5s
# future_temp = [125,250,375,500,625,750,750,750,750,750] #3s
# future_temp = [125,250,375,500,625,750,875,875,875,875] #3.5s
# future_temp = [125,250,375,500,625,750,875,1000,1000,1000] #4s
# future_temp = [125,250,375,500,625,750,875,1000,1125,1125] #4.5s
future_temp = [125,250,375,500,625,750,875,1000,1125,1250] #5s
print("start")

condition_h = 1
def human_condition(data):
    global condition_h
    if data.data == 'stop_human':
        condition_h = 1
    else:
        condition_h = 0

def main():
    global pos
    rospy.init_node('human_control', anonymous=True)
    sleep_time = 0.0
    # msg = rospy.wait_for_message("/HighController/start", Int32)
    msg = rospy.wait_for_message("/flag", String)
    rospy.Subscriber("/flag", String, human_condition)
    # print("MSG")
    # msg = True
    if(msg):
        d = 1.0
        for l in range (5):
            for k in range (len(pos)):
                time.sleep(sleep_time)
                temp = pos[k]
                print(l,k)
                for i in range (len(temp)-future_temp[9]):
                    point_array = [0]*463
                    # print(temp[i][3*0], future_temp[9], (temp[i+future_temp[9]][3*0]))
                    for a in range(14):
                        point_array[3*a] = (temp[i][3*a])+d
                        point_array[3*a+1] = (temp[i][3*a+1])+d
                        point_array[3*a+2] = (temp[i][3*a+2])
                    point_array[42] = k+1
                    for f in range(10):
                        for a in range(14):    
                            point_array[f*42+43+3*a] = (temp[i+future_temp[f]][3*a])+d
                            point_array[f*42+43+3*a+1] = (temp[i+future_temp[f]][3*a+1])+d
                            point_array[f*42+43+3*a+2] = (temp[i+future_temp[f]][3*a+2])
                    obstacle_data = Float64MultiArray()
                    obstacle_data.data = point_array
                    if condition_h==0: 
                        human_spheres.publish(obstacle_data)
                        # print("wake")
                    else:
                        time.sleep(0.05)
                        # print("sleep")
                    # human_spheres.publish(obstacle_data)
                    time.sleep(0.004)
            sleep_time+=0.1
        rospy.spin()
        print("the end")
    print("exit")    

if __name__ == '__main__': main()
