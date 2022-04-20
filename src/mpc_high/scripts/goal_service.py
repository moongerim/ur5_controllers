#!/usr/bin/env python
import rospy
import time
from gazebo_msgs.srv import SetModelConfiguration

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock

from numpy import genfromtxt
my_data = genfromtxt('/home/robot/October/Bench17/SSM_cont/src/gen3_p/scripts/trajectory_exp.txt', delimiter=' ')
num_of_points = len(my_data)
print("num_of_points",num_of_points)

position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def clock_callback(data):
    global sim_time
    sim_time = data.clock.secs + data.clock.nsecs*1000000

def callback(data):
    global position
    global velocity
    for i in range(7):
        position[i]=data.position[i]
        velocity[i]=data.velocity[i] 
    
def listener():
    global position
    global velocity
    global my_data
    P = 12.0 # 12 is too much
    goal_pose = [0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0]
    gazebo_namespace = '/gazebo'
    model_name = 'my_phantom_gen3'
    model_param_name = 'robot_description'
    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/my_gen3/joint_states", JointState, callback)
    print('connecting')
    rospy.loginfo("Waiting for service %s/set_model_configuration"%gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace+'/set_model_configuration')
    rate = rospy.Rate(20) # 50ms
    vells_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal_iterator = 0
    k='-'
    while not rospy.is_shutdown():
        start = time.time()
        goal_pose = [my_data[goal_iterator][0], my_data[goal_iterator][1], my_data[goal_iterator][2], my_data[goal_iterator][3], my_data[goal_iterator][4], my_data[goal_iterator][5], my_data[goal_iterator][6]]
        #rospy.loginfo(k)# to print on  terminal
        joint_positions = goal_pose
        try:
            start = time.time()
            set_model_configuration = rospy.ServiceProxy(gazebo_namespace+'/set_model_configuration', SetModelConfiguration)
            #rospy.loginfo("Calling service %s/set_model_configuration"%gazebo_namespace)
            #resp = set_model_configuration(model_name, model_param_name, joint_names, joint_positions)
            resp = set_model_configuration(model_name, model_param_name, joint_names, position)
            rospy.loginfo("Set model configuration status: %s, time %f"%(resp.status_message, time.time() - start))
            #print('Elapsed time',time.time() - start)
            #print(resp.success)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
        goal_iterator=(goal_iterator+1)%len(my_data)
        rate.sleep()
    print('well done, I am out')

if __name__ == '__main__':
    listener()
