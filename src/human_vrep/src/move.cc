#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <sensor_msgs/JointState.h>
float point_array_temp[58];
float point_array_temp_high[560];
float point_array[58];
float point_array_high[560];
double from_high[31] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10};
float sphere_radi[14]={0.5510,0.6010,0.5010,0.5010,0.5010,0.5010,0.5010,0.5010,0.4510,0.4510,0.4810,0.4810,0.5510,0.6010};

void chatterCallback(const std_msgs::Float64MultiArray msg)
{
  for (int i = 0; i<14; i++){
    point_array_temp[i*4] = msg.data[i*3];
    point_array_temp[i*4+1] = msg.data[i*3+1];
    point_array_temp[i*4+2] = msg.data[i*3+2];
    point_array_temp[i*4+3] = sphere_radi[i];
  }
  for (int j = 0; j<10; j++){
    for (int i = 0; i<14; i++){
      point_array_temp_high[j*56+i*4] = msg.data[j*42+43+i*3];
      point_array_temp_high[j*56+i*4+1] = msg.data[j*42+43+i*3+1];
      point_array_temp_high[j*56+i*4+2] = msg.data[j*42+43+i*3+2];
      point_array_temp_high[j*56+i*4+3] = sphere_radi[i];
    }
  }
  point_array_temp[56] = msg.data[463];
  point_array_temp[57] = msg.data[464];
}
double state_feedback_temp[12];
double state_feedback[12];
void feedbackCB(const sensor_msgs::JointState msg) 
{
  for (int i = 0; i < 6; ++i) 
  {
    state_feedback_temp[i] = msg.position[i];
    state_feedback_temp[i+6] = msg.velocity[i];
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_control");
  ros::NodeHandle nodeHandle("~");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<std_msgs::Float64MultiArray>("/Obstacle/human_spheres", 1, chatterCallback);
  ros::Subscriber arm_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, feedbackCB);
  ros:: Publisher chatter_low = n.advertise<std_msgs::Float64MultiArray>("/Obstacle/mpc_low_spheres", 1);
  ros:: Publisher chatter_high = n.advertise<std_msgs::Float64MultiArray>("/Obstacle/mpc_high_spheres", 1);
  ros:: Publisher states_low = n.advertise<std_msgs::Float64MultiArray>("/joint_states_low", 1);
  ros:: Publisher states_high = n.advertise<std_msgs::Float64MultiArray>("/joint_states_high", 1);
  // ros::Rate loop_rate(40);
  while (ros::ok())
  {
    for (int i = 0; i<14; i++) {
      point_array[i*4] = point_array_temp[i*4]; // x offset
      point_array[i*4+1] = point_array_temp[i*4+1]; // y offset
      point_array[i*4+2] = point_array_temp[i*4+2]-1.2; // z offset
      point_array[i*4+3] = point_array_temp[i*4+3];
    }
    
    point_array[56]=point_array_temp[56];
    point_array[57]=point_array_temp[57];

    for (int j = 0; j<10; j++){
      for (int i = 0; i<14; i++) {
        point_array_high[j*56+i*4] = point_array_temp_high[j*56+i*4]; // x offset
        point_array_high[j*56+i*4+1] = point_array_temp_high[j*56+i*4+1]; // y offset
        point_array_high[j*56+i*4+2] = point_array_temp_high[j*56+i*4+2]-1.2; // z offset
        point_array_high[j*56+i*4+3] = point_array_temp_high[j*56+i*4+3];
      }
    }
    // printf("low=%f, high = %f\n",point_array[0], point_array_high[0]);
    for (int i = 0; i < 12; ++i) state_feedback[i] = state_feedback_temp[i];
    std_msgs::Float64MultiArray obstacle_data;
    obstacle_data.data.clear();
    for (int i = 0; i < 58; i++){
      obstacle_data.data.push_back(point_array[i]);
    } 
    chatter_low.publish(obstacle_data);
    std_msgs::Float64MultiArray obstacle_data_high;
    obstacle_data_high.data.clear();
    for (int i = 0; i < 560; i++){
      obstacle_data_high.data.push_back(point_array_high[i]);
    } 
    chatter_high.publish(obstacle_data_high);

    std_msgs::Float64MultiArray state_data;
    state_data.data.clear();
    for (int i = 0; i < 12; i++) state_data.data.push_back(state_feedback[i]);
    states_low.publish(state_data);
    states_high.publish(state_data);

    ros::spinOnce();
  }
  ros::spin();
  return 0;
}

