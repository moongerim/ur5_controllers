#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "std_msgs/Int32.h"
#include <time.h>

using namespace std;

ofstream myfile;
int rti_num = 20;
clock_t start;
double cpu_time;
double state_feedback[12];
double given_vels[6];
void feedbackCB(const sensor_msgs::JointState msg) 
{
  for (int i = 0; i < 6; ++i) 
  {
    state_feedback[i] = msg.position[i];
    state_feedback[i+6] = msg.velocity[i];
  }
}

void given_jv(const std_msgs::Float64MultiArray msg) 
{
  for (int i = 0; i < 7; ++i) 
  {
    given_vels[i] = msg.data[i];
  }
}

// cartesian positions of the 10 test points:
Eigen::MatrixXf get_cpose(float theta_1, float theta_2, float theta_3, float theta_4, float theta_5, float theta_6){
  Eigen::MatrixXf mat(3,11);
  mat <<0, 1*(0.4*sin(theta_1)-0.425*cos(theta_1)*cos(theta_2))/3,  2*(0.2*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2))/3,   0.11*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2), -0.425*cos(theta_1)*cos(theta_2)+(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/4, -0.425*cos(theta_1)*cos(theta_2)+2*(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/4, -0.425*cos(theta_1)*cos(theta_2)+3*(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/4, -(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000, 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3), 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.09465*cos(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2)) - 0.09465*sin(theta_4)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - 1.0*cos(theta_1)*cos(theta_2)*cos(theta_3)) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3), 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.0823*cos(theta_5)*sin(theta_1) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3),
        0, 1*(-0.4*cos(theta_1)-0.425*cos(theta_2)*sin(theta_1))/3, 2*(-0.2*cos(theta_1) - 0.425*cos(theta_2)*sin(theta_1))/3, -0.11*cos(theta_1) - 0.425*cos(theta_2)*sin(theta_1), -0.425*cos(theta_2)*sin(theta_1)+(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/4, -0.425*cos(theta_2)*sin(theta_1)+2*(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/4, -0.425*cos(theta_2)*sin(theta_1)+3*(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/4, -(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000, 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1), 0.09465*cos(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2)) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.09465*sin(theta_4)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - 1.0*cos(theta_2)*cos(theta_3)*sin(theta_1)) + 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1), 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_1)*cos(theta_5) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1),
        0, 0.08945+(0.08945-0.425*sin(theta_2)-0.08945)/3,          0.08945+2*(0.08945-0.425*sin(theta_2)-0.08945)/3, 0.08945  -0.425*sin(theta_2),                                  0.08945 - 0.425*sin(theta_2)+(0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.08945 - 0.425*sin(theta_2)))/4,                       0.08945 - 0.425*sin(theta_2)+2*(0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.08945 - 0.425*sin(theta_2)))/4,                      0.08945 - 0.425*sin(theta_2)+3*(0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.08945 - 0.425*sin(theta_2)))/4,                      0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3),             0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3),                                                                                            0.08945 - 0.39225*sin(theta_2 + theta_3) - 0.425*sin(theta_2) - 0.09465*cos(theta_2 + theta_3 + theta_4),                                                                                                                                                                                                                                                                       0.09465*sin(theta_2 + theta_3)*sin(theta_4) - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3) - sin(theta_5)*(0.0823*cos(theta_2 + theta_3)*sin(theta_4) + 0.0823*sin(theta_2 + theta_3)*cos(theta_4)) - 0.09465*cos(theta_2 + theta_3)*cos(theta_4) + 0.08945;
 return mat;
}

Eigen::MatrixXf get_velocity(float theta_1, float theta_2, float theta_3, float theta_4, float theta_5, float theta_6,
                             float u_1, float u_2, float u_3, float u_4, float u_5, float u_6){
	Eigen::MatrixXf mat(30,1);
  mat <<(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2))/3,
        (0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2))/3,
        (-0.425*u_2*cos(theta_2))/3,
        2*(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2))/3,
        2*(0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2))/3,
        2*(-0.425*u_2*cos(theta_2))/3,
        0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2),
        0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2),
        -0.425*u_2*cos(theta_2),
        0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)+1/4*(u_2*(0.425*cos(theta_1)*sin(theta_2) + 0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)) + u_1*(0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + u_3*(0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2))-(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2))),
        0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)+1/4*(u_3*(0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*u_1*(0.425*cos(theta_1)*cos(theta_2) - 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + u_2*(0.425*sin(theta_1)*sin(theta_2) + 0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2))-(0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2))),
        -0.425*u_2*cos(theta_2)+1/4*(- 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3)-(-0.425*u_2*cos(theta_2))),
        0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)+2/4*(u_2*(0.425*cos(theta_1)*sin(theta_2) + 0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)) + u_1*(0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + u_3*(0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2))-(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2))),
        0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)+2/4*(u_3*(0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*u_1*(0.425*cos(theta_1)*cos(theta_2) - 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + u_2*(0.425*sin(theta_1)*sin(theta_2) + 0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2))-(0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2))),
        -0.425*u_2*cos(theta_2)+2/4*(- 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3)-(-0.425*u_2*cos(theta_2))),
        0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)+3/4*(u_2*(0.425*cos(theta_1)*sin(theta_2) + 0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)) + u_1*(0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + u_3*(0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2))-(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2))),
        0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)+3/4*(u_3*(0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*u_1*(0.425*cos(theta_1)*cos(theta_2) - 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + u_2*(0.425*sin(theta_1)*sin(theta_2) + 0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2))-(0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2))),
        -0.425*u_2*cos(theta_2)+3/4*(- 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3)-(-0.425*u_2*cos(theta_2))),
        u_2*(0.425*cos(theta_1)*sin(theta_2) + 0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)) + u_1*(0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + u_3*(0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)),
        u_3*(0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*u_1*(0.425*cos(theta_1)*cos(theta_2) - 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + u_2*(0.425*sin(theta_1)*sin(theta_2) + 0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)),
        - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3),
        u_1*(0.10915*cos(theta_1) + 0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + 0.00025*u_2*cos(theta_1)*(1569.0*sin(theta_2 + theta_3) + 1700.0*sin(theta_2)) + 0.39225*u_3*sin(theta_2 + theta_3)*cos(theta_1),
        u_1*(0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + 0.00025*u_2*sin(theta_1)*(1569.0*sin(theta_2 + theta_3) + 1700.0*sin(theta_2)) + 0.39225*u_3*sin(theta_2 + theta_3)*sin(theta_1),
        - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3),
        u_1*(0.10915*cos(theta_1) + 0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + 0.00005*u_2*cos(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 7845.0*sin(theta_2 + theta_3) + 8500.0*sin(theta_2)) + 0.00015*u_3*cos(theta_1)*(631.0*cos(theta_2 + theta_3 + theta_4) + 2615.0*sin(theta_2 + theta_3)) + 0.09465*u_4*cos(theta_2 + theta_3 + theta_4)*cos(theta_1),
        u_1*(0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + 0.00005*u_2*sin(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 7845.0*sin(theta_2 + theta_3) + 8500.0*sin(theta_2)) + 0.00015*u_3*sin(theta_1)*(631.0*cos(theta_2 + theta_3 + theta_4) + 2615.0*sin(theta_2 + theta_3)) + 0.09465*u_4*cos(theta_2 + theta_3 + theta_4)*sin(theta_1),
        0.09465*u_2*sin(theta_2 + theta_3 + theta_4) + 0.09465*u_3*sin(theta_2 + theta_3 + theta_4) + 0.09465*u_4*sin(theta_2 + theta_3 + theta_4) - 0.39225*u_2*cos(theta_2 + theta_3) - 0.39225*u_3*cos(theta_2 + theta_3) - 0.425*u_2*cos(theta_2),
        u_1*(0.10915*cos(theta_1) + 0.0823*cos(theta_1)*cos(theta_5) + 0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) - 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) - 1.0*u_5*(0.0823*sin(theta_1)*sin(theta_5) + 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*cos(theta_5)) + 0.00005*u_2*cos(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 823.0*cos(theta_2 + theta_3 + theta_4 - 1.0*theta_5) - 823.0*cos(theta_2 + theta_3 + theta_4 + theta_5) + 7845.0*sin(theta_2 + theta_3) + 8500.0*sin(theta_2)) + 0.00005*u_4*cos(theta_1)*(1646.0*sin(theta_2 + theta_3 + theta_4)*sin(theta_5) - 1893.0*sin(theta_2 + theta_3)*sin(theta_4) + 1893.0*cos(theta_2 + theta_3)*cos(theta_4)) + 0.00005*u_3*cos(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 823.0*cos(theta_2 + theta_3 + theta_4 - 1.0*theta_5) - 823.0*cos(theta_2 + theta_3 + theta_4 + theta_5) + 7845.0*sin(theta_2 + theta_3)),
        u_5*(0.0823*cos(theta_1)*sin(theta_5) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_5)*sin(theta_1)) + u_1*(0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.0823*cos(theta_5)*sin(theta_1) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + 0.00005*u_2*sin(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 823.0*cos(theta_2 + theta_3 + theta_4 - 1.0*theta_5) - 823.0*cos(theta_2 + theta_3 + theta_4 + theta_5) + 7845.0*sin(theta_2 + theta_3) + 8500.0*sin(theta_2)) + 0.00005*u_4*sin(theta_1)*(1646.0*sin(theta_2 + theta_3 + theta_4)*sin(theta_5) - 1893.0*sin(theta_2 + theta_3)*sin(theta_4) + 1893.0*cos(theta_2 + theta_3)*cos(theta_4)) + 0.00005*u_3*sin(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 823.0*cos(theta_2 + theta_3 + theta_4 - 1.0*theta_5) - 823.0*cos(theta_2 + theta_3 + theta_4 + theta_5) + 7845.0*sin(theta_2 + theta_3)),
        u_4*(0.09465*sin(theta_2 + theta_3 + theta_4) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_5)) - 1.0*u_3*(0.39225*cos(theta_2 + theta_3) - 0.09465*sin(theta_2 + theta_3 + theta_4) + 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_5)) - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2) - 0.09465*cos(theta_2 + theta_3)*sin(theta_4) - 0.09465*sin(theta_2 + theta_3)*cos(theta_4) + 0.0823*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_5) - 0.0823*sin(theta_2 + theta_3)*sin(theta_4)*sin(theta_5)) - 0.0823*u_5*sin(theta_2 + theta_3 + theta_4)*cos(theta_5);
    return mat;
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "test_acc");

  ros::NodeHandle n;
  ROS_INFO("Node Started");
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/data", 1);
  ros::Subscriber arm_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, feedbackCB);
  ros::Subscriber vel_sub = n.subscribe<std_msgs::Float64MultiArray>("/given_jv", 1, given_jv);
  // cposes of 10 test points:
  double ctp[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  // linear vels of 10 test points:
  double lin_v_given[10] = {0,0,0,0,0,0,0,0,0,0};
  double lin_v_real[10] = {0,0,0,0,0,0,0,0,0,0};
  ros::Rate loop_rate(200);
  while (ros::ok())
  {
    start = clock();
    cpu_time = ((double) start) / CLOCKS_PER_SEC;
    Eigen::MatrixXf mat2 = get_cpose(state_feedback[0], state_feedback[1],
                                     state_feedback[2], state_feedback[3],
                                     state_feedback[4], state_feedback[5]);

    for (int j = 0; j < 10; j++) {
      Eigen::Vector3f w;
      w = mat2.col(j+1).transpose();
      ctp[j*3+0] = w[0];
      ctp[j*3+1] = w[1];
      ctp[j*3+2] = w[2];
      }
    Eigen::MatrixXf vell_mat = get_velocity(state_feedback[0], state_feedback[1], 
                                            state_feedback[2], state_feedback[3],
                                            state_feedback[4], state_feedback[5],
                                            state_feedback[6], state_feedback[7], 
                                            state_feedback[8], state_feedback[9], 
                                            state_feedback[10], state_feedback[11]);
    for (int k=0; k<10; k++) {
      lin_v_real[k] = sqrt(vell_mat.coeff(k*3 + 0,0)*vell_mat.coeff(k*3 + 0,0) + vell_mat.coeff(k*3 + 1,0)*vell_mat.coeff(k*3 + 1,0) + vell_mat.coeff(k*3 + 2,0)*vell_mat.coeff(k*3 + 2,0));
    }

    Eigen::MatrixXf vell_mat_given = get_velocity(state_feedback[0], state_feedback[1], 
                                            state_feedback[2], state_feedback[3],
                                            state_feedback[4], state_feedback[5],
                                            given_vels[0], given_vels[1], 
                                            given_vels[2], given_vels[3], 
                                            given_vels[4], given_vels[5]);
    for (int k=0; k<10; k++) {
      lin_v_given[k] = sqrt(vell_mat_given.coeff(k*3 + 0,0)*vell_mat_given.coeff(k*3 + 0,0) + vell_mat_given.coeff(k*3 + 1,0)*vell_mat_given.coeff(k*3 + 1,0) + vell_mat_given.coeff(k*3 + 2,0)*vell_mat_given.coeff(k*3 + 2,0));
    }

    std_msgs::Float64MultiArray values;
    values.data.clear();
    for (int i = 0; i < 12; i++) values.data.push_back(state_feedback[i]);
    for (int i = 0; i < 10; i++) values.data.push_back(lin_v_real[i]);
    for (int i = 0; i < 10; i++) values.data.push_back(lin_v_given[i]);
    for (int i = 0; i < 30; i++) values.data.push_back(ctp[i]);
    chatter_pub.publish(values);
    ros::spinOnce();
    loop_rate.sleep();
  }
  // myfile.close();
  return 0;
}

