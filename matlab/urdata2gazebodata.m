clear all
close all
clc
cd /home/robot/workspaces/ur5_mpc_ursim/mpc_log/20220414_133924
% cd /home/robot/workspaces/ur5_mpc_ursim/src/nn_train/test_log/20220414_112006
load('2000.mat');
temp(:,1:6) = joint_positions;
temp(:,7:48) = human_poses;