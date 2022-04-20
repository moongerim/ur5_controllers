clear all;
close all;
clc
cd '/home/robot/workspaces/ur5_mpc_ursim/data_2403'

file_n = 5123;
for i = 1:file_n
    filename = sprintf('data_%i.csv',i);
    low = load(filename);
    file_number(i) = low(1,158);
end