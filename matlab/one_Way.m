close all;
clear all;
clc
i=21;
cd /home/robot/workspaces/Big_Data/mpc_log/20220420_140058
filename = sprintf('%i.mat',i);
load(filename);
human_1 = human_poses;
cd /home/robot/workspaces/Big_Data/mpc_log/20220420_140058
filename = sprintf('%i.mat',i);
load(filename);
human_2 = human_poses;

plot_2f(human_1,human_2,'init_goal.png')

function a = plot_2f(data1, data2,name)
    fig_5 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(:,1));
    plot(data2(:,1));
    title("jp 1")

    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(:,2));
    plot(data2(:,2));
    title("jp 2 ")

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(:,3));
    plot(data2(:,3));
    title("jp 3")

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(:,4));
    plot(data2(:,4));
    title("jp 4")

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(:,5));
    plot(data2(:,5));
    title("jp 5")
    
    subplot(4,2,6);
    grid on;
    hold on;
    l1 = plot(data1(:,6));
    l2 = plot(data2(:,6));
    title("tp 6")
    
    hL = legend([l1,l2],["init", "goal"]);
    newPosition = [0.6 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(hL,'Position', newPosition,'Units', newUnits);
    saveas(fig_5, name);
end