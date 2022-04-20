close all;
clear all;
clc
cd /home/robot/workspaces/Big_Data/mpc_log/20220414_133924
load('2000.mat');
MPC_jp = joint_positions(:,1:6);
MPC_jv = low_controller(:,1:6);
MPC_h  = human_poses(:,1:42);
for i=10:length(MPC_jp)
    if MPC_h(i,1)~=0
        s_1 = i;
        break
    end
end
        
cd /home/robot/workspaces/ur5_mpc_ursim/src/nn_train/test_log/20220418_211851
load('2000.mat');
NN_jp = joint_positions(:,1:6);
NN_jv = actions(:,1:6);
NN_h  = human_poses(:,1:42);
for i=1:length(NN_jp)
    if NN_h(i,1)~=2.5
        s_2 = i;
        break
    end
end
plot_2f(MPC_jp, NN_jp, 'jp.png', 0.05, s_1, s_2)

function a = plot_2f(data1, data2, name, dt, s_1, s_2)
    len = 1500
    fig_5 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(s_1:s_1+len,1));
    plot(data2(s_2:s_2+len,1));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 1")

    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(s_1:s_1+len,2));
    plot(data2(s_2:s_2+len,2));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 2 ")

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(s_1:s_1+len,3));
    plot(data2(s_2:s_2+len,3));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 3")

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(s_1:s_1+len,4));
    plot(data2(s_2:s_2+len,4));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 4")

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(s_1:s_1+len,5));
    plot(data2(s_2:s_2+len,5));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 5")
    
    subplot(4,2,6);
    grid on;
    hold on;
    l1 = plot(data1(s_1:s_1+len,6));
    l2 = plot(data2(s_2:s_2+len,6));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 6")
    
    hL = legend([l1,l2],["MPC", "NN"]);
    newPosition = [0.6 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(hL,'Position', newPosition,'Units', newUnits);
    saveas(fig_5, name);
end