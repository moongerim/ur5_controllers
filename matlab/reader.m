close all;
clear all;
clc
cd /home/robot/workspaces/Big_Data/data_1505

% human close experiment
load('human_close.mat')

% plot SSM limits
plot_7(MPC_ctv(:,1:7), MPC_ctv(:,8:14), 'mpc_limits_hc.png')
plot_7(NN_pred_ctv(:,1:7), NN_pred_ctv(:,8:14), 'nn_pred_limits_hc.png')  %% NN with human prediction
plot_7(NN_ctv(:,1:7), NN_ctv(:,8:14), 'nn_limits_hc.png')  %% simple NN

% plot joint positions and velocities
plot_3f(MPC_jp,NN_pred_jp,NN_jp,'jp_hc.png')
plot_3f(MPC_jv,NN_pred_jv,NN_jv,'jv_hc.png')

% human close experiment
load('human_far_away.mat')

% plot SSM limits
plot_7(MPC_ctv(:,1:7), MPC_ctv(:,8:14), 'mpc_limits_hfa.png')
plot_7(NN_pred_ctv(:,1:7), NN_pred_ctv(:,8:14), 'nn_pred_limits_hfa.png')
plot_7(NN_ctv(:,1:7), NN_ctv(:,8:14), 'nn_limits_hfa.png')

% plot joint positions and velocities
plot_3f(MPC_jp,NN_pred_jp,NN_jp,'jp_hfa.png')
plot_3f(MPC_jv,NN_pred_jv,NN_jv,'jv_hfa.png')

% plot NN learning curves
load('learning_curves.mat')
% 1. Training path from A to B
plot_losses(AtoB_t, AtoB_v,'AtoB.png')
% 2. Training path from B to A
plot_losses(BtoA_t, BtoA_v,'BtoA.png')
% 3. Training path close to B 
plot_losses(B_t, B_v,'B.png')
% 4. Training path close to A 
plot_losses(A_t, A_v,'A.png')

% 5. Training path from A to B using MPC with future human poses
plot_losses(AtoB_p_t, AtoB_p_v,'AtoB_p.png')
% 6. Training path from B to A using MPC with future human poses
plot_losses(BtoA_p_t, BtoA_p_v,'BtoA_p_t.png')
% 7. Training path close to B using MPC with future human poses
plot_losses(B_p_t, B_p_v,'B_p.png')
% 8. Training path close to A using MPC with future human poses
plot_losses(A_p_t, A_p_v,'A_p.png')


function plot_losses(data_1, data_2, name)
    fig_1 = figure('name',name)
    hold on
    plot(data_1)
    plot(data_2)
    legend('train loss', 'validation loss')
    saveas(fig_1, name);
end

function plot_7(data1, data2, name)
    len = length(data1);
    dt = 0.05;
    fig_1 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(:,1),'r');
    plot(data2(:,1),'k');
    title("tp 1")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(:,2),'r');
    plot(data2(:,2),'k');
    title("tp 2 ")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(:,3),'r');
    plot(data2(:,3),'k');
    title("tp 3")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(:,4),'r');
    plot(data2(:,4),'k');
    title("tp 4")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(:,5),'r');
    plot(data2(:,5),'k');
    title("tp 5")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,6);
    grid on;
    hold on;
    plot(data1(:,6),'r');
    plot(data2(:,6),'k');
    title("tp 6")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,7);
    grid on;
    hold on;
    l1 = plot(data1(:,7),'r');
    l2 = plot(data2(:,7),'k');
    title("tp 7")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    hL = legend([l1,l2],["limit", "max vel"]);
    newPosition = [0.6 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(hL,'Position', newPosition,'Units', newUnits);
    saveas(fig_1, name);
end

function plot_3f(data1, data2, data3, name)
    len = length(data1);
    dt = 0.05;
    fig_2 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(:,1));
    plot(data2(:,1));
    plot(data3(:,1));
    title("joint 1")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(:,2));
    plot(data2(:,2));
    plot(data3(:,2));
    title("joint 2 ")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(:,3));
    plot(data2(:,3));
    plot(data3(:,3));
    title("joint 3")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(:,4));
    plot(data2(:,4));
    plot(data3(:,4));
    title("joint 4")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(:,5));
    plot(data2(:,5));
    plot(data3(:,5));
    title("joint 5")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,6);
    grid on;
    hold on;
    l1 = plot(data1(:,6));
    l2 = plot(data2(:,6));
    l3 = plot(data3(:,6));
    title("joint 6")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    hL = legend([l1,l2,l3],["mpc", "nn with prediction", "nn"]);
    newPosition = [0.6 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(hL,'Position', newPosition,'Units', newUnits);
    saveas(fig_2, name);
end




