close all;
clear all;
clc
i=1;
% MPC
% A to B
cd /home/robot/workspaces/Big_Data/mpc_log/20220427_105926
% B to A
% cd /home/robot/workspaces/Big_Data/mpc_log/20220427_112106
% % 
filename = sprintf('%i.mat',i);
load(filename);
jp_1 = joint_positions;
jv_1 = low_controller(:,1:6);
sd_1 = smallest_dist;
len = length(jp_1);
A = zeros(len,14);
for f=1:len
    tp = test_points(jp_1(f,1),jp_1(f,2),jp_1(f,3),jp_1(f,4),jp_1(f,5),jp_1(f,6));
    tp_vel = test_vel(jp_1(f,1),jp_1(f,2),jp_1(f,3),jp_1(f,4),jp_1(f,5),jp_1(f,6),jv_1(f,1),jv_1(f,2),jv_1(f,3),jv_1(f,4),jv_1(f,5),jv_1(f,6));
    A(f,:) = limit_check(tp, human_poses(f,:), tp_vel);
end
name = sprintf('limit_mpc_%i.png', i);
% plot_2f_7(A(:,1:7), A(:,8:14), name)
% plot_2f_7(lin_vel_limit, ctv_linear, 'limit_mpc_2')
% NN with pred
% A to B
cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220427_114012
% % B to A
% cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220427_115509
filename = sprintf('%i.mat',i);
load(filename);
jp_2 = joint_positions;
jv_2 = actions;
hp_2 = human_poses;
sd_2 = find_sd(jp_2, hp_2);
len = length(jp_2);
A = zeros(len,14);
for f=1:len
    tp = test_points(jp_2(f,1),jp_2(f,2),jp_2(f,3),jp_2(f,4),jp_2(f,5),jp_2(f,6));
    tp_vel = test_vel(jp_2(f,1),jp_2(f,2),jp_2(f,3),jp_2(f,4),jp_2(f,5),jp_2(f,6),jv_2(f,1),jv_2(f,2),jv_2(f,3),jv_2(f,4),jv_2(f,5),jv_2(f,6));
    A(f,:) = limit_check(tp, human_poses(f,:), tp_vel);
end
name = sprintf('limit_nn_pred_%i.png', i);
% plot_2f_7(A(:,1:7), A(:,8:14), name)

% NN
% A to B
cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220427_121232
% % B to A
% cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220427_122747
filename = sprintf('%i.mat',i);
load(filename);
jp_3 = joint_positions;
jv_3 = actions;
hp_3 = human_poses;
sd_3 = find_sd(joint_positions, human_poses);

len = length(jp_3);
A = zeros(len,14);
for f=1:len
    tp = test_points(jp_3(f,1),jp_3(f,2),jp_3(f,3),jp_3(f,4),jp_3(f,5),jp_3(f,6));
    tp_vel = test_vel(jp_3(f,1),jp_3(f,2),jp_3(f,3),jp_3(f,4),jp_3(f,5),jp_3(f,6),jv_3(f,1),jv_3(f,2),jv_3(f,3),jv_3(f,4),jv_3(f,5),jv_3(f,6));
    A(f,:) = limit_check(tp, human_poses(f,:), tp_vel);
end
% name = sprintf('limit_nn_%i.png', i);
% plot_2f_7(A(:,1:7), A(:,8:14), name)


len = length(sd_1);
dt = 0.05;
% name = sprintf('jp_%i.png',i);
% plot_3f(jp_1,jp_2,jp_3,name)
% name = sprintf('jv_%i.png',i);
% plot_3f(jv_1,jv_2,jv_3,name)
name = sprintf('jp_jv_%i.png',i);
plot_3f_2(jp_1, jp_2, jp_3, jv_1, jv_2, jv_3, name)

figure_1 = figure('Name', 'SD')
hold on
plot(sd_1);
plot(sd_2);
plot(sd_3);
set(gca,'XTick',0:200:200*len);
set(gca,'XTickLabel',0:dt*200:len*200*dt);
legend("mpc", "nn with prediction", "nn"); 
name = sprintf('sd_%i.png',i);
saveas(figure_1,name);
function a = plot_2f(data1, data2,name)
    len = length(data1)
    dt = 0.05
    fig_5 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(:,1));
    plot(data2(:,1));
    title("jp 1")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(:,2));
    plot(data2(:,2));
    title("jp 2 ")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(:,3));
    plot(data2(:,3));
    title("jp 3")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(:,4));
    plot(data2(:,4));
    title("jp 4")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(:,5));
    plot(data2(:,5));
    title("jp 5")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,6);
    grid on;
    hold on;
    l1 = plot(data1(:,6));
    l2 = plot(data2(:,6));
    title("tp 6")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    hL = legend([l1,l2],["mpc", "nn"]);
    newPosition = [0.6 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(hL,'Position', newPosition,'Units', newUnits);
    saveas(fig_5, name);
end

function a = plot_2f_7(data1, data2, name)
    len = length(data1)
    dt = 0.05
    fig_5 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(:,1),'r');
    plot(data2(:,1),'k');
    title("jp 1")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(:,2),'r');
    plot(data2(:,2),'k');
    title("jp 2 ")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(:,3),'r');
    plot(data2(:,3),'k');
    title("jp 3")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(:,4),'r');
    plot(data2(:,4),'k');
    title("jp 4")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(:,5),'r');
    plot(data2(:,5),'k');
    title("jp 5")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,6);
    grid on;
    hold on;
    plot(data1(:,6),'r');
    plot(data2(:,6),'k');
    title("jp 6")
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
    saveas(fig_5, name);
end

function a = plot_3f_2(data1, data2, data3, jv_1, jv_2, jv_3, name)
    len = length(data1)
    dt = 0.05
    fig_5 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    yyaxis left
    plot(data1(:,1));
    plot(data2(:,1),'--');
    plot(data3(:,1),':');
    yyaxis right
    plot(jv_1(:,1));
    plot(jv_2(:,1),'--');
    plot(jv_3(:,1),':');
    title("joint 1")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    yyaxis left
    xlabel('time')
    ylabel('jp 1')
    yyaxis right
    ylabel('jv 1')
    
    subplot(4,2,2);
    grid on;
    hold on;
    yyaxis left
    plot(data1(:,2));
    plot(data2(:,2),'--');
    plot(data3(:,2),':');
    yyaxis right
    plot(jv_1(:,2));
    plot(jv_2(:,2),'--');
    plot(jv_3(:,2),':');
    title("joint 2")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    yyaxis left
    xlabel('time')
    ylabel('jp 2')
    yyaxis right
    ylabel('jv 2')
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,3);
    grid on;
    hold on;
    yyaxis left
    plot(data1(:,3));
    plot(data2(:,3),'--');
    plot(data3(:,3),':');
    yyaxis right
    plot(jv_1(:,3));
    plot(jv_2(:,3),'--');
    plot(jv_3(:,3),':');
    title("joint 3")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    yyaxis left
    xlabel('time')
    ylabel('jp 3')
    yyaxis right
    ylabel('jv 3')
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,4);
    grid on;
    hold on;
    yyaxis left
    plot(data1(:,4));
    plot(data2(:,4),'--');
    plot(data3(:,4),':');
    yyaxis right
    plot(jv_1(:,4));
    plot(jv_2(:,4),'--');
    plot(jv_3(:,4),':');
    title("joint 4")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    yyaxis left
    xlabel('time')
    ylabel('jp 4')
    yyaxis right
    ylabel('jv 4')
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,5);
    grid on;
    hold on;
    yyaxis left
    plot(data1(:,5));
    plot(data2(:,5),'--');
    plot(data3(:,5),':');
    yyaxis right
    plot(jv_1(:,5));
    plot(jv_2(:,5),'--');
    plot(jv_3(:,5),':');
    title("joint 5")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    yyaxis left
    xlabel('time')
    ylabel('jp 5')
    yyaxis right
    ylabel('jv 5')
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,6);
    grid on;
    hold on;
    yyaxis left
    l1 = plot(data1(:,6));
    l2 = plot(data2(:,6),'--');
    l3 = plot(data3(:,6),':');
    yyaxis right
    l4 = plot(jv_1(:,6));
    l5 = plot(jv_2(:,6),'--');
    l6 = plot(jv_3(:,6),':');
    title("joint 6")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    yyaxis left
    xlabel('time')
    ylabel('jp 6')
    yyaxis right
    ylabel('jv 6')
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    h = legend([l1,l2,l3,l4,l5,l6],["jp mpc", "jp nn with prediction", "jp nn","jv mpc", "jv nn with prediction", "jv nn"],'NumColumns',2);
    newPosition = [0.5 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(h,'Position', newPosition,'Units', newUnits);
    saveas(fig_5, name);
end

function a = plot_3f(data1, data2, data3, name)
    len = length(data1)
    dt = 0.05
    fig_5 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(:,1));
    plot(data2(:,1));
    plot(data3(:,1));
    title("jp 1")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(:,2));
    plot(data2(:,2));
    plot(data3(:,2));
    title("jp 2 ")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(:,3));
    plot(data2(:,3));
    plot(data3(:,3));
    title("jp 3")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(:,4));
    plot(data2(:,4));
    plot(data3(:,4));
    title("jp 4")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(:,5));
    plot(data2(:,5));
    plot(data3(:,5));
    title("jp 5")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    subplot(4,2,6);
    grid on;
    hold on;
    l1 = plot(data1(:,6));
    l2 = plot(data2(:,6));
    l3 = plot(data3(:,6));
    title("tp 6")
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    
    hL = legend([l1,l2,l3],["mpc", "nn with prediction", "nn"]);
    newPosition = [0.6 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(hL,'Position', newPosition,'Units', newUnits);
    saveas(fig_5, name);
end


function smallest_dist = find_sd(joint_positions, human_poses)
    robot_spheres = [0.15, 0.15, 0.15, 0.08, 0.08, 0.12, 0.1];
    human_sphers = [0.5510,0.6010,0.5010,0.5010,0.5010,0.5010,0.5010,0.5010,0.4510,0.4510,0.4810,0.4810,0.5510,0.6010];
    for j = 1:length(joint_positions)
        min_distance(j,:) = [100,100,100,100,100,100,100];
        smallest_dist(j) = 100;
        tp = test_points(joint_positions(j,1),joint_positions(j,2),joint_positions(j,3),joint_positions(j,4),joint_positions(j,5),joint_positions(j,6));

        for i=1:7
            for k = 1:14
                temp(j) = norm([tp(i,1),tp(i,2),tp(i,3)]-[human_poses(j,k*3-2),human_poses(j,k*3-1),human_poses(j,k*3)]);
                temp(j) = temp(j) - robot_spheres(i) - human_sphers(k);
                if temp(j)<min_distance(j,i)
                    min_distance(j,i)=temp(j);
                end
                if smallest_dist(j)>min_distance(j,i)
                    smallest_dist(j) = min_distance(j,i);
                end
            end
        end
    end  
end


function tp=test_points(theta_1,theta_2,theta_3,theta_4,theta_5,theta_6)
    tp = zeros(7,3);
    z_sh = 0.1;
    tp(1,1) = 0.06*sin(theta_1);
    tp(1,2) = -0.06*cos(theta_1);
    tp(1,3) = 0.0894+z_sh;
    tp(2,1) = (-0.425*cos(theta_1)*cos(theta_2))/2+0.14*sin(theta_1);
    tp(2,2) = (-0.425*cos(theta_2)*sin(theta_1))/2-0.14*cos(theta_1);
    tp(2,3) = (0.0894 - 0.425*sin(theta_2))/2+z_sh;
    tp(3,1) = -0.425*cos(theta_1)*cos(theta_2)+0.11*sin(theta_1);
    tp(3,2) = -0.425*cos(theta_2)*sin(theta_1)-0.11*cos(theta_1);
    tp(3,3) = 0.0894 - 0.425*sin(theta_2)+z_sh;
    tp(4,1) = -0.425*cos(theta_1)*cos(theta_2)+(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/3+0.02*sin(theta_1);
    tp(4,2) = -0.425*cos(theta_2)*sin(theta_1)+(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/3-0.02*cos(theta_1);
    tp(4,3) = 0.0894 - 0.425*sin(theta_2)+(0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.0894 - 0.425*sin(theta_2)))/3+z_sh;
    tp(5,1) = -0.425*cos(theta_1)*cos(theta_2)+2*(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/3+0.02*sin(theta_1);
    tp(5,2) = -0.425*cos(theta_2)*sin(theta_1)+2*(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/3-0.02*cos(theta_1);
    tp(5,3) = 0.0894 - 0.425*sin(theta_2)+2*(0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.0894 - 0.425*sin(theta_2)))/3+z_sh;
    tp(6,1) = -(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000+0.06*sin(theta_1);
    tp(6,2) = -(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-0.06*cos(theta_1);
    tp(6,3) = 0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)+z_sh;
    tp(7,1) = 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.0823*cos(theta_5)*sin(theta_1) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)-0.05*sin(theta_1);
    tp(7,2) = 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_1)*cos(theta_5) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)+0.05*cos(theta_1);
    tp(7,3) = 0.09465*sin(theta_2 + theta_3)*sin(theta_4) - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3) - sin(theta_5)*(0.0823*cos(theta_2 + theta_3)*sin(theta_4) + 0.0823*sin(theta_2 + theta_3)*cos(theta_4)) - 0.09465*cos(theta_2 + theta_3)*cos(theta_4) + 0.08945+z_sh;
end

function array = limit_check(test_point_cposes, human_poses, test_point_vels)
    array = zeros(1,14);
    robot_spheres = [0.15, 0.15, 0.15, 0.08, 0.08, 0.12, 0.1];
    sphere_radi = [0.5510,0.6010,0.5010,0.5010,0.5010,0.5010,0.5010,0.5010,0.4510,0.4510,0.4810,0.4810,0.5510,0.6010];
    min_dist = [1000,1000,1000,1000,1000,1000,1000];
    spheres_dist = [0,0,0,0,0,0,0];
    for j = 0:6
        w = [test_point_cposes(j*3+1),test_point_cposes(j*3+2),test_point_cposes(j*3+3)];
        for k = 0:13
            p = [human_poses(k*3+1),human_poses(k*3+2),human_poses(k*3+3)];
            local_val = norm(p-w);
            if local_val<min_dist(j+1)
                min_dist(j+1) = local_val;
                spheres_dist(j+1) = robot_spheres(j+1)+sphere_radi(k+1);
            end
        end
    end
    max_vell = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    max_linear_vell = 0;
    for k = 0:6
        temp_linear_vell = sqrt(test_point_vels(k*3+1)*test_point_vels(k*3+1)+test_point_vels(k*3+2)*test_point_vels(k*3+2)+test_point_vels(k*3+3)*test_point_vels(k*3+3));
        max_vell(k+1) = temp_linear_vell;
        if max_linear_vell < temp_linear_vell
             max_linear_vell = temp_linear_vell;
        end
    end
    lin_vell_limit_arr=[10, 10, 10, 10, 10, 10, 10];
    lin_vell_scale = 10;
    alpha=[2.79, 1.95, 1, 0.8, 0.65, 0.45, 0.35];
    for i = 0:6
%         sqrt_temp_value = (min_dist(i+1)*min_dist(i+1)-(spheres_dist(i+1)+d_bar))*(spheres_dist(i+1)+d_bar);
        sqrt_temp_value = (min_dist(i+1)+spheres_dist(i+1))*(min_dist(i+1)+spheres_dist(i+1))-spheres_dist(i+1)*spheres_dist(i+1);
        if sqrt_temp_value<0
           lin_vell_limit_arr(i+1) = 0.00000000000000;
        else
            lin_vell_limit_arr(i+1) = alpha(i+1)*sqrt(sqrt_temp_value);
        end
          temp_scale = (lin_vell_limit_arr(i+1)/max_vell(i+1));
        if lin_vell_scale>temp_scale
            lin_vell_scale = temp_scale;
        end
    end
    array(1:7) = lin_vell_limit_arr;
    array(8:14) = max_vell;
end

function A = test_vel(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, u_1,u_2,u_3,u_4,u_5,u_6)
    A = zeros(21,1);
    A(1) =  0.06*u_1*cos(theta_1);
    A(2) =  0.06*u_1*sin(theta_1);
    A(3) =  0;
    A(4) =  u_1*(0.14*cos(theta_1) + 0.2125*cos(theta_2)*sin(theta_1)) + 0.2125*u_2*cos(theta_1)*sin(theta_2);
    A(5) =  u_1*(0.14*sin(theta_1) - 0.2125*cos(theta_1)*cos(theta_2)) + 0.2125*u_2*sin(theta_1)*sin(theta_2);
    A(6) =  -0.2125*u_2*cos(theta_2);
    A(7) = u_1*(0.11*cos(theta_1) + 0.425*cos(theta_2)*sin(theta_1)) + 0.425*u_2*cos(theta_1)*sin(theta_2);
    A(8) = u_1*(0.11*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2)) + 0.425*u_2*sin(theta_1)*sin(theta_2);
    A(9) = -0.425*u_2*cos(theta_2);
    A(10) = 0.02*u_1*cos(theta_1) + 0.13075*u_1*cos(theta_2 + theta_3)*sin(theta_1) + 0.13075*u_2*sin(theta_2 + theta_3)*cos(theta_1) + 0.13075*u_3*sin(theta_2 + theta_3)*cos(theta_1) + 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2);
    A(11) = 0.02*u_1*sin(theta_1) - 0.13075*u_1*cos(theta_2 + theta_3)*cos(theta_1) + 0.13075*u_2*sin(theta_2 + theta_3)*sin(theta_1) + 0.13075*u_3*sin(theta_2 + theta_3)*sin(theta_1) - 0.425*u_1*cos(theta_1)*cos(theta_2) + 0.425*u_2*sin(theta_1)*sin(theta_2);
    A(12) = - 1.0*u_2*(0.13075*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.13075*u_3*cos(theta_2 + theta_3);
    A(13) = 0.02*u_1*cos(theta_1) + 0.2615*u_1*cos(theta_2 + theta_3)*sin(theta_1) + 0.2615*u_2*sin(theta_2 + theta_3)*cos(theta_1) + 0.2615*u_3*sin(theta_2 + theta_3)*cos(theta_1) + 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2);
    A(14) = 0.02*u_1*sin(theta_1) - 0.2615*u_1*cos(theta_2 + theta_3)*cos(theta_1) + 0.2615*u_2*sin(theta_2 + theta_3)*sin(theta_1) + 0.2615*u_3*sin(theta_2 + theta_3)*sin(theta_1) - 0.425*u_1*cos(theta_1)*cos(theta_2) + 0.425*u_2*sin(theta_1)*sin(theta_2);
    A(15) = - 1.0*u_2*(0.2615*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.2615*u_3*cos(theta_2 + theta_3);
    A(16) = u_1*(0.06*cos(theta_1) + 0.00025*sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2))) + 0.00025*u_2*cos(theta_1)*(1569.0*sin(theta_2 + theta_3) + 1700.0*sin(theta_2)) + 0.39225*u_3*sin(theta_2 + theta_3)*cos(theta_1);
    A(17) = u_1*(0.06*sin(theta_1) - 0.00025*cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2))) + 0.00025*u_2*sin(theta_1)*(1569.0*sin(theta_2 + theta_3) + 1700.0*sin(theta_2)) + 0.39225*u_3*sin(theta_2 + theta_3)*sin(theta_1);
    A(18) = - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3);
    A(19) = 0.05915*u_1*cos(theta_1) + 0.0823*u_1*cos(theta_1)*cos(theta_5) + 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2) - 0.0823*u_5*sin(theta_1)*sin(theta_5) + 0.09465*u_2*cos(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) + 0.09465*u_3*cos(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) + 0.09465*u_4*cos(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.09465*u_1*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*u_1*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.09465*u_2*sin(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) - 0.09465*u_3*sin(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) - 0.09465*u_4*sin(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.39225*u_1*cos(theta_2)*cos(theta_3)*sin(theta_1) + 0.39225*u_2*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*u_2*cos(theta_1)*cos(theta_3)*sin(theta_2) + 0.39225*u_3*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*u_3*cos(theta_1)*cos(theta_3)*sin(theta_2) - 0.39225*u_1*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*u_5*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*cos(theta_5) + 0.0823*u_1*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.0823*u_2*sin(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.0823*u_3*sin(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.0823*u_4*sin(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5);
    A(20) = 0.05915*u_1*sin(theta_1) - 0.425*u_1*cos(theta_1)*cos(theta_2) + 0.0823*u_1*cos(theta_5)*sin(theta_1) + 0.0823*u_5*cos(theta_1)*sin(theta_5) + 0.425*u_2*sin(theta_1)*sin(theta_2) + 0.09465*u_1*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*u_1*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) + 0.09465*u_2*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) + 0.09465*u_3*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) + 0.09465*u_4*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.39225*u_1*cos(theta_1)*cos(theta_2)*cos(theta_3) - 0.09465*u_2*sin(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*u_3*sin(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*u_4*sin(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) + 0.39225*u_1*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*u_2*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*u_2*cos(theta_3)*sin(theta_1)*sin(theta_2) + 0.39225*u_3*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*u_3*cos(theta_3)*sin(theta_1)*sin(theta_2) - 0.0823*u_1*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) - 0.0823*u_5*cos(theta_2 + theta_3 + theta_4)*cos(theta_5)*sin(theta_1) + 0.0823*u_2*sin(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.0823*u_3*sin(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.0823*u_4*sin(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5);
    A(21) = u_4*(0.09465*sin(theta_2 + theta_3 + theta_4) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_5)) - 1.0*u_3*(0.39225*cos(theta_2 + theta_3) - 0.09465*sin(theta_2 + theta_3 + theta_4) + 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_5)) - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2) - 0.09465*cos(theta_2 + theta_3)*sin(theta_4) - 0.09465*sin(theta_2 + theta_3)*cos(theta_4) + 0.0823*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_5) - 0.0823*sin(theta_2 + theta_3)*sin(theta_4)*sin(theta_5)) - 0.0823*u_5*sin(theta_2 + theta_3 + theta_4)*cos(theta_5);
end

