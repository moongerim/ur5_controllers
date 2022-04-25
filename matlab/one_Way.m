close all;
clear all;
clc
i=1;
% MPC
% A to B
% cd /home/robot/workspaces/Big_Data/mpc_log/20220420_145951
% B to A
cd /home/robot/workspaces/Big_Data/mpc_log/20220420_151727

filename = sprintf('%i.mat',i);
load(filename);
jp_1 = joint_positions;
jv_1 = low_controller(:,1:6);
sd_1 = smallest_dist;
% sd_2 = find_sd(joint_positions, human_poses)

% NN with pred
% A to B
% cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220420_171624
% % B to A
cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220420_173843
filename = sprintf('%i.mat',i);
load(filename);
jp_2 = joint_positions;
jv_2 = actions;
sd_2 = find_sd(joint_positions, human_poses)

% NN
% A to B
% cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220425_151225
% % B to A
cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220425_153409
filename = sprintf('%i.mat',i);
load(filename);
jp_3 = joint_positions;
jv_3 = actions;

sd_3 = find_sd(joint_positions, human_poses)
% sd_2 = smallest_dist;

len = length(sd_1);
dt = 0.05;
plot_3f(jp_1,jp_2,jp_3,'jp.png')
plot_3f(jv_1,jv_2,jv_3,'jv.png')
figure_1 = figure('Name', 'SD')
hold on
plot(sd_1);
plot(sd_2);
plot(sd_3);
set(gca,'XTick',0:200:200*len);
set(gca,'XTickLabel',0:dt*200:len*200*dt);
legend("mpc", "nn with prediction", "nn"); 

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
                temp = norm([tp(i,1),tp(i,2),tp(i,3)]-[human_poses(j,k*3-2),human_poses(j,k*3-1),human_poses(j,k*3)]);
                temp = temp - robot_spheres(i) - human_sphers(k);
                if temp<min_distance(i)
                    min_distance(j,i)=temp;
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
