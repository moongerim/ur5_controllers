
% MPC load
clear all
close all
clc
cd /home/robot/workspaces/Big_Data/mpc_log/20220427_112106
total_file_n = 25;
iter_total_file = 0;
for file_number =1:total_file_n
    iter_total_file = iter_total_file+1;
    fileneme = sprintf('%i.mat',file_number);
    load(fileneme);
    high_goal = from_high_controller(:,19:24);
    s_mpc = 1;
    len_f = length(joint_positions);
    t_mpc = len_f-s_mpc;
    cost = cost_calc(joint_positions(s_mpc:len_f,:), high_goal(s_mpc:len_f,:), low_controller(s_mpc:len_f,1:6), human_poses(s_mpc:len_f,:), t_mpc);
    mpc_cost = cost(1,:);
    mpc_jp_cost = cost(2,:);
    mpc_jv_cost = cost(3,:);
    mpc_rh_cost = cost(4,:);

    dt = 0.05;
%     fig_mpc_lost = figure(1)
%     hold on
%     plot(mpc_cost,'r');
%     plot(mpc_jp_cost,'k');
%     plot(mpc_jv_cost,'b');
%     plot(mpc_rh_cost,'g');
%     set(gca,'XTick',0:200:200*t_mpc);
%     set(gca,'XTickLabel',0:dt*200:t_mpc*200*dt);

%     legend("total mpc","jp","jv","rh");
%     saveas(fig_mpc_lost,'mpc_lost.png');

    error(file_number) =0; 
    for i = s_mpc:len_f
        for j = 1:7
            if ctv_linear(i,j)>lin_vel_limit(i,j)+0.01
                error(file_number) = error(file_number)+1;
            end
        end
    end
    time_spent(file_number) = time(len_f)-time(s_mpc);
    total_MPC(file_number) = sum(mpc_cost);
    total_MPC_jp(file_number) = sum(mpc_jp_cost);
    total_MPC_jv(file_number) = sum(mpc_jv_cost);
    total_MPC_rh(file_number) = sum(mpc_rh_cost);

%         plot_f(joint_positions, 'joint_positions.png', dt, s_mpc, len_f);
%         plot_f(low_MPC_solutions, 'jointvels_MPC.png', dt, s_mpc, len_f);

%     range(1,:) = max(low_MPC_solutions);
%     range(2,:) = min(low_MPC_solutions);

end
big_time = max(time_spent)
average_time = sum(time_spent)/iter_total_file
average_total_cost = sum(total_MPC)/iter_total_file
average_jp_cost = sum(total_MPC_jp)/iter_total_file
average_jv_cost = sum(total_MPC_jv)/iter_total_file
average_rh_cost = sum(total_MPC_rh)/iter_total_file
%% NN 
clear all
close all
clc
cd /home/robot/workspaces/Big_Data/nn_train/test_log/20220427_122747
total_file_n = 25
for file_number = 1:total_file_n

filename = sprintf('%i.mat', file_number);
load(filename);

robot_spheres = [0.15, 0.15, 0.15, 0.08, 0.08, 0.12, 0.1];
human_sphers = [0.5510,0.6010,0.5010,0.5010,0.5010,0.5010,0.5010,0.5010,0.4510,0.4510,0.4810,0.4810,0.5510,0.6010];

for j = 1:length(actions)
%     for j = 1
    min_dist(j,:) = [100,100,100,100,100,100,100];
    smallest_dist(j) = 100;
    tp = test_points(joint_positions(j,1),joint_positions(j,2),joint_positions(j,3),joint_positions(j,4),joint_positions(j,5),joint_positions(j,6));
    
    for i=1:7
        for k = 1:14
            temp = norm([tp(i,1),tp(i,2),tp(i,3)]-[human_poses(j,k*3-2),human_poses(j,k*3-1),human_poses(j,k*3)]);
            temp = temp - robot_spheres(i) - human_sphers(k);
            if temp<min_dist(i)
                min_dist(j,i)=temp;
            end
            if smallest_dist(j)>min_dist(j,i)
                smallest_dist(j) = min_dist(j,i);
            end
        end
    end
end  

s = 3;

len = length(actions);
dt = 0.025;

max(nn_time);
[len,l] = size(actions);
e = len;
s_nn = 3;
t_nn = len-s_nn;

cost = cost_calc(joint_positions(s_nn:e,:), goal(s_nn:e,:), actions(s_nn:e,:), human_poses(s_nn:e,:), t_nn);
nn_cost = cost(1,:);
jp_cost = cost(2,:);
jv_cost = cost(3,:);
rh_cost = cost(4,:);
total_NN(file_number) = sum(nn_cost);
total_NN_jp(file_number) = sum(jp_cost);
total_NN_jv(file_number) = sum(jv_cost);
total_NN_rh(file_number) = sum(rh_cost);
time_spent(file_number) = time(t_nn)-time(s_nn);
% figure(3)
% hold on
% plot(nn_cost,'r');
% plot(jp_cost,'k');
% plot(jv_cost,'b');
% plot(rh_cost,'g');
% set(gca,'XTick',0:200:200*len);
% set(gca,'XTickLabel',0:dt*200:len*200*dt);
% legend("total nn","jp","jv","rh");

% range(1,:) = max(actions);
% range(2,:) = min(actions);


% plot_f(joint_positions, 'joint_positions_NN.png', dt, s_nn, e);
% plot_f(actions, 'jointvels_NN.png', dt, s_nn, e);
% fig_sd = figure(4)
% hold on
% plot(smallest_dist);
% set(gca,'XTick',0:200:200*len);
% set(gca,'XTickLabel',0:dt*200:len*200*dt);
% saveas(fig_sd,'sd.png');

end
average_time = sum(time_spent)/total_file_n
average_total_cost = sum(total_NN)/total_file_n
average_jp_cost = sum(total_NN_jp)/total_file_n
average_jv_cost = sum(total_NN_jv)/total_file_n
average_rh_cost = sum(total_NN_rh)/total_file_n
%% cost calculation
function C = cost_calc(jp, goal, q_d, human, l)
    gamma = 3;
    c1 = 10;
    c2 = 1;
    c3 = 500;
    for i = 1:l
        q = jp(i,1:6);
        q_goal = goal(i,1:6);
        q_dot = q_d(i,1:6);
        jp_cost(i) = norm(q-q_goal)*norm(q-q_goal);
        jp_cost(i) = jp_cost(i) * c1;
        jv_cost(i) = sum(q_dot.*q_dot);
        jv_cost(i) = jv_cost(i) * c2;
        Sphere1_X = human(i,1);
        Sphere1_Y = human(i,2);
        Sphere1_Z = human(i,3);
        Sphere2_X = human(i,4);
        Sphere2_Y = human(i,5);
        Sphere2_Z = human(i,6);
        Sphere14_X = human(i,19);
        Sphere14_Y = human(i,20);
        Sphere14_Z = human(i,21);

        human_stick(1) = (Sphere1_X + Sphere2_X + Sphere14_X)/3;
        human_stick(2) = (Sphere1_Y + Sphere2_Y + Sphere14_Y)/3;
        human_stick(3) = (Sphere1_Z + Sphere2_Z + Sphere14_Z)/3;
        ee_current_pose = ee_pose(q(1),q(2),q(3),q(4),q(5),q(6));
        ee_goal_pose = ee_pose(q_goal(1),q_goal(2),q_goal(3),q_goal(4),q_goal(5),q_goal(6));
        goal_dist = norm(ee_current_pose-ee_goal_pose);
        robot_obst_dist = norm(ee_current_pose-human_stick);
        RH_cost(i) = exp(-gamma*(robot_obst_dist/goal_dist));
        RH_cost(i) = RH_cost(i) * c3;
        Cost(i) = jp_cost(i)+jv_cost(i)+RH_cost(i);
    end
    C(1,:) = Cost;
    C(2,:) = jp_cost;
    C(3,:) = jv_cost;
    C(4,:) = RH_cost;
end

function A = ee_pose(q1,q2,q3,q4,q5,q6)
    cq1 = cos(q1);
	sq1 = sin(q1);
	cq2 = cos(q2);
	sq2 = sin(q2);
	cq3 = cos(q3);
	sq3 = sin(q3);
	cq4 = cos(q4);
	sq4 = sin(q4);
	cq5 = cos(q5);
	sq5 = sin(q5);
    cq6 = cos(q6);
	sq6 = sin(q6);
    
    T7_1 = 0.10915*sq1 - 0.425*cq1*cq2 + 0.0823*cq5*sq1 + 0.39225*cq1*sq2*sq3 - 0.0823*cos(q2 + q3 + q4)*cq1*sq5 + 0.09465*cos(q2 + q3)*cq1*sq4 + 0.09465*sin(q2 + q3)*cq1*cq4 - 0.39225*cq1*cq2*cq3-0.05*sq1;
    T7_2 = 0.39225*sq1*sq2*sq3 - 0.0823*cq1*cq5 - 0.425*cq2*sq1 - 0.10915*cq1 - 0.0823*cos(q2 + q3 + q4)*sq1*sq5 + 0.09465*cos(q2 + q3)*sq1*sq4 + 0.09465*sin(q2 + q3)*cq4*sq1 - 0.39225*cq2*cq3*sq1+0.05*cq1;
    T7_3 = 0.1+0.09465*sin(q2 + q3)*sq4 - 0.425*sq2 - 0.39225*sin(q2 + q3) - sq5*(0.0823*cos(q2 + q3)*sq4 + 0.0823*sin(q2 + q3)*cq4) - 0.09465*cos(q2 + q3)*cq4 + 0.08945;
    A = [T7_1,T7_2,T7_3];
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

function a = plot_f(data, name, dt, s, len)
    
    fig_1 = figure('Name', name);
    subplot(3,2,1);
    grid on;
    hold on;
    plot(data(s:len,1));
    % plot(lin_vell_limit_array(s:len,1));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("q 1")

    subplot(3,2,2);
    grid on;
    hold on;
    plot(data(s:len,2));
    % plot(lin_vell_limit_array(s:len,2));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("q 2 ")

    subplot(3,2,3);
    grid on;
    hold on;
    plot(data(s:len,3));
    % plot(lin_vell_limit_array(s:len,3));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("q 3")

    subplot(3,2,4);
    grid on;
    hold on;
    plot(data(s:len,4));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("q 4")

    subplot(3,2,5);
    grid on;
    hold on;
    plot(data(s:len,5));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("q 5")

    subplot(3,2,6);
    grid on;
    hold on;
    l1 = plot(data(s:len,6));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("q 6")
    saveas(fig_1,name)
end

function a = plot_2f(data1, data2, name, dt, s, len)
    
    fig_5 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(s:len,1));
    plot(data2(s:len,1));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 1")

    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(s:len,2));
    plot(data2(s:len,2));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 2 ")

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(s:len,3));
    plot(data2(s:len,3));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 3")

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(s:len,4));
    plot(data2(s:len,4));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 4")

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(s:len,5));
    plot(data2(s:len,5));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 5")
    
    subplot(4,2,6);
    grid on;
    hold on;
    plot(data1(s:len,6));
    plot(data2(s:len,6));
    set(gca,'XTick',0:200:200*len);
    set(gca,'XTickLabel',0:dt*200:len*200*dt);
    title("tp 6")
    
    subplot(4,2,7);
    grid on;
    hold on;
    l1 = plot(data1(s:len,7));
    l2 = plot(data2(s:len,7));
    hL = legend([l1,l2],["lin vel", "limit"]);
    newPosition = [0.6 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(hL,'Position', newPosition,'Units', newUnits);
    name = sprintf('limits_%i.png');
    saveas(fig_5, name);
end