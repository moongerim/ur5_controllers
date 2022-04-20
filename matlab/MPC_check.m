clear all
close all
clc

cd /home/robot/workspaces/ur5_mpc_vrep/
filename = 'data_low.csv'
low = load(filename);
[len,i] = size(low);

ctp_f = low(:,1:21);
human_sphere = low(:,22:63);
joint_positions = low(:,64:69);
goal_positions =  low(:,70:75);
q_dot_real = low(:,76:81);
low_MPC_solutions =  low(:,82:90);
min_dist = low(:,91:97);
smallest_distance = low(:,98);
linvelscale = low(:,99);
from_high = low(:,100:129);
ctv = low(:,130:150);
lin_vel_limit = low(:,151:157);
kkt_val_low = low_MPC_solutions(:,8);

high_MPC_solutions = from_high(:,1:12);
high_joint_pos = from_high(:,13:18);
high_goal = from_high(:,19:24);
high_ee = from_high(:,25:27);
kkt_val_high = from_high(:,29);
high_max_dist = from_high(:,30);

robot_spheres = [0.15, 0.15, 0.15, 0.08, 0.08, 0.12, 0.1];
sphere_radi = [0.5510,0.6010,0.5010,0.5010,0.5010,0.5010,0.5010,0.5010,0.4510,0.4510,0.4810,0.4810,0.5510,0.6010];
for l = s:len
    min_d = [10000,10000,10000,10000,10000,10000,10000];
    ans_sd = 10000;
    temp = 10000;
    q = joint_positions(l,:);
    tp= test_points(q(1),q(2),q(3),q(4),q(5),q(6));
    for j=1:7
        robot_vec = [tp(j*3-2),tp(j*3-1),tp(j*3)];
        for k = 1:14
            human_vec = [human_sphere(l,k*3-2),human_sphere(l,k*3-1),human_sphere(l,k*3)];
            temp = norm(robot_vec -human_vec);
            temp = temp - robot_spheres(j) - sphere_radi(k);
            if temp<min_d(j)
                min_d(j)=temp;
            end
            if ans_sd>min_d(j)
                ans_sd = min_d(j);
            end
        end
    end
    smallest_d(l) = ans_sd;
end 
figure(1)
hold on
% a = 21
% plot(tp(s:len,a),'r');
% plot(ctp_f(s:len,a),'k');
plot(smallest_distance(s:len),'r');
plot(smallest_d,'k');

function tp=test_points(theta_1,theta_2,theta_3,theta_4,theta_5,theta_6)
    tp = zeros(1,21);
    z_sh = 0.1;
    tp(1) = 0.06*sin(theta_1);
    tp(2) = -0.06*cos(theta_1);
    tp(3) = 0.0894+z_sh;
    tp(4) = (-0.425*cos(theta_1)*cos(theta_2))/2+0.14*sin(theta_1);
    tp(5) = (-0.425*cos(theta_2)*sin(theta_1))/2-0.14*cos(theta_1);
    tp(6) = (0.0894 - 0.425*sin(theta_2))/2+z_sh;
    tp(7) = -0.425*cos(theta_1)*cos(theta_2)+0.11*sin(theta_1);
    tp(8) = -0.425*cos(theta_2)*sin(theta_1)-0.11*cos(theta_1);
    tp(9) = 0.0894 - 0.425*sin(theta_2)+z_sh;
    tp(10) = -0.425*cos(theta_1)*cos(theta_2)+(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/3+0.02*sin(theta_1);
    tp(11) = -0.425*cos(theta_2)*sin(theta_1)+(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/3-0.02*cos(theta_1);
    tp(12) = 0.0894 - 0.425*sin(theta_2)+(0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.0894 - 0.425*sin(theta_2)))/3+z_sh;
    tp(13) = -0.425*cos(theta_1)*cos(theta_2)+2*(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/3+0.02*sin(theta_1);
    tp(14) = -0.425*cos(theta_2)*sin(theta_1)+2*(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/3-0.02*cos(theta_1);
    tp(15) = 0.0894 - 0.425*sin(theta_2)+2*(0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.0894 - 0.425*sin(theta_2)))/3+z_sh;
    tp(16) = -(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000+0.06*sin(theta_1);
    tp(17) = -(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-0.06*cos(theta_1);
    tp(18) = 0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)+z_sh;
    tp(19) = 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.0823*cos(theta_5)*sin(theta_1) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)-0.05*sin(theta_1);
    tp(20) = 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_1)*cos(theta_5) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)+0.05*cos(theta_1);
    tp(21) = 0.09465*sin(theta_2 + theta_3)*sin(theta_4) - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3) - sin(theta_5)*(0.0823*cos(theta_2 + theta_3)*sin(theta_4) + 0.0823*sin(theta_2 + theta_3)*cos(theta_4)) - 0.09465*cos(theta_2 + theta_3)*cos(theta_4) + 0.08945+z_sh;
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
