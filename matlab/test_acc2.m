clear all;
close all;
clc;
% cd '/home/robot/workspaces/ur5_mpc_ursim/Network_log/20211209_170428'
f = 1;
load('0.mat')
len = length(time_t)
%% Plot
step = 200;
dt = 0.005;
fig_1 = figure('Name', 'Velocities')
subplot(3,2,1);
grid on;
hold on;
plot(real_vel(:,1))
plot(joint_velocities(:,1))
title('jv_{1}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

subplot(3,2,2);
grid on
hold on;
plot(real_vel(:,2))
plot(joint_velocities(:,2))
title('jv_{2}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

subplot(3,2,3);
hold on;
grid on;
plot(real_vel(:,3))
plot(joint_velocities(:,3))
title('jv_{3}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

subplot(3,2,4);
grid on;
hold on;
plot(real_vel(:,4))
plot(joint_velocities(:,4))
title('jv_{4}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

subplot(3,2,5);
grid on;
hold on;
plot(real_vel(:,5))
plot(joint_velocities(:,5))
title('jv_{5}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

subplot(3,2,6);
grid on;
hold on;
l1 = plot(real_vel(:,6));
l2 = plot(joint_velocities(:,6))
title('jv_{6}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

fig_2 = figure('Name', 'test point velocities 1-5')
subplot(5,1,1);
grid on;
hold on;
plot(real_lin_vel(:,1))
plot(given_lin_vel(:,1))
title('TP_{1}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

subplot(5,1,2);
grid on
hold on
plot(real_lin_vel(:,2))
plot(given_lin_vel(:,2))
title('TP_{2}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

subplot(5,1,3);
hold on;
grid on;
plot(real_lin_vel(:,3))
plot(given_lin_vel(:,3))
title('TP_{3}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);

subplot(5,1,4);
grid on;
hold on;
plot(real_lin_vel(:,4))
plot(given_lin_vel(:,4))
title('TP_{4}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);
subplot(5,1,5);
grid on;
hold on;
l1 = plot(real_lin_vel(:,5))
l2 = plot(given_lin_vel(:,5))
title('TP_{5}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);
% Construct a Legend with the data from the sub-plots
hL = legend([l1,l2],["linear vel real", "linear vel given"]);
% Programatically move the Legend
newPosition = [0.8 0.1 0.1 0.1];
newUnits = 'normalized';
set(hL,'Position', newPosition,'Units', newUnits);

fig_6 = figure('Name', 'test point velocities 5-10')
subplot(5,1,1);
hold on;
grid on;
plot(real_lin_vel(:,6))
plot(given_lin_vel(:,6))
title('TP_{6}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);
xlabel('time, seconds')

subplot(5,1,2);
hold on;
grid on;
plot(real_lin_vel(:,7))
plot(given_lin_vel(:,7))
title('TP_{7}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);
xlabel('time, seconds')

subplot(5,1,3);
hold on;
grid on;
plot(real_lin_vel(:,8))
plot(given_lin_vel(:,8))
title('TP_{8}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);
xlabel('time, seconds')

subplot(5,1,4);
hold on;
grid on;
plot(real_lin_vel(:,9))
plot(given_lin_vel(:,9))
title('TP_{9}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);
xlabel('time, seconds')

subplot(5,1,5);
hold on;
grid on;
l1 = plot(real_lin_vel(:,10))
l2 = plot(given_lin_vel(:,10))
title('TP_{10}')
set(gca,'XTick',0:step:step*len);
set(gca,'XTickLabel',0:dt*step:len*step*dt);
xlabel('time, seconds')
% Construct a Legend with the data from the sub-plots
hL = legend([l1,l2],["linear vel real", "linear vel given"]);
% Programatically move the Legend
newPosition = [0.8 0.1 0.1 0.1];
newUnits = 'normalized';
set(hL,'Position', newPosition,'Units', newUnits);


%% Calculation test points
start = 1
v_s = zeros(10,9);
t_s = zeros(10,9);
for iter = 1:50
    % robot starts to move:
    for p = 1:10
        for i=start:len
            if linear_vell_real(i,p)>0 && linear_vell_given(i,p)>0
                n = i;
                break;
            end
        end

        % robot stops:
        for i = n:len
            if linear_vell_given(i,p)==0
                s = i;
                break;
            end
        end 

        reference = linear_vell_given(n,p)
        epsilon = reference*0.02;
        for t = s:-1:n
            if reference-epsilon>linear_vell_real(t,p) || reference+epsilon<linear_vell_real(t,p)
                t_s(p,iter) = t;
                v_s(p,iter) = linear_vell_real(t,p);
                break;
            end
        end
    end
    start = s;
end

%% Calculation joints
v_s = zeros(6,10);
t_s = zeros(6,10);
% p = 2
for p=1:6
    for f = 1:10
        filename = sprintf('%i.mat',f);
        load(filename);
        if joint_n(1)==p
            len = length(time_t);
            for i=1:len
                if real_vel(i,p)==0
                    s = i;
                    break;
                end
            end
            reference = real_vel(1,p);
            epsilon = reference*0.02;
            for t = s:-1:1
                if reference-epsilon>joint_velocities(t,p) || reference+epsilon<joint_velocities(t,p)
                    t_s(p,f) = time_t(t);
                    v_s(p,f) = joint_velocities(t,p)
                    break
                end
            end
        end
    end
end
%% Plot Velocity vs Time to stop
figure_5 = figure('Name','Velocity vs Time')
hold on
plot(t_s(1,:), v_s(1,:), '.');

for i = 1:6
    for k = 1:50
        acc(i,k)=v_s(i,k)/t_s(i,k);
    end
end



