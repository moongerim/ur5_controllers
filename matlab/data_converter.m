close all;
clear all;
clc
cd /home/robot/workspaces/Big_Data/mpc_log/20220416_212153
file_n = 4792;
iter_AB_eval = 456;
iter_AB_train = 2930;
iter_AB_test = 1;
iter_BA_eval = 1;
iter_BA_train = 1;
iter_BA_test = 1;
init_poses = zeros(file_n-1,6);
goal_poses = zeros(file_n-1,6);

for i=2:file_n
    filename = sprintf('%i.mat',i);
    load(filename);
    init_poses(i-1,:)=joint_positions(1,:);
    goal_poses(i-1,:)=from_high_controller(2,19:24);
    e = length(joint_positions);
    data_to_A = zeros(e,54);
    data_to_B = zeros(e,54);
    if from_high_controller(2,19)==3.0
        data_to_A(:,1:6) = joint_positions(:,1:6);
        data_to_A(:,7:48) = human_poses(:,1:42);
        data_to_A(:,49:54) = low_controller(:,1:6);
        T = array2table(data_to_A);
        if file_n(1)<151
            new_filename = sprintf('data_to_B_train_%i.csv',iter_AB_train);
            iter_AB_train = iter_AB_train+1;
        end
        if file_n(1)>150 && file_n(1)<181
            new_filename = sprintf('data_to_B_eval_%i.csv',iter_AB_eval);
            iter_AB_eval = iter_AB_eval+1;
        end
        if file_n(1)>180
            new_filename = sprintf('data_to_B_test_%i.csv',iter_AB_test);
            iter_AB_test = iter_AB_test+1;
        end
        writetable(T,new_filename);
    end
    if from_high_controller(2,19)==0.0
        data_to_B(:,1:6) = joint_positions(:,1:6);
        data_to_B(:,7:48) = human_poses(:,1:42);
        data_to_B(:,49:54) = low_controller(:,1:6);
        T = array2table(data_to_B);
        if file_n(1)<151
            new_filename = sprintf('data_to_A_train_%i.csv',iter_BA_train);
            iter_BA_train = iter_BA_train+1;
        end
        if file_n(1)>150 && file_n(1)<181
            new_filename = sprintf('data_to_A_eval_%i.csv',iter_BA_eval);
            iter_BA_eval = iter_BA_eval+1;
        end
        if file_n(1)>180
            new_filename = sprintf('data_to_A_test_%i.csv',iter_BA_test);
            iter_BA_test = iter_BA_test+1;
        end
        writetable(T,new_filename);
    end
end

plot_2f(init_poses,goal_poses,'init_goal.png')

function a = plot_2f(data1, data2,name)
    fig_5 = figure('Name', name);
    subplot(4,2,1);
    grid on;
    hold on;
    plot(data1(:,1),'.');
    plot(data2(:,1));
    title("jp 1")

    subplot(4,2,2);
    grid on;
    hold on;
    plot(data1(:,2),'.');
    plot(data2(:,2));
    title("jp 2 ")

    subplot(4,2,3);
    grid on;
    hold on;
    plot(data1(:,3),'.');
    plot(data2(:,3));
    title("jp 3")

    subplot(4,2,4);
    grid on;
    hold on;
    plot(data1(:,4),'.');
    plot(data2(:,4));
    title("jp 4")

    subplot(4,2,5);
    grid on;
    hold on;
    plot(data1(:,5),'.');
    plot(data2(:,5));
    title("jp 5")
    
    subplot(4,2,6);
    grid on;
    hold on;
    l1 = plot(data1(:,6),'.');
    l2 = plot(data2(:,6));
    title("tp 6")
    
    hL = legend([l1,l2],["init", "goal"]);
    newPosition = [0.6 0.1 0.1 0.1];
    newUnits = 'normalized';
    set(hL,'Position', newPosition,'Units', newUnits);
    saveas(fig_5, name);
end