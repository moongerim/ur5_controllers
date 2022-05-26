close all;
clear all;
clc
cd /home/robot/workspaces/Big_Data/mpc_log/20220424_101739
file_n = 5543;
iter_AB_eval = 1;
iter_AB_train = 1;
iter_AB_test = 1;
iter_BA_eval = 1;
iter_BA_train = 1;
iter_BA_test = 1;
init_poses = zeros(file_n-1,6);
goal_poses = zeros(file_n-1,6);
human_numbers = zeros(1,195);
human_numbers_ind = [1:1:195];
human_numbers_ts = zeros(1,195);
for i=2:file_n
    filename = sprintf('%i.mat',i);
    load(filename);
    init_poses(i-1,:)=joint_positions(1,:);
    goal_poses(i-1,:)=from_high_controller(2,19:24);
    e = length(joint_positions);
    for ff=1:195
        if file_n(1)==human_numbers_ind(ff)
            human_numbers(ff) = human_numbers(ff)+1;
            human_numbers_ts(ff)= human_numbers_ts(ff)+e;
        break
        end
    end
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

human_1 = sum(human_numbers_ts(1:15))
human_2 = sum(human_numbers_ts(16:30))
human_3 = sum(human_numbers_ts(31:45))
human_4 = sum(human_numbers_ts(46:60))
human_5 = sum(human_numbers_ts(61:75))
human_6 = sum(human_numbers_ts(76:90))
human_7 = sum(human_numbers_ts(91:105))
human_8 = sum(human_numbers_ts(106:120))
human_9 = sum(human_numbers_ts(121:135))
human_10 = sum(human_numbers_ts(136:150))
human_11 = sum(human_numbers_ts(151:165))
human_12 = sum(human_numbers_ts(166:180))
human_13 = sum(human_numbers_ts(181:195))

%%
close all;
clear all;
clc
cd /home/robot/workspaces/Big_Data/mpc_log/20220421_105211 
file_n = 7694;
iter_AB_eval = 1;
iter_AB_train = 1;
iter_AB_test = 1;
iter_BA_eval = 1;
iter_BA_train = 1;
iter_BA_test = 1;
init_poses = zeros(file_n-1,6);
goal_poses = zeros(file_n-1,6);
human_numbers = zeros(1,195);
human_numbers_ind = [1:1:195];
human_numbers_ts = zeros(1,195);
for i=2:file_n
    filename = sprintf('%i.mat',i);
    load(filename);
    init_poses(i-1,:)=joint_positions(1,:);
    goal_poses(i-1,:)=from_high_controller(2,19:24);
    
    e = length(joint_positions);
    for ff=1:195
        if file_n(1)==human_numbers_ind(ff)
            human_numbers(ff) = human_numbers(ff)+1;
            human_numbers_ts(ff)= human_numbers_ts(ff)+e;
        break
        end
    end
    
    data_to_A = zeros(e,54);
    data_to_B = zeros(e,54);
    if from_high_controller(2,19)==3.0
        data_to_A(:,1:6) = joint_positions(:,1:6);
        data_to_A(:,7:48) = human_poses(:,1:42);
        data_to_A(:,49:54) = low_controller(:,1:6);
        T = array2table(data_to_A);
        if file_n(1)<151
            new_filename = sprintf('data_AB_train_%i.csv',iter_AB_train);
            iter_AB_train = iter_AB_train+1;
        end
        if file_n(1)>150 && file_n(1)<181
            new_filename = sprintf('data_AB_eval_%i.csv',iter_AB_eval);
            iter_AB_eval = iter_AB_eval+1;
        end
        if file_n(1)>180
            new_filename = sprintf('data_AB_test_%i.csv',iter_AB_test);
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
            new_filename = sprintf('data_BA_train_%i.csv',iter_BA_train);
            iter_BA_train = iter_BA_train+1;
        end
        if file_n(1)>150 && file_n(1)<181
            new_filename = sprintf('data_BA_eval_%i.csv',iter_BA_eval);
            iter_BA_eval = iter_BA_eval+1;
        end
        if file_n(1)>180
            new_filename = sprintf('data_BA_test_%i.csv',iter_BA_test);
            iter_BA_test = iter_BA_test+1;
        end
        writetable(T,new_filename);
    end
end

human_1 = sum(human_numbers_ts(1:15))
human_2 = sum(human_numbers_ts(16:30))
human_3 = sum(human_numbers_ts(31:45))
human_4 = sum(human_numbers_ts(46:60))
human_5 = sum(human_numbers_ts(61:75))
human_6 = sum(human_numbers_ts(76:90))
human_7 = sum(human_numbers_ts(91:105))
human_8 = sum(human_numbers_ts(106:120))
human_9 = sum(human_numbers_ts(121:135))
human_10 = sum(human_numbers_ts(136:150))
human_11 = sum(human_numbers_ts(151:165))
human_12 = sum(human_numbers_ts(166:180))
human_13 = sum(human_numbers_ts(181:195))

