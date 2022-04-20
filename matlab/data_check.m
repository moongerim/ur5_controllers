clear all
close all
clc
cd /home/robot/workspaces/ur5_mpc_ursim/data_1004
iter_AB_eval = 1;
iter_AB_train = 1;
iter_AB_test = 1;
iter_BA_eval = 1;
iter_BA_train = 1;
iter_BA_test = 1;
total = 0;
l_total = 0;
files = 9962;
iter_temp = 1;

for i =1:files
    filename = sprintf('data_%i.csv',i);
    if isfile(filename)
        low = load(filename);
        [len,l] = size(low);
        if len>10
            l_total = l_total+len;
            ctp_f = low(:,1:21);
            human_sphere = low(:,22:63);
            joint_positions = low(:,64:69);
            goal_positions =  low(:,70:75);
            q_dot_real = low(:,76:81);

            low_MPC_solutions =  low(:,82:90);
            max_time_low(i) = max(low_MPC_solutions(:,7));
            min_dist = low(:,91:97);
            smallest_distance = low(:,98);
            linvelscale = low(:,99);

            from_high = low(:,100:129);

            ctv = low(:,130:150);
            lin_vel_limit = low(:,151:157);

            kkt_val_low = low_MPC_solutions(:,8);

            high_MPC_solutions = from_high(:,1:12);
            max_time_high(i) = max(from_high(:,28));
            high_joint_pos = from_high(:,13:18);
            high_goal = from_high(:,19:24);
            max_diff = zeros(1,len);
            for s = 1:len
                temp = abs(high_goal(s,:)-joint_positions(s,:));
                max_diff(s) = max(temp);
            end
            high_ee = from_high(:,25:27);
            kkt_val_high = from_high(:,29);
            max_kkt_high(i) = max(kkt_val_high);
            high_max_dist = from_high(:,30);
            file_n = low(1,158);
            file_n_list(i) = file_n;
            dt = 0.05;
            s = 1;
            e = len;
            error=0;
            iter=1;
            for w = s:e
                if  kkt_val_high(w)>0.01

                    total = total+1; error=error+1; TABLE_err(i) = error;

                end
            end
            data_AB = zeros(e,56);
            data_BA = zeros(e,56);
% 
            perc(i) = error/len;
    
            if perc(i)>0
                total=total+1;
            end
        %     
            if high_goal(2,1)==3.0
                data_AB(:,1:6) = joint_positions(:,1:6);
                data_AB(:,7:48) = human_sphere(:,1:42);
                data_AB(:,49:54) = low_MPC_solutions(:,1:6);
                data_AB(:,55) = kkt_val_high(:,1);
                data_AB(:,56) = smallest_distance(:,1);
                T = array2table(data_AB);
%                 if file_n<136 || (file_n>150 && file_n<166)
                if file_n<151
                    new_filename = sprintf('data_AB_train_%i.csv',iter_AB_train);
                    iter_AB_train = iter_AB_train+1;
                end
%                  if file_n>165
                if file_n>150 && file_n<181
                    new_filename = sprintf('data_AB_eval_%i.csv',iter_AB_eval);
                    iter_AB_eval = iter_AB_eval+1;
                 end
%                 if file_n>135 && file_n<151
                if file_n>180
                    new_filename = sprintf('data_AB_test_%i.csv',iter_AB_test);
                    iter_AB_test = iter_AB_test+1;
                end
    
                writetable(T,new_filename);
            end
            if high_goal(2,1)==0.0
                data_BA(:,1:6) = joint_positions(:,1:6);
                data_BA(:,7:48) = human_sphere(:,1:42);
                data_BA(:,49:54) = low_MPC_solutions(:,1:6);
                data_BA(:,55) = kkt_val_high(:,1);
                data_BA(:,56) = smallest_distance(:,1);
                T = array2table(data_BA);
%                 if file_n<136 || (file_n>150 && file_n<166)
                if file_n<151
                    new_filename = sprintf('data_BA_train_%i.csv',iter_BA_train);
                    iter_BA_train = iter_BA_train+1;
                end
%                 if file_n>165
                if file_n>150 && file_n<181
                    new_filename = sprintf('data_BA_eval_%i.csv',iter_BA_eval);
                    iter_BA_eval = iter_BA_eval+1;
                end
%                 if file_n>135 && file_n<151
                if file_n>180
                    new_filename = sprintf('data_BA_test_%i.csv',iter_BA_test);
                    iter_BA_test = iter_BA_test+1;
                end

                writetable(T,new_filename);
            end
%             dt = 0.025;
%             joint_poses=joint_positions;
%             goal = goal_positions;
%             smallest_dist = smallest_distance;
%             fig_2 = figure('Name', 'positions');
%             subplot(3,2,1);
%             grid on;
%             hold on;
%             plot(joint_poses(s:len,1));
%             plot(goal(s:len,1));
%             set(gca,'XTick',0:200:200*len);
%             set(gca,'XTickLabel',0:dt*200:len*200*dt);
%             title("q 1")
%         
%             subplot(3,2,2);
%             grid on;
%             hold on;
%             plot(joint_poses(s:len,2));
%             plot(goal(s:len,2));
%             set(gca,'XTick',0:200:200*len);
%             set(gca,'XTickLabel',0:dt*200:len*200*dt);
%             title("q 2 ")
%         
%             subplot(3,2,3);
%             grid on;
%             hold on;
%             plot(joint_poses(s:len,3));
%             plot(goal(s:len,3));
%             set(gca,'XTick',0:200:200*len);
%             set(gca,'XTickLabel',0:dt*200:len*200*dt);
%             title("q 3")
%         
%             subplot(3,2,4);
%             grid on;
%             hold on;
%             plot(joint_poses(s:len,4));
%             plot(goal(s:len,4));
%             set(gca,'XTick',0:200:200*len);
%             set(gca,'XTickLabel',0:dt*200:len*200*dt);
%             title("q 4")
%         
%             subplot(3,2,5);
%             grid on;
%             hold on;
%             plot(joint_poses(s:len,5));
%             plot(goal(s:len,5));
%             set(gca,'XTick',0:200:200*len);
%             set(gca,'XTickLabel',0:dt*200:len*200*dt);
%             title("q 5")
%         
%             subplot(3,2,6);
%             grid on;
%             hold on;
%             l1 = plot(joint_poses(s:len,6));
%             l2 = plot(goal(s:len,6));
%             set(gca,'XTick',0:200:200*len);
%             set(gca,'XTickLabel',0:dt*200:len*200*dt);
%             title("q 6")
%             % Construct a Legend with the data from the sub-plots
%             hL = legend([l1,l2],["position", "goal"]);
%             % Programatically move the Legend
%             newPosition = [0.45 0.1 0.1 0.1];
%             newUnits = 'normalized';
%             set(hL,'Position', newPosition,'Units', newUnits);
%             name = sprintf('pos_%i.png',i)
%             saveas(fig_2, name);
%             
%             fig_3 = figure('Name','distances')
%             hold on;
%             plot(smallest_dist(s:len))
%             set(gca,'XTick',0:200:200*len);
%             set(gca,'XTickLabel',0:dt*200:len*200*dt);
%             name = sprintf('dist_%i.png',i)
%             saveas(fig_3, name);
        end
    end
end

% P=total/l_total
