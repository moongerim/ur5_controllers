% AtoB_with_pred = 20220418_132547
% AtoB = 20220425_141300
% 
% BtoA_with_pred = 20220416_175504
% BtoA = 20220425_132233
% 
% AtoB_close_with_pred = 20220413_210952
% AtoB_close = 20220425_120455
% 
% BtoA_close_with_pred = 20220413_183838
% BtoA_close = 20220425_104106

clear all
close all
clc
cd /home/robot/workspaces/Big_Data/nn_train/log/

% Training path from A to B
AtoB_t = load('20220425_141300/train_log.csv');
AtoB_v = load('20220425_141300/eval_log.csv');
plot_losses(AtoB_t, AtoB_v,'AtoB.png')

% Training path from B to A
BtoA_t = load('20220425_132233/train_log.csv');
BtoA_v = load('20220425_132233/eval_log.csv');
plot_losses(BtoA_t, BtoA_v,'BtoA.png')

% Training path close to B 
B_t = load('20220425_120455/train_log.csv');
B_v = load('20220425_120455/eval_log.csv');
plot_losses(B_t, B_v,'closetoB.png')

% Training path close to A 
A_t = load('20220425_104106/train_log.csv');
A_v = load('20220425_104106/eval_log.csv');
plot_losses(A_t, A_v,'closetoA.png')

% Training path from A to B using MPC with future human poses
AtoB_p_t = load('20220418_132547/train_log.csv');
AtoB_p_v = load('20220418_132547/eval_log.csv');
plot_losses(AtoB_p_t, AtoB_p_v,'AtoB_pred.png')

% Training path from B to A using MPC with future human poses
BtoA_p_t = load('20220416_175504/train_log.csv');
BtoA_p_v = load('20220416_175504/eval_log.csv');
plot_losses(BtoA_p_t, BtoA_p_v,'BtoA_pred.png')

% Training path close to B using MPC with future human poses
B_p_t = load('20220413_210952/train_log.csv');
B_p_v = load('20220413_210952/eval_log.csv');
plot_losses(B_p_t, B_p_v,'closetoB_pred.png')

% Training path close to A using MPC with future human poses
A_p_t = load('20220518_184535/train_log.csv');
A_p_v = load('20220518_184535/eval_log.csv');
plot_losses(A_p_t, A_p_v,'closetoA_pred.png')

save('learning_curves.mat','AtoB_t','AtoB_v','BtoA_t','BtoA_v','B_t','B_v','A_t','A_v', ...
      'AtoB_p_t','AtoB_p_v','BtoA_p_t','BtoA_p_v','B_p_t','B_p_v','A_p_t','A_p_v') 

function plot_losses(data_1, data_2, name)
    fig_1 = figure('name',name)
    hold on
    plot(data_1)
    plot(data_2)
    legend('train loss', 'validation loss')
    saveas(fig_1, name);
end