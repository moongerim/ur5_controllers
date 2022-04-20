#!/usr/bin/env python
import numpy as np
import torch
import torch.nn as nn
# import torch.optim as optim
import torch.nn.functional as F
import copy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os
import stream_tee as stream_tee
import __main__ as main
import csv
# from std_msgs.msg import Float64MultiArray
import random
torch.manual_seed(1)

class MyModel(nn.Module):
    def __init__(self, dev, input_dim, output_dim, hidden_dim):
        super().__init__()
        self.dev = dev
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.hidden_dim = hidden_dim
        current_dim = input_dim
        self.layers = nn.ModuleList()
        for hdim in hidden_dim:
            self.layers.append(nn.Linear(current_dim,hdim))
            current_dim = hdim
        self.layers.append(nn.Linear(current_dim,output_dim))
        
    def forward(self, x):
        for layer in self.layers[:-1]:
            x = torch.tanh(layer(x))
        out = self.layers[-1](x)
        return out

def train(epoch, dev, model, x_train, y_train, optimizer, log_interval, loss_function, log_file):
    runLoss = 0
    model.train()
    for b in range(0, len(x_train), n_batch):
        seq_data = np.array(x_train[b:b+n_batch])
        seq_label = np.array(y_train[b:b+n_batch])
        seq_data = torch.tensor([i for i in seq_data], dtype=torch.float32).to(dev)
        seq_label = torch.tensor([i for i in seq_label], dtype=torch.float32).to(dev)
        optimizer.zero_grad()
        y_pred = model(seq_data)
        single_loss = loss_function(y_pred, seq_label)
        runLoss += single_loss.item()
        single_loss.backward()
        optimizer.step()
    runLoss /=len(x_train)
    print ('Train epoch {} loss: {:.6f}'.format(epoch,  runLoss))
    
    return runLoss

def evals(model, x_eval, y_eval, dev, loss_function, log_file):
    total_loss = 0
    model.eval()
    with torch.no_grad():
        for b in range(0, len(x_eval), n_batch):
            seq_data = np.array(x_eval[b:b+n_batch])
            seq_label = np.array(y_eval[b:b+n_batch])
            seq_data = torch.tensor([i for i in seq_data], dtype=torch.float32).to(dev)
            seq_label = torch.tensor([i for i in seq_label], dtype=torch.float32).to(dev)
            y_pred = model(seq_data)
            single_loss = loss_function(y_pred, seq_label)
            total_loss += single_loss.item()
        total_loss /= len(x_eval)
    print('Evaluation Set: Average loss: {:4f}\n'.format(total_loss))
    return total_loss

def test(model, x_test, y_test):
    model.eval()
    predictions=[]
    real_data=[]
    n_batch = 1
    with torch.no_grad():
        for b in range(0, len(x_test), n_batch):
            # print(b)
            seq_data = np.array(x_test[b:b+n_batch])
            seq_label = np.array(y_test[b:b+n_batch])
            real_data.append(seq_label)
            seq_data = torch.tensor([i for i in seq_data], dtype=torch.float32).to(dev)
            predictions.append(model(seq_data))
    return predictions, real_data

def load_data(n_files,datatype,filename):

    full_data = None
    for i in range(1, n_files):
        if datatype=='train':
            raw_data = np.loadtxt('{}_train_{}.csv'.format(filename,i), skiprows = 1, delimiter=',')
        else:
            raw_data = np.loadtxt('{}_eval_{}.csv'.format(filename,i), skiprows = 1, delimiter=',')

        if full_data is None:
            full_data = raw_data
        else:
            full_data = np.concatenate((full_data, raw_data), axis=0)
    return full_data


def visualize(predicted_data, real_data, logfile, x_test,loss1,loss2,directory):
    os.chdir(directory)
    p_data = np.asarray(predicted_data)
    r_data = np.asarray(real_data)
    p_len = len(p_data)
    r_len = len(r_data)
    x1 = np.zeros(p_len)
    x2 = np.zeros(p_len)
    x3 = np.zeros(p_len)
    x4 = np.zeros(p_len)
    x5 = np.zeros(p_len)
    x6 = np.zeros(p_len)
    x7 = np.zeros(p_len)
    x8 = np.zeros(p_len)
    x9 = np.zeros(p_len)
    x10 = np.zeros(p_len)
    x11 = np.zeros(p_len)
    x12 = np.zeros(p_len)
    x13 = np.zeros(p_len)
    x14 = np.zeros(p_len)
    x15 = np.zeros(p_len)
    x16 = np.zeros(p_len)
    x17 = np.zeros(p_len)
    x18 = np.zeros(p_len)
    x19 = np.zeros(p_len)
    x20 = np.zeros(p_len)
    x21 = np.zeros(p_len)
    x22 = np.zeros(p_len)
    x23 = np.zeros(p_len)
    x24 = np.zeros(p_len)
    x25 = np.zeros(p_len)
    x26 = np.zeros(p_len)
    x27 = np.zeros(p_len)
    x28 = np.zeros(p_len)
    x29 = np.zeros(p_len)
    x30 = np.zeros(p_len)
    x31 = np.zeros(p_len)
    x32 = np.zeros(p_len)
    x33 = np.zeros(p_len)
    x34 = np.zeros(p_len)
    x35 = np.zeros(p_len)
    x36 = np.zeros(p_len)
    x37 = np.zeros(p_len)
    x38 = np.zeros(p_len)
    x39 = np.zeros(p_len)
    x40 = np.zeros(p_len)
    x41 = np.zeros(p_len)
    x42 = np.zeros(p_len)
    x43 = np.zeros(p_len)
    x44 = np.zeros(p_len)
    x45 = np.zeros(p_len)
    x46 = np.zeros(p_len)
    x47 = np.zeros(p_len)
    x48 = np.zeros(p_len)
    vp1 = np.zeros(p_len)
    vp2 = np.zeros(p_len)
    vp3 = np.zeros(p_len)
    vp4 = np.zeros(p_len)
    vp5 = np.zeros(p_len)
    vp6 = np.zeros(p_len)
    vr1 = np.zeros(r_len) 
    vr2 = np.zeros(r_len) 
    vr3 = np.zeros(r_len) 
    vr4 = np.zeros(r_len)
    vr5 = np.zeros(r_len) 
    vr6 = np.zeros(r_len)
    for i in range(p_len):
        init = p_data[i]
        init = list(init.cpu().numpy())
        vp1[i] = init[0][0]
        vp2[i] = init[0][1]
        vp3[i] = init[0][2]
        vp4[i] = init[0][3]
        vp5[i] = init[0][4]
        vp6[i] = init[0][5]
        real = r_data[i]
        real = list(real)
        
        vr1[i] = real[0][0]
        vr2[i] = real[0][1]
        vr3[i] = real[0][2]
        vr4[i] = real[0][3]
        vr5[i] = real[0][4]
        vr6[i] = real[0][5]

        x1[i] = x_test[i][0]
        x2[i] = x_test[i][1]
        x3[i] = x_test[i][2]
        x4[i] = x_test[i][3]
        x5[i] = x_test[i][4]
        x6[i] = x_test[i][5]
        x7[i] = x_test[i][6]
        x8[i] = x_test[i][7]
        x9[i] = x_test[i][8]
        x10[i] = x_test[i][9]
        x11[i] = x_test[i][10]
        x12[i] = x_test[i][11]
        x13[i] = x_test[i][12]
        x14[i] = x_test[i][13]
        x15[i] = x_test[i][14]
        x16[i] = x_test[i][15]
        x17[i] = x_test[i][16]
        x18[i] = x_test[i][17]
        x19[i] = x_test[i][18]
        x20[i] = x_test[i][19]
        x21[i] = x_test[i][20]
        x22[i] = x_test[i][21]
        x23[i] = x_test[i][22]
        x24[i] = x_test[i][23]
        x25[i] = x_test[i][24]
        x26[i] = x_test[i][25]
        x27[i] = x_test[i][26]
        x28[i] = x_test[i][27]
        x29[i] = x_test[i][28]
        x30[i] = x_test[i][29]
        x31[i] = x_test[i][30]
        x32[i] = x_test[i][31]
        x33[i] = x_test[i][32]
        x34[i] = x_test[i][33]
        x35[i] = x_test[i][34]
        x36[i] = x_test[i][35]
        x37[i] = x_test[i][36]
        x38[i] = x_test[i][37]
        x39[i] = x_test[i][38]
        x40[i] = x_test[i][39]
        x41[i] = x_test[i][40]
        x42[i] = x_test[i][41]
        x43[i] = x_test[i][42]
        x44[i] = x_test[i][43]
        x45[i] = x_test[i][44]
        x46[i] = x_test[i][45]
        x47[i] = x_test[i][46]
        x48[i] = x_test[i][47]
        
        row_data = [vp1[i],vp2[i],vp3[i],vp4[i],vp5[i],vp6[i],vr1[i],vr2[i],vr3[i], vr4[i], vr5[i], vr6[i], x1[i], x2[i], x3[i], x4[i], x5[i], x6[i], x7[i], x8[i],x9[i], x10[i], x11[i], x12[i], x13[i], x14[i], x15[i], x16[i],x17[i], x18[i], x19[i], x20[i], x21[i], x22[i], x23[i], x24[i],x25[i], x26[i], x27[i], x28[i], x29[i], x30[i], x31[i], x32[i],x33[i], x34[i], x35[i], x36[i], x37[i], x38[i], x39[i], x40[i],x41[i], x42[i], x43[i], x44[i], x45[i], x46[i], x47[i], x48[i]]
        
        with open(logfile,  'a') as fd:
            wr = csv.writer(fd, dialect='excel')
            wr.writerow(row_data)
    a = 6
    fig, axs = plt.subplots(3,2)   
    time = np.arange(0,p_len,1)
    axs[0,0].plot(time, vp1, 'r', label = 'predicted')
    axs[0,0].set_ylabel('q_1_dot')
    axs[0,0].plot(time, vr1, 'k', label = 'real')
    axs[0,0].grid(True)
    axs[1,0].plot(time, vp2, 'r', label = 'predicted')
    axs[1,0].set_ylabel('q_2_dot')
    axs[1,0].plot(time, vr2, 'k', label = 'real')
    axs[1,0].grid(True)
    axs[2,0].plot(time, vp3, 'r', label = 'predicted')
    axs[2,0].set_ylabel('q_3_dot')
    axs[2,0].plot(time, vr3, 'k', label = 'real')
    axs[2,0].grid(True)
    axs[0,1].plot(time, vp4, 'r', label = 'predicted')
    axs[0,1].set_ylabel('q_4_dot')
    axs[0,1].plot(time, vr4, 'k', label = 'real')
    axs[0,1].grid(True)
    axs[1,1].plot(time, vp5, 'r', label = 'predicted')
    axs[1,1].set_ylabel('q_5_dot')
    axs[1,1].plot(time, vr5, 'k', label = 'real')
    axs[1,1].grid(True)
    axs[2,1].plot(time, vp6, 'r', label = 'predicted')
    axs[2,1].set_ylabel('q_6_dot')
    axs[2,1].plot(time, vr6, 'k', label = 'real')
    axs[2,1].grid(True)
    fig.show()
    fig.savefig("test_{}.png".format(run_name))
    plt.close()

    a = 2
    fig2, axs = plt.subplots(2,1)   
    time = np.arange(0,len(loss1),1)
    axs[0].plot(time, loss1)
    axs[0].set_ylabel('train loss')
    axs[1].plot(time, loss2)
    axs[1].set_ylabel('eval loss')
    fig2.show()
    fig2.savefig("loss.png")
    plt.close()

if __name__ == '__main__':
    run_name = stream_tee.generate_timestamp()
    main_dir = '/home/robot/workspaces/Big_Data/nn_train/log/'
    os.chdir(main_dir)
    os.makedirs(run_name)

    train_log = 'train_log.csv'
    eval_log = 'eval_log.csv'
    test_log = 'test_log.csv'
    network_log = 'net_log.csv'

    # Change filename here:
    filename = 'data_to_B'
    if filename =='data_to_A':
        direction = '/home/robot/workspaces/Big_Data/mpc_log/20220415_143814'
        train_files = 4811
        eval_files = 917
        test_file = 'data_to_A_test_5.csv'
    if filename =='data_to_B':
        direction = '/home/robot/workspaces/Big_Data/mpc_log/data_to_B'
        train_files = 4811
        eval_files = 913
        test_file = 'data_to_B_test_5.csv'
    if filename =='data_AB':
        train_files = 3076
        eval_files = 592
        test_file = 'data_AB_test_5.csv'
    if filename =='data_BA':
        train_files = 3076
        eval_files = 594
        test_file = 'data_BA_test_5.csv'
    
    n_batch = 1000
    print(direction)
    os.chdir(direction)

    train_data = load_data(train_files,'train', filename)
    x_train = train_data[:,0:48]
    y_train = train_data[:,48:54]
    print(len(x_train))

    eval_data = load_data(eval_files,'eval', filename)
    random.shuffle(eval_data)
    x_eval = eval_data[:,0:48]
    y_eval = eval_data[:,48:54]

    test_data = np.loadtxt(test_file, skiprows = 1, delimiter=',')
    x_test = test_data[:,0:48]
    y_test = test_data[:,48:54]

    dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    log_dir = main_dir+run_name
    os.chdir(log_dir)
    n = [200,200,200]
    for i in range(len(n)):
        with open(network_log,  'a') as fd:
            wr = csv.writer(fd, dialect='excel')
            wr.writerow([n[i],filename])

    model = MyModel(dev,48,6,n).to(dev)
    epoches = 500
    log_interval = 500
    loss_function = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    lower_loss = 1
    train_losses = []
    eval_losses = []
    # os.chdir('/home/robot/workspaces/ur5_mpc_ursim/src/nn_train/log/20220413_164248')
    # model.cuda()
    # model.load_state_dict(torch.load('model.pth'))

    for epoch in range(epoches):
        log_dir = main_dir+run_name
        os.chdir(log_dir)
        
        loss1 = train(epoch, dev, model, x_train, y_train, optimizer, log_interval, loss_function, train_log)
        with open(train_log,  'a') as fd:
            wr = csv.writer(fd, dialect='excel')
            wr.writerow([loss1])
        train_losses.append(loss1)
        
        loss2 = evals(model, x_eval, y_eval, dev, loss_function, eval_log)
        with open(eval_log,'a') as fd:
            wr = csv.writer(fd, dialect='excel')
            wr.writerow([loss2])
        eval_losses.append(loss2)
        if loss2<lower_loss:
            lower_loss = loss2
            print('\nThe lowest loss is: {:4f}\n '.format(lower_loss))
            log_dir = main_dir+run_name
            os.chdir(log_dir)
            torch.save(model.state_dict(), 'model.pth')
    
    print("\nA testing part")
    log_dir = main_dir+run_name
    os.chdir(log_dir)
    model.cuda()
    model.load_state_dict(torch.load('model.pth'))
    predicted_data, real_data = test(model, x_test, y_test)
    visualize(predicted_data, real_data, test_log, x_test,train_losses,eval_losses,log_dir)

