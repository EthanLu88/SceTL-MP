

import numpy as np
import matplotlib.pyplot as plt

import itertools as it
import time
import scipy.io as scio
import itertools as it

import copy

def reward_p1(traj_ego_grid,traj_obj_grid):   #建R
    r = np.zeros((3,30))

    obj_begin = traj_obj_grid[0,1] 
    obj_end = traj_obj_grid[-1,1]

    r[1,obj_begin : obj_end] = -1


    ego_end = traj_ego_grid[-5:, :]  
    r[(ego_end[0, 0]-1):(ego_end[4,0]+2),ego_end[0,1]:ego_end[4,1]]=1


    return r

def add_dif_data(cl_data,row_cl):
    for j in range(1, row_cl):
        dif = (cl_data[j, 0] - cl_data[j - 1, 0])
        xx=int(dif[0])
        if dif == 1:
            continue
        else:
            f_dif = np.linspace(cl_data[j-1, 0], cl_data[j, 0], num=xx, endpoint=False).reshape(xx,1)
            y_dif = np.linspace(cl_data[j-1, 1], cl_data[j, 1], num=xx, endpoint=False).reshape(xx,1)
            x_dif = np.linspace(cl_data[j-1, 2], cl_data[j, 2], num=xx, endpoint=False).reshape(xx,1)
            new_data = np.c_[f_dif, y_dif]
            new_data = np.c_[new_data,x_dif] 
            new_data=new_data[1:,:]
            new_data.reshape(-1,3)
            np.array(new_data)
            c=np.ndim(new_data )
            d=np.ndim(cl_data )
            cl_data = np.r_[cl_data, new_data]
    e=np.ndim(cl_data)
    # cl_data = np.unique(cl_data, axis=0)  
    cl_data = cl_data[np.argsort(cl_data[:, 0])] 
    return cl_data

def s_order(cl_data):
    cl_data[:,1]=(cl_data[:,1])*3+59 
    to_mid=18-(cl_data[0,2])*3
    cl_data[:,2]=(cl_data[:,2])*3+to_mid 
    cl_data = cl_data.astype('float')
    cl_data=np.around(cl_data) 
    s_certain=np.c_[cl_data[:,2],cl_data[:,1]]
    s_certain = s_certain.astype('int')
    return s_certain

def traj_order(traj_ec):


    traj_ec[:, 1] = (traj_ec[:, 1]) * 3 + 59 
    to_mid = 18 - (traj_ec[0, 0]) * 3
    traj_ec[:, 0] = ((traj_ec[:, 0]) * 3 + to_mid)  
    traj_ec = traj_ec.astype('float')
    traj_ec = np.around(traj_ec)  
    traj_ec = traj_ec.astype('int')
    return traj_ec



def cacul_reward(v_sr,rows,model_data):
    v_sr_list = []
    for i in range(rows): 

        # print(i)

        cl_data=model_data[i,42]
        #print(cl_data)
        # print('\n')
        row_cl=cl_data.shape[0]
        # print(row_cl)
        # print('\n')
        cl_data_new=add_dif_data(cl_data,row_cl) 
        #print(cl_data_new)
        #print('\n')
        s_certain=s_order(cl_data_new)


        # motion_list,x,y = pick_line(i)

        sum = 0
        for j in range(len(s_certain)):  
            sum += v_sr[s_certain[j,0],s_certain[j,1]]   # calculate V



        # print(sum)
        v_sr_list.append(sum / len(s_certain))# average reward


    return v_sr_list



def MaxMinNormalization(x):
    Max = np.max(x)

    Min = np.min(x)
    x = (x - Min) / (Max - Min)
    return x




def M_ratio(md):
    if md == 1:
         a = 0

    if md == 2:
        a = 40

    if md ==3:
        a = 20

    return a

def main(md , traj, sr_M , model_data) :  
    # data = scio.loadmat('D:/研零/drivermodel_new.mat')  # 
    #
    # model = "drivermodel" + str(md)
    # model_data = copy.deepcopy(data[model])
    # model_data = model_data[1:, :]  # 
    # model_data_2 = copy.deepcopy(model_data)
    # for ttttt in range(model_data_2.shape[0]):  
    #     linshi = model_data_2[ttttt, 42]
    #     linshi[:, 2] = -linshi[:, 2]
    # # model_data_2[:,42][:,2] = -model_data_2[:,42][:,2]
    # model_data = np.r_[model_data, model_data_2]
    rows = model_data.shape[0]  


    # dir_matrix = 'F:/研零/test/tsinghua_data/' + 'sr_matrix_model' + str(md) + '_old.npy' 
    # M_1 = np.load(dir_matrix)
    # M_2 = np.load("F:/研零/test/sr_M_36_240.npy")
    # M_1 = MaxMinNormalization(M_1)
    # M_2 = MaxMinNormalization(M_2)
    # rmse_driver = []
    #
    # a = M_ratio(md)
    # M = a * M_1 + (100 - a) * M_2
    #
    # sr_M = MaxMinNormalization(M)
    # np.save('E:/研零/test/driver3/8M1_2M2.npy', M)
    rmse_all = 0
    rmse_m = []



    dif_heng = traj[0, 0]
    dif_zong = traj[0, 1]

    traj[:, 0] = traj[:, 0] - dif_heng 
    traj[:, 1] = traj[:, 1] - dif_zong
    traj[:, 6] = traj[:, 6] - dif_heng
    traj[:, 7] = traj[:, 7] - dif_zong
    traj_ego_grid = copy.deepcopy(traj[:, 0:2])
    traj_obj_grid = copy.deepcopy(traj[:, 6:8])
    traj_ego_grid = traj_order(traj_ego_grid) 
    traj_obj_grid = traj_order(traj_obj_grid)

    r = reward_p1(traj_ego_grid, traj_obj_grid) 

    reward_vec = np.array(r.flatten())
    v_sr = (sr_M @ reward_vec).reshape(r.shape)  # V

    # V for each MP
    model_data_t = copy.deepcopy(model_data)
    v_sr_list = cacul_reward(v_sr, rows, model_data_t)

    uind_rep = v_sr_list.index(max(v_sr_list))  # greedy MP
    del traj
    return uind_rep





if __name__ == '__main__':
    main()