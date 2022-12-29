

# ==============================================================================
# -- input  --------------------------------------------------------------------
# ==============================================================================
from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
from math import *
from numpy import *
import os
import random
import re
import sys
import weakref
from collections import deque
from matplotlib.pyplot import *
import scipy.io as scio
import copy

sys.path.append('D:\carla0_9_4\CARLA_0.9.4\PythonAPI\carla-0.9.4-py3.7-win-amd64.egg');
sys.path.append('D:\carla0_9_4\CARLA_0.9.4');
sys.path.append('D:\carla0_9_4\CARLA_0.9.4\PythonAPI');


sys.path.append("D:\py_workspace\carla_demo\choose_jiyuan.py")

try:
    import choose_jiyuan
except:
    raise




try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# find carla module
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import glob
import os
import sys


try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# add PythonAPI for release mode 
# try:
#     sys.path.append(glob.glob('PythonAPI')[0])
# except IndexError:
#     pass

import carla
from carla import ColorConverter as cc
from agents.navigation.roaming_agent import RoamingAgent
from agents.navigation.basic_agent import BasicAgent

# try:
#     sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass
#
# import carla
from carla import *
# from carla import ColorConverter as cc
# from agents.navigation.roaming_agent import RoamingAgent
# from agents.navigation.basic_agent import BasicAgent
import random
import time
import pandas
import pygame

# ==============================================================================
# -- function ------------------------------------------------------------------
# ==============================================================================
def callback(event) :
    print(event.other_actor.type_id)
    a = event.other_actor.type_id

    if 'model3' in str(event.other_actor.type_id):   # collision or not
        collision_pointer[traj_num] += 1  


    # if 'vehicle' in vehicle_co.type_id :
    #     collision_pointer[traj_num] += 1

    #

def reset(transform_oth,traj_num) :
    global vehicle
    global vehicle_oth
    global colss
    global transform_bp
    global transform

    
    print('Ready to restart')
    vehicle.destroy()
    vehicle_oth.destroy()
    colss.destroy()
    
    time.sleep(1)

    vehicle = world.spawn_actor(bp, transform)
    vehicle_oth = world.spawn_actor(bp_oth, transform_oth)
    colss = world.spawn_actor(bp_colss, transform_bp, attach_to=vehicle)
    colss.listen(callback)

    #vehicle.set_transform(transform)
    #vehicle_oth.set_transform(transform_oth)
    time.sleep(2)






def control_loop(a) :   

    num1 = len(a[0])
    ayaw = [] 
    for j in range (num1) :


        control.throttle = a[0][j]
        control.brake = a[1][j]
        control.steer = a[2][j]
        #control.throttle = 0.6
        #control.brake = 0
        #control.steer = 0
        
        control_oth.throttle = mean(a[0])   
        control_oth.brake = 0
        control_oth.steer = 1.7458463844377548e-05
        
        vehicle.apply_control(control)
        vehicle_oth.apply_control(control_oth)
        time.sleep(dt/2) 
    


# MP select
def search_for_motionlist(uind,md) :
    dir_uind =  'D:/py_workspace/carla_demo/image_output/driver' +str(md) + '/control_follow_pid+stanley' + str(uind) + '.npy' 
    motion_list = np.load(dir_uind)
    return motion_list



# scenario reproduction
def state_initialization(tv) :

    for i2 in range(54):
        x = vehicle.get_transform().location.x
        x_oth = vehicle_oth.get_transform().location.x

        ## condition
        if vehicle.get_velocity().x > tv[0]: 

            break
        
        # v,x,y,yaw = state_req(vehicle.get_velocity(),vehicle.get_transform(),18.1668,9)
        # v_oth,x_oth,y_oth,yaw_oth = state_req(vehicle_oth.get_velocity(),vehicle_oth.get_transform(),18.1668,9)

        control.throttle = thr_list_hv[i2]
        control.brake = bra_list_hv[i2]
        control.steer = str_list_hv[i2]
        
        control_oth.throttle = thr_list_hv[i2]
        control_oth.brake = bra_list_hv[i2]
        control_oth.steer = str_list_hv[i2]

        vehicle.apply_control(control)
        vehicle_oth.apply_control(control_oth)
        time.sleep(dt/2)
        
        #print(target_speed)
        #print(throttle)

    #print('Ego',v,x,y,yaw)
    #print('Front',v_oth,x_oth,y_oth,yaw_oth)
    #print('delta_x = ',x_oth - x)

# ==============================================================================
# ------------------------------------------------------------------------------
# ==============================================================================








# state ini
thr_list_hv = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.9590499997138977, 0.9713000059127808, 0.9829000234603882, 0.9944999814033508, 1.0, 1.0, 1.0, 0.9373499751091003, 0.8431500196456909, 0.8496000170707703, 0.7518500089645386, 0.7554000020027161, 0.6547499895095825, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582, 0.6553999781608582]
bra_list_hv = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
str_list_hv = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.236863580648787e-05, 6.982198101468384e-05, 8.727514068596065e-05, 0.00010472875874256715, 0.0001221823476953432, 0.00013963606033939868, 0.0001570899476064369, 0.00017454380576964468, 0.00019199734379071742, 0.00019199714006390423, 0.00019199705275241286, 0.00019199696544092149, 0.00019199727103114128, 0.00017454393673688173, 0.00013963675883132964, 8.727546082809567e-05, 5.236790093476884e-05, 3.4914137359010056e-05, 6.982126797083765e-05, 0.00015708977298345417, 0.0002443587873131037, 0.0002618128783069551, 0.00024435960222035646, 0.00020945249707438052, 0.00017454562475904822, 0.00013963866513222456, 0.00010473151633050293, 6.982430204516277e-05, 5.2370673074619845e-05, 3.4917029552161694e-05, 3.491705865599215e-05, 3.491712050163187e-05, 3.491717507131398e-05, 3.491725874482654e-05, 3.4917302400572225e-05, 3.491736060823314e-05, 3.4917415177915245e-05, 3.491743700578809e-05, 1.7463757103541866e-05, 1.7463793483329937e-05, 3.491757524898276e-05, 3.491766983643174e-05, 3.491776806185953e-05, 3.49178008036688e-05]

thr_list_ov = [0.6430799961090088, 0.6783599853515625, 0.7175400257110596, 0.7567200064659119, 0.7958999872207642, 0.8350800275802612, 0.8742600083351135, 0.9134399890899658, 0.9526200294494629, 0.8846200108528137, 0.9179199934005737, 0.8433899879455566, 0.8701599836349487, 0.7890999913215637, 0.8093400001525879, 0.721750020980835, 0.7354599833488464, 0.748520016670227, 0.6543999910354614, 0.6615800261497498, 0.6681100130081177, 0.6746399998664856, 0.6811699867248535, 0.6876999735832214, 0.5870500206947327, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083, 0.5877000093460083]
bra_list_ov = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
str_list_ov = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.491466850391589e-05, 5.236812285147607e-05, 6.982171908020973e-05, 6.982048216741532e-05, 8.727460954105482e-05, 0.00010472866415511817, 0.00012218290066812187, 0.0001221822021761909, 0.00012218207120895386, 0.0001221819838974625, 0.00010472741269040853, 0.00010472758003743365, 0.00010472774010850117, 8.727366366656497e-05, 8.727370732231066e-05, 8.727396925678477e-05, 6.981984188314527e-05, 6.982008926570415e-05, 5.236612196313217e-05, 5.236632205196656e-05, 3.491222014417872e-05, 3.4912263799924403e-05, 1.7458101865486242e-05, 1.745812733133789e-05, 1.745814734022133e-05, 1.745813642628491e-05, 1.7458134607295506e-05, 1.745814734022133e-05, 1.7458150978200138e-05, 3.970436424083346e-09, 3.980099361200473e-09, 1.7458174625062384e-05, 1.7458169168094173e-05, 4.0279912738583334e-09, 4.038854584109686e-09, 4.033355871513322e-09, 1.745822373777628e-05, 1.74582346517127e-05, 4.086130989122694e-09, 4.081978310921386e-09, 4.077834070415065e-09, 4.091293526187201e-09, 1.7458312868257053e-05, 4.168776435165e-09, 4.1821248686346735e-09, 1.7458351067034528e-05, 4.227012517787898e-09, 4.240996442916867e-09, 4.254961272209812e-09, 1.7458463844377548e-05]


control = VehicleControl(throttle=0,steer=0,brake =0)
control_oth = VehicleControl(throttle=0,steer=0,brake =0)


dt = 0.1 





# scenario
md=1  # style 1
tang_name = "driver_tang" + str(md)
dir_tang = 'D:/研零/new_lu/huandao/data/'+ "driver_tang_" + str(md) + '.mat'  
data_tang = scio.loadmat(dir_tang)
traj_tang = data_tang[tang_name]

#M
dir_matrix = 'F:/研零/test/driver' + str(md) + '/M_final.npy'  
M = np.load(dir_matrix)

# data input
data = scio.loadmat('D:/研零/drivermodel_new.mat')

model = "drivermodel" + str(md)
model_data = copy.deepcopy(data[model])
model_data = model_data[1:, :] 
model_data_2 = copy.deepcopy(model_data)
for ttttt in range(model_data_2.shape[0]): 
    linshi = model_data_2[ttttt, 42]
    linshi[:, 2] = -linshi[:, 2]
# model_data_2[:,42][:,2] = -model_data_2[:,42][:,2]
model_data_zy = np.r_[model_data, model_data_2]



#坐标变换用全局变量

# ==============================================================================
# -- main --------------------------------------------------------------------
# ==============================================================================
### start service

try:

    #local port 2000
    client = carla.Client(host = '127.0.0.1', port = 2000)
    #
    client.set_timeout(80.0)
    # world
    world = client.get_world()
    # world = client.load_world('Town03')
    # world blue map
    blueprint_library = world.get_blueprint_library()

    # actor

    bp = blueprint_library.find('vehicle.ford.mustang')
    color = random.choice(bp.get_attribute('color').recommended_values)
    bp.set_attribute('color', color)
    transform = Transform(Location(x=-355.724, y=29.8407, z=1.2029), Rotation(pitch=0,yaw=0,roll=0))
    vehicle = world.spawn_actor(bp, transform)

    global traj_num
    traj_num = 0
    collision_pointer = np.zeros(len(traj_tang))
    bp_colss = blueprint_library.find('sensor.other.collision')
    transform_bp = carla.Transform(carla.Location(x=0.8, z=1.7))
    colss = world.spawn_actor(bp_colss, transform_bp, attach_to=vehicle)
    colss.listen(callback)

    # bp_oth = blueprint_library.find('model3')
    bp_oth = blueprint_library.filter("model3")[0]
    transform_oth = Transform(Location(x=-300.724, y=29.8407, z=1.2029), Rotation(pitch=0,yaw=0,roll=0))
    vehicle_oth = world.spawn_actor(bp_oth, transform_oth)



    for traj_num in range(len(traj_tang)):  
        print("tang轨迹：%d"%traj_num)

        traj_tang_en = traj_tang[traj_num, 0]   
        traj_tang_oth = list(traj_tang_en[:,3])  
        traj_tang_oth = np.asarray(traj_tang_oth)
        traj_tang_oth = traj_tang_oth.ravel()
        traj_tang_oth = traj_tang_oth.tolist()

        traj_tang_v = list(traj_tang_en[:,8])
        traj_tang_v = np.asarray(traj_tang_v)
        traj_tang_v = traj_tang_v.ravel()
        traj_tang_v = traj_tang_v.tolist()

        transform_oth = Transform(Location(x=-355.724 + traj_tang_oth[0], y=29.8407, z=1.2029),
                                  Rotation(pitch=0, yaw=0, roll=0))    
        reset(transform_oth,traj_num)

        uind = choose_jiyuan.main(md, traj_tang_en , M , model_data_zy)  
        # uind = 0



        state_initialization(traj_tang_v)


        motion_list = search_for_motionlist(uind,md)
        ayaw_ave = control_loop(motion_list)


        time.sleep(0.5)

    print(collision_pointer)
    fei = np.nonzero(collision_pointer)
    print(fei)
    print(len(traj_tang))


finally:
    vehicle.destroy()
    vehicle_oth.destroy()
    colss.destroy()















