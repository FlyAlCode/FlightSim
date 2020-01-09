# !/usr/bin/env python
# -*- coding:utf-8 -*-  

import os
import os.path
import shutil
import cv2
import random
import numpy as np
import time
import math
import threading


# global variables
root_dir = '/media/data/li/Dataset/flight_sim/'
sat_root_dir = root_dir + 'sid/'									# path where the color aerial ortho imageries are saved 
road_root_dir = root_dir + 'shp/'									# path where the road vector map is saved
init_pose_dir = root_dir + 'initial_pose/'							# path where the initial pose files are saved

acc_error_file = root_dir + 'imu_error_model/' + 'acc.calib'		# path where the imu error parameter files are saved
gyro_error_file = root_dir + 'imu_error_model/' + 'gyro.calib'

# result_root_dir = root_dir + 'result-single/'
result_root_dir = root_dir + 'sequence_data/sequences_similarity/'	# path where the generated images are saved
max_img_num_in_a_sequence = 200										# the maximum number of images in one sequence

# initilize cmd
cmd = 'bin/' + 'flight_sim'											

# initilize random generator
np.random.seed(int(time.time())) 
def MakeMutiSequence(shp_file, sid_file,  initial_pose_file, result_dir):
	# 读取初始位置和飞行方向
	initial_pose_file = init_pose_dir + initial_pose_file
	pose_init = np.loadtxt(initial_pose_file)
	sequence_num, tmp = pose_init.shape

	sid_file = sat_root_dir + sid_file
	shp_file = road_root_dir + shp_file
	result_dir = result_root_dir + result_dir
	if os.path.exists(result_dir):
		shutil.rmtree(result_dir)
	os.mkdir(result_dir)

	sequence_count = 0
	while sequence_count<sequence_num:
		current_result_dir = result_dir + str(sequence_count+1) + '/'
		if os.path.exists(current_result_dir):
			shutil.rmtree(current_result_dir)
		os.mkdir(current_result_dir)
		os.mkdir(current_result_dir + 'sat/')
		os.mkdir(current_result_dir + 'road/')
		
		# 生成初始位姿
		p = [pose_init[sequence_count][0], pose_init[sequence_count][1], np.random.uniform(800, 1200, 1)[0]]
		v = [pose_init[sequence_count][2]*40, pose_init[sequence_count][3]*40, np.random.uniform(-1, 1, 1)[0]]
		# 单应变换
		# angle = np.random.uniform(-10.0/180.0*3.1415926, 10.0/180.0*3.1415926, 3)
		# 相似变换
		angle = [0,0,0]
		angle[2] = math.atan2(v[0], v[1])

		# imu 参数
		a_mean = np.random.uniform(-2.0, 2.0, 3)
		a_mean[2] = 0
		# a_mean = [10.0, 10.0, 0]
		# a_mean = [0, 0, 0]

		# 单应变换
		# w_mean = [0, 0, np.random.uniform(-0.2, 0.2, 1)[0]]
		# 相似变换
		w_mean = [0, 0, 0.0]
		
		current_cmd = cmd + ' ' + acc_error_file
		current_cmd = current_cmd + ' '  + gyro_error_file 
		current_cmd = current_cmd + ' '  + sid_file 
		current_cmd = current_cmd + ' '  + shp_file  
		current_cmd = current_cmd + ' '  + current_result_dir  
		current_cmd = current_cmd + ' ' + str(angle[0])      
		current_cmd = current_cmd + ' ' + str(angle[1])
		current_cmd = current_cmd + ' ' + str(angle[2])
		current_cmd = current_cmd + ' ' + str(p[0])
		current_cmd = current_cmd + ' ' + str(p[1])
		current_cmd = current_cmd + ' ' + str(p[2])
		current_cmd = current_cmd + ' ' + str(v[0])
		current_cmd = current_cmd + ' ' + str(v[1])
		current_cmd = current_cmd + ' ' + str(v[2])
		current_cmd = current_cmd + ' ' + str(a_mean[0])
		current_cmd = current_cmd + ' ' + str(a_mean[1])
		current_cmd = current_cmd + ' ' + str(a_mean[2])
		current_cmd = current_cmd + ' ' + str(w_mean[0])
		current_cmd = current_cmd + ' ' + str(w_mean[1])
		current_cmd = current_cmd + ' ' + str(w_mean[2])
		current_cmd = current_cmd + ' ' + str(max_img_num_in_a_sequence)
		print(current_cmd)
		os.system(current_cmd)
		sequence_count = sequence_count + 1


area_count = 1
while(area_count<=20):
	current_args = (str(area_count)+'/road.shp', str(area_count)+'.sid', str(area_count)+'.txt', str(area_count)+'/')
	t = threading.Thread(target=MakeMutiSequence, args=current_args)
	print('start thread ', area_count, 'with arg: ', current_args)
	t.start()
	area_count = area_count + 1

