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
root_dir = '/home/li/DataSet/flight_sim_image_sequence/'
sat_root_dir = root_dir + 'sid/'
road_root_dir = root_dir + 'shp/'
init_pose_dir = root_dir + 'initial_pose/'

acc_error_file = root_dir + 'imu_error_model/' + 'acc.calib'
gyro_error_file = root_dir + 'imu_error_model/' + 'gyro.calib'

result_root_dir = root_dir + 'result/'
max_img_num_in_a_sequence = 200

# initilize cmd
cmd = root_dir + 'flight_sim'

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

	sequence_count = 0
	while sequence_count<sequence_num:
		current_result_dir = result_dir + str(sequence_count+1) + '/'
		if os.path.exists(current_result_dir):
			shutil.rmtree(current_result_dir)
		os.mkdir(current_result_dir)
		os.mkdir(current_result_dir + 'sat/')
		os.mkdir(current_result_dir + 'road/')
		
		# 生成初始位姿
		# v = np.random.uniform(-400, 400, 3)
		# v[2] = np.random.uniform(-50, 50, 1)
		# angle = np.random.uniform(-30/180*3.1415926, 30/180*3.1415926, 3)
		# angle[2] = math.atan2(v[0], v[1])
		# p = np.random.uniform(min_x, max_x, 3)
		# p[1] = np.random.uniform(min_y, max_y, 1)						
		# p[2] = np.random.uniform(700, 1300, 1)								# 高度
		p = [pose_init[sequence_count][0], pose_init[sequence_count][1], np.random.uniform(500, 700, 1)[0]]
		v = [pose_init[sequence_count][2]*500, pose_init[sequence_count][3]*500, np.random.uniform(-50, 50, 1)[0]]
		angle = np.random.uniform(-30/180*3.1415926, 30/180*3.1415926, 3)
		angle[2] = math.atan2(v[0], v[1])

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
		current_cmd = current_cmd + ' ' + str(max_img_num_in_a_sequence)
		print(current_cmd)
		# print('making sequence :' + str(sequence_count))
		os.system(current_cmd)
		sequence_count = sequence_count + 1

# MakeMutiSequence('2/road.shp', 'coq2013_2.sid', '2.txt', '2/')
area_count = 3
while(area_count<=20):
	current_args = (str(area_count)+'/road.shp', str(area_count)+'.sid', str(area_count)+'.txt', str(area_count)+'/')
	t = threading.Thread(target=MakeMutiSequence, args=current_args)
	print('start thread ', area_count, 'with arg: ', current_args)
	t.start()
	area_count = area_count + 1






'''
def MakeMutiSequence(shp_file, sid_file, result_dir, sequence_num, min_x, max_x, min_y, max_y):
	sid_file = sat_root_dir + sid_file
	shp_file = road_root_dir + shp_file
	result_dir = result_root_dir + result_dir

	sequence_count = 0
	while sequence_count<sequence_num:
		current_result_dir = result_dir + str(sequence_count) + '/'
		if os.path.exists(current_result_dir):
			shutil.rmtree(current_result_dir)
		os.mkdir(current_result_dir)
		os.mkdir(current_result_dir + 'sat/')
		os.mkdir(current_result_dir + 'road/')
		
		# 生成初始位姿
		# v = np.random.uniform(-400, 400, 3)
		# v[2] = np.random.uniform(-50, 50, 1)
		# angle = np.random.uniform(-30/180*3.1415926, 30/180*3.1415926, 3)
		# angle[2] = math.atan2(v[0], v[1])
		# p = np.random.uniform(min_x, max_x, 3)
		# p[1] = np.random.uniform(min_y, max_y, 1)						
		# p[2] = np.random.uniform(700, 1300, 1)								# 高度

		
	
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
		print(current_cmd)
		os.system(current_cmd)
		sequence_count = sequence_count + 1

MakeMutiSequence('10/epsg2033-10.shp', '10/coq2013_10.sid', '10/', 50, 301500, 301500+66000, 4.665e+06-12000, 4.665e+06)
'''







# area_dirs = os.listdir(img_root_dir)
# for area_dir in area_dirs:
# 	current_area_dir = img_root_dir + area_dir + '/data/'
# 	current_map_file = img_root_dir + area_dir + '/reference/map'
# 	offset_txt = area_dir.split('_', 2)
# 	offset_x = offset_txt[0]
# 	offset_y = offset_txt[1]
# 	result_file = img_root_dir + area_dir + '/our_localization_result'
# 	print('Dealing with ', current_area_dir, '....')

# 	current_cmd  = cmd + current_map_file + ' ' + offset_x + ' ' + offset_y + ' ' + result_file + ' '
# 	place_dirs = os.listdir(current_area_dir)
# 	for place_dir in place_dirs:
# 		current_cmd = current_cmd + current_area_dir + place_dir + '/cross_pts '
	
# 	os.system(current_cmd)
        
        


