#!/usr/bin/python
# -*- coding:utf-8 -*- 
import os
import sys
import numpy as np

prefix = '/home/soap/Workspace/2011_09_26/'

# Read Calib 
calib_cam_to_cam = open(prefix + 'calib_cam_to_cam.txt', 'r')
calib_imu_to_velo = open(prefix + 'calib_imu_to_velo.txt', 'r')
calib_velo_to_cam = open(prefix + 'calib_velo_to_cam.txt', 'r')


# Calib cam to cam
# TODO

# Calib cam imu to velo
calib_imu_to_velo.readline()
line = calib_imu_to_velo.readline()
R_imu_to_velo = line.split()[1:]
line = calib_imu_to_velo.readline()
t_imu_to_velo = line.split()[1:]

for idx, item in enumerate(R_imu_to_velo):
  R_imu_to_velo[idx] = float(item)
for idx, item in enumerate(t_imu_to_velo):
  t_imu_to_velo[idx] = float(item)

# Calib velo to cam
calib_velo_to_cam.readline()
line = calib_velo_to_cam.readline()
R_velo_to_cam = line.split()[1:]
line = calib_velo_to_cam.readline()
t_velo_to_cam = line.split()[1:]

for idx, item in enumerate(R_velo_to_cam):
  R_velo_to_cam[idx] = float(item)
for idx, item in enumerate(t_velo_to_cam):
  t_velo_to_cam[idx] = float(item)

# Calculate T
R_imu_to_velo = np.matrix([R_imu_to_velo[0:3],
                           R_imu_to_velo[3:6],
                           R_imu_to_velo[6:9]])
t_imu_to_velo = np.matrix([[t_imu_to_velo[0]],
                           [t_imu_to_velo[1]],
                           [t_imu_to_velo[2]]])

R_velo_to_cam = np.matrix([R_velo_to_cam[0:3],
                           R_velo_to_cam[3:6],
                           R_velo_to_cam[6:9]])
t_velo_to_cam = np.matrix([[t_velo_to_cam[0]],
                           [t_velo_to_cam[1]],
                           [t_velo_to_cam[2]]])

# R_imu_to_img = [ -UnitY(), -UnitX(), -UnitZ() ];
R_imu_to_img = R_velo_to_cam * R_imu_to_velo

# p_img_in_imu = [ 0.0065, 0.0638, 0.0000 ];
t_imu_to_img = R_velo_to_cam * t_imu_to_velo + t_velo_to_cam

#T_imu_to_img = np.matrix([[R_imu_to_img.item((0, 0)), R_imu_to_img.item((0, 1)), R_imu_to_img.item((0, 2)), t_imu_to_img.item(0)],
#                          [R_imu_to_img.item((1, 0)), R_imu_to_img.item((1, 1)), R_imu_to_img.item((1, 2)), t_imu_to_img.item(1)],
#                          [R_imu_to_img.item((2, 0)), R_imu_to_img.item((2, 1)), R_imu_to_img.item((2, 2)), t_imu_to_img.item(2)],
#                          [0, 0, 0, 1]])

# Output
print 'R_imu_to_img'
for row in range(3):
  for col in range(3):
    if row * col != 4:
      sys.stdout.write(str(R_imu_to_img.item((row, col))) + ' ')
    else:
      sys.stdout.write(str(R_imu_to_img.item((row, col))) + '\n')
print 'p_img_in_imu'
for idx in range(3):
  if idx != 2:
    sys.stdout.write(str(t_imu_to_img.item(idx)) + ' ')
  else:
    sys.stdout.write(str(t_imu_to_img.item(idx)) + '\n')
sys.stdout.flush
