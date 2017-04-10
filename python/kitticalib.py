#!/usr/bin/python2
# -*- coding:utf-8 -*- 
import os
import sys
import numpy as np

prefix = sys.argv[1]
if prefix[-1] != '/':
    prefix = prefix + '/'

# Read Calib 
calib_cam_to_cam = open(prefix + 'calib_cam_to_cam.txt', 'r')
calib_imu_to_velo = open(prefix + 'calib_imu_to_velo.txt', 'r')
calib_velo_to_cam = open(prefix + 'calib_velo_to_cam.txt', 'r')

# Calib cam to cam
# Currently we only use the first grayscale camera
calib_cam_to_cam.readline()  # calib_time
calib_cam_to_cam.readline()  # corner_dist
line = calib_cam_to_cam.readline()  # S_00
S_00 = line.split()[1:]
for idx, item in enumerate(S_00):
  S_00[idx] = float(item)
S_00 = np.matrix(S_00)

line = calib_cam_to_cam.readline()  # K_00
K_00 = line.split()[1:]
for idx, item in enumerate(K_00):
  K_00[idx] = float(item)
K_00 = np.matrix([K_00[0:3], K_00[3:6], K_00[6:9]])
intrinsics = [K_00.item(0, 0), K_00.item(1, 1), K_00.item(0, 2), K_00.item(1, 2), K_00.item(0, 1)]

line = calib_cam_to_cam.readline()  # D_00
D_00 = line.split()[1:]
for idx, item in enumerate(D_00):
  D_00[idx] = float(item)
D_00 = np.matrix(D_00)

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
    if col != 2:
      sys.stdout.write(str(R_imu_to_img.item((row, col))) + ' ')
    else:
      sys.stdout.write(str(R_imu_to_img.item((row, col))) + '\n')
print ''

print 'p_img_in_imu'
for idx in range(3):
  if idx != 2:
    sys.stdout.write(str(t_imu_to_img.item(idx)) + ' ')
  else:
    sys.stdout.write(str(t_imu_to_img.item(idx)) + '\n')
print ''

print 'Image size'
print str(S_00.item(0)) + 'x' + str(S_00.item(1))
print ''

print 'Intrinsics : [fu, fv, cu, cv, w] = ' + str(intrinsics)
for row in range(3):
  for col in range(3):
    if col != 2:
      sys.stdout.write(str(K_00.item((row, col))) + ' ')
    else:
      sys.stdout.write(str(K_00.item((row, col))) + '\n')
sys.stdout.flush

