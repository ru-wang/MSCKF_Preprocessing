#!/usr/bin/python2
# -*- coding:utf-8 -*- 
import os
import sys
import numpy as np

def read_from(calib_path, verbose = False):
  if calib_path[-1] != '/':
      calib_path = calib_path + '/'

  # Read Calib 
  calib_cam_to_cam = open(calib_path + 'calib_cam_to_cam.txt', 'r')
  calib_imu_to_velo = open(calib_path + 'calib_imu_to_velo.txt', 'r')
  calib_velo_to_cam = open(calib_path + 'calib_velo_to_cam.txt', 'r')

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

  line = calib_cam_to_cam.readline()  # R_00
  R_00 = line.split()[1:]
  for idx, item in enumerate(R_00):
    R_00[idx] = float(item)
  R_00 = np.matrix([R_00[0:3], R_00[3:6], R_00[6:9]])

  line = calib_cam_to_cam.readline()  # T_00
  T_00 = line.split()[1:]
  for idx, item in enumerate(T_00):
    T_00[idx] = float(item)
  T_00 = np.matrix([[T_00[0]],
                    [T_00[1]],
                    [T_00[2]]])

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
  # The rigid body transformation from IMU coordinates to camera coordinates
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

  R_imu_to_img = R_velo_to_cam * R_imu_to_velo
  t_imu_to_img = R_velo_to_cam * t_imu_to_velo + t_velo_to_cam - T_00

  # Camera orientation in IMU frame
  C_imu_to_cam = np.transpose(R_imu_to_img)
  # Camera position in IMU coordinates
  p_cam_in_imu = -C_imu_to_cam * t_imu_to_img

  if verbose:
    np.set_printoptions(precision=8, suppress=True)

    print 'R [VELO <-- IMU]:'
    print R_imu_to_velo
    print ''
    print 't [VELO <-- IMU]:'
    print t_imu_to_velo
    print ''

    print 'R [CAM <-- VELO]:'
    print R_velo_to_cam
    print ''
    print 't [CAM <-- VELO]:'
    print t_velo_to_cam
    print ''

    print 'T_00:'
    print T_00
    print ''

    print 'R [CAM <-- IMU]:'
    print R_imu_to_img
    print ''
    print 't [CAM <-- IMU]:'
    print t_imu_to_img
    print ''

    print 'Camera orientation in IMU frame'
    print C_imu_to_cam
    print ''
    print 'Camera position in IMU coordinates'
    print p_cam_in_imu
    print ''

    print 'K:'
    print intrinsics
    print ''

  return [ C_imu_to_cam,
           p_cam_in_imu,
           np.matrix(intrinsics[0]),
           np.matrix(intrinsics[1]),
           np.matrix(intrinsics[2]),
           np.matrix(intrinsics[3]),
           np.matrix(intrinsics[4]) ]
