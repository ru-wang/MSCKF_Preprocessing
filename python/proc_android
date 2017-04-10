#!/usr/bin/python2
# -*- coding:utf-8 -*- 

import os
import sys

def PrintMergeDataAndQuit(merge_imu_data):
    duplicates = 0
    line_id = 0
    last_t = merge_imu_data[line_id]

    for t in merge_imu_data[1:]:
        line_id += 1
        assert t[-1] != last_t[-1]
        last_t = t
        for v in last_t:
            assert v != None
        print ' '.join(last_t)
    quit()

gyro_file_id = 0
gyro_file = None
def ReadNextGyroLine():
    global gyro_file_id
    global gyro_file
    if not gyro_file:
        filename = prefix + ('G%08i.txt' % gyro_file_id)
        if os.path.isfile(filename):
            gyro_file = open(filename, 'r')
        else:
            return None
    gyro_line = gyro_file.readline()
    if gyro_line == '':
        gyro_file.close()
        gyro_file_id += 1
        filename = prefix + '/' + ('G%08i.txt' % gyro_file_id)
        if os.path.isfile(filename):
            gyro_file = open(filename, 'r')
            gyro_line = gyro_file.readline()
        else:
            return None
    return gyro_line.strip().split(" ")

acce_file_id = 0
acce_file = None
def ReadNextAcceLine():
    global acce_file_id
    global acce_file
    if not acce_file:
        filename = prefix + '/' + ('A%08i.txt' % acce_file_id)
        if os.path.isfile(filename):
            acce_file = open(filename, 'r')
        else:
            return None
    acce_line = acce_file.readline()
    if acce_line == '':
        acce_file.close()
        acce_file_id += 1
        filename = prefix + '/' + ('A%08i.txt' % acce_file_id)
        if os.path.isfile(filename):
            acce_file = open(filename, 'r')
            acce_line = acce_file.readline()
        else:
            return None
    return acce_line.strip().split(" ")

prefix = sys.argv[1]

merge_gyro_data = []
merge_acce_data = []

# Read all lines
line = ReadNextGyroLine()
while line:
    curr_imu = []
    for v in line[:-1]: curr_imu.append(v)
    for i in range(3): curr_imu.append(None)
    curr_imu.append(line[-1])
    merge_gyro_data.append(curr_imu)
    line = ReadNextGyroLine()

line = ReadNextAcceLine()
while line:
    curr_imu = []
    for i in range(3): curr_imu.append(None)
    for v in line[:-1]: curr_imu.append(v)
    curr_imu.append(line[-1])
    merge_acce_data.append(curr_imu)
    line = ReadNextAcceLine()

# Sort by timestamp
merge_gyro_data.sort(key = lambda t: t[-1])
merge_acce_data.sort(key = lambda t: t[-1])

# Remove duplicates
last_t = merge_gyro_data[0]
t_id = 1
while True:
    t = merge_gyro_data[t_id]
    if t[-1] == last_t[-1]:
        del merge_gyro_data[t_id]
    else:
        last_t = merge_gyro_data[t_id]
        t_id += 1
    if t_id == len(merge_gyro_data):
        break

last_t = merge_acce_data[0]
t_id = 1
while True:
    t = merge_acce_data[t_id]
    if t[-1] == last_t[-1]:
        del merge_acce_data[t_id]
    else:
        last_t = merge_acce_data[t_id]
        t_id += 1
    if t_id == len(merge_acce_data):
        break

# Merge all
gyro_id = 0;
acce_id = 0
merge_imu_data = []
while gyro_id < len(merge_gyro_data) and acce_id < len(merge_acce_data):
    gyro = merge_gyro_data[gyro_id]
    acce = merge_acce_data[acce_id]
    if gyro[-1] < acce[-1]:
        merge_imu_data.append([v for v in gyro])
        gyro_id += 1
    elif acce[-1] < gyro[-1]:
        merge_imu_data.append([v for v in acce])
        acce_id += 1
    else:
        imu = []
        for v in gyro[0:3]: imu.append(v)
        for v in acce[3:-1]: imu.append(v)
        imu.append(gyro[-1])
        merge_imu_data.append(imu)
        gyro_id += 1
        acce_id += 1
else:
    if not gyro_id < len(merge_gyro_data):
        while acce_id < len(merge_acce_data):
            merge_imu_data.append([v for v in acce])
            acce_id += 1
    elif not acce_id < len(merge_acce_data):
        while gyro_id < len(merge_gyro_data):
            merge_imu_data.append([v for v in gyro])
            gyro_id += 1

# Fill in the blanks
if merge_imu_data[0][0] == None:
    while merge_imu_data[1][0] == None:
        del merge_imu_data[0]
elif merge_imu_data[0][3] == None:
    while merge_imu_data[1][3] == None:
        del merge_imu_data[0]
for i in range(1, len(merge_imu_data)):
    last_t = merge_imu_data[i - 1]
    t = merge_imu_data[i]
    if t[0] == None:
        t[0] = last_t[0]
        t[1] = last_t[1]
        t[2] = last_t[2]
    elif t[3] == None:
        t[3 + 0] = last_t[3 + 0]
        t[3 + 1] = last_t[3 + 1]
        t[3 + 2] = last_t[3 + 2]

# Remove the first
del merge_imu_data[0]

# Print and quit
PrintMergeDataAndQuit(merge_imu_data)
