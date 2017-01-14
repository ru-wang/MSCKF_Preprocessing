#!/usr/bin/python2
# -*- coding:utf-8 -*- 

import os
import sys

def WriteGyroCSVFile(merge_gyro_data, out_prefix):
    filename = out_prefix + 'gyroscope.csv'
    with open(filename, 'w+') as gyrofile:
        for t in merge_gyro_data:
            t[-1] = str(long(t[-1]) / 1e9)
            gyrofile.write(','.join(t) + '\n')

def WriteAcceCSVFile(merge_acce_data, out_prefix):
    filename = out_prefix + 'accelerometer.csv'
    with open(filename, 'w+') as accefile:
        for t in merge_acce_data:
            t[-1] = str(long(t[-1]) / 1e9)
            accefile.write(','.join(t) + '\n')

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
if prefix[-1] != '/':
    prefix = prefix + '/'

out_prefix = prefix
if len(sys.argv) > 2:
    out_prefix = sys.argv[2]
    if out_prefix[-1] != '/':
        out_prefix = out_prefix + '/'

merge_gyro_data = []
merge_acce_data = []

# Read all lines
line = ReadNextGyroLine()
while line:
    curr_gyro = line[:]
    merge_gyro_data.append(curr_gyro)
    line = ReadNextGyroLine()

line = ReadNextAcceLine()
while line:
    curr_acce = line[:]
    merge_acce_data.append(curr_acce)
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

last_timestamp = long(merge_gyro_data[0][-1])
for t in merge_gyro_data[1:]:
    timestamp = long(t[-1])
    fps = 1e9 / float(timestamp - last_timestamp)
    print '%ld - %ld = %ld (FPS: %f)' %(timestamp, last_timestamp, timestamp - last_timestamp, fps)
    last_timestamp = timestamp
times_overall = len(merge_gyro_data) - 1
fps_overall = times_overall * 1e9 / float(long(merge_gyro_data[-1][-1]) - long(merge_gyro_data[0][-1]))
print 'FPS Overall: ' + str(fps_overall)

last_timestamp = long(merge_acce_data[0][-1])
for t in merge_acce_data[1:]:
    timestamp = long(t[-1])
    fps = 1e9 / float(timestamp - last_timestamp)
    print '%ld - %ld = %ld (FPS: %f)' %(timestamp, last_timestamp, timestamp - last_timestamp, fps)
    last_timestamp = timestamp
times_overall = len(merge_acce_data) - 1
fps_overall = times_overall * 1e9 / float(long(merge_acce_data[-1][-1]) - long(merge_acce_data[0][-1]))
print 'FPS Overall: ' + str(fps_overall)

# Write to files
WriteGyroCSVFile(merge_gyro_data, out_prefix)
WriteAcceCSVFile(merge_acce_data, out_prefix)

print('done')
