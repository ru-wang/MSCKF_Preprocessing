#!/usr/bin/python2
# -*- coding:utf-8 -*- 
import kittiimu as kimu

import os
import sys

def read_from(oxts_path, velocity = False, verbose = False):
  if oxts_path[-1] != '/':
    oxts_path = oxts_path + '/'

  # timestamps file
  filename_timestamps = oxts_path + 'timestamps.txt'
  file_timestamps = open(filename_timestamps, 'r')

  # extract ground truth data
  output_arr = []
  file_id = 0
  while True:
    data_tuple = []

    filename = oxts_path + 'data/' + ('%010i.txt' % file_id)
    file_id = file_id + 1
    if not os.path.isfile(filename):
      break

    # read timestamps
    new_timestamp = file_timestamps.readline()
    real_timestamp = kimu.timestamp_sub(new_timestamp, '1970-01-01 00:00:00')
    data_tuple.append(real_timestamp)

    # read IMU files
    data_file = open(filename, 'r')
    line = data_file.readline().split(" ", 6)
    data_tuple.append(float(line[0]))  # latitude of the oxts-unit (deg)
    data_tuple.append(float(line[1]))  # longitude of the oxts-unit (deg)
    data_tuple.append(float(line[2]))  # altitude of the oxts-unit (m)
    data_tuple.append(float(line[3]))  # roll, 0 = level, positive = left side up, range: -pi...+pi
    data_tuple.append(float(line[4]))  # pitch, 0 = level, positive = front down, range: -pi/2...+pi/2
    data_tuple.append(float(line[5]))  # heading, 0 = east, positive = counter clockwise, range: -pi...+pi
    data_file.close()

    if verbose:
      print data_tuple
    output_arr.append([v for v in data_tuple])

  # close timestamps file
  file_timestamps.close()
  if verbose:
    print ''

  return output_arr

if __name__ == '__main__':
  oxts_path = os.path.expanduser(sys.argv[1])
  out_name = 'gt.txt'
  if len(sys.argv) > 2:
    out_name = sys.argv[2]
  aligned_gyro_accel = read_from(oxts_path, True, True)
  out = open(out_name, 'w')
  for a_tuple in aligned_gyro_accel:
    line = [str(v) for v in a_tuple]
    out.write(' '.join(line) + '\n')
  out.close()
