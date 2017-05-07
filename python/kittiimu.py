#!/usr/bin/python2
# -*- coding:utf-8 -*- 
import os
import sys

# timestamp subtraction
def timestamp_sub(timestamp, last_timestamp):
  '''
  Calculate the time interval between `timestamp` and `last_timestamp`.

  Args:
    last_timestamp (string): The last timestamp in the format 'YYYY-MM-DD hh:mm:ss.XXXX', where XXXX is the fractions
    timestamp      (string): The current timestamp in the format  'YYYY-MM-DD hh:mm:ss.XXXX', where XXXX is the fractions

  Return:
    time_interval (float): The time interval between last timestamp and current timestamp
                           in second

  Example:
    t_interval = timestamp_sub('2011-09-26 13:02:25.594360375',
                               '2011-09-26 13:02:25.604340603')

  '''
  CALENDAR = (0, 31,
    31 + 28,
    31 + 28 + 31,
    31 + 28 + 31 + 30,
    31 + 28 + 31 + 30 + 31,
    31 + 28 + 31 + 30 + 31 + 30,
    31 + 28 + 31 + 30 + 31 + 30 + 31,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30)

  CALENDAR_LEAP_YEAR = (0, 31,
    31 + 29,
    31 + 29 + 31,
    31 + 29 + 31 + 30,
    31 + 29 + 31 + 30 + 31,
    31 + 29 + 31 + 30 + 31 + 30,
    31 + 29 + 31 + 30 + 31 + 30 + 31,
    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31,
    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30,
    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31,
    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30)

  interval = 0.0
  if last_timestamp == timestamp:
    return interval

  last_date, last_time = last_timestamp.split()
  curr_date, curr_time = timestamp.split()

  last_time = last_time.split(':')
  curr_time = curr_time.split(':')

  curr_date = curr_date.split('-')
  last_date = last_date.split('-')

##  TODO Currently we only consider the situation
##       where all the timestamps are of the same date.

  interval = float(curr_time[2]) - float(last_time[2])
  interval += (int(curr_time[0]) - int(last_time[0])) * 60 * 60
  interval += (int(curr_time[1]) - int(last_time[1])) * 60

  return interval

def read_from(imu_path, velocity = False, verbose = False):
  if imu_path[-1] != '/':
    imu_path = imu_path + '/'

  # timestamps file
  filename_timestamps = imu_path + 'timestamps.txt'
  file_timestamps = open(filename_timestamps, 'r')

  # extract IMU Data
  output_arr = []
  file_id = 0
  while True:
    data_tuple = []

    filename = imu_path + 'data/' + ('%010i.txt' % file_id)
    file_id = file_id + 1
    if not os.path.isfile(filename):
      break

    # read timestamps
    new_timestamp = file_timestamps.readline()
    real_timestamp = timestamp_sub(new_timestamp, '1970-01-01 00:00:00')
    data_tuple.append(real_timestamp)

    # read IMU files
    data_file = open(filename, 'r')
    line = data_file.readline().split(" ", 20)
    data_tuple.append(float(line[-4]))  # angular rate around x (rad/s)
    data_tuple.append(float(line[-3]))  # angular rate around y (rad/s)
    data_tuple.append(float(line[-2]))  # angular rate around z (rad/s)
    data_tuple.append(float(line[11]))  # acceleration in x (m/s^2)
    data_tuple.append(float(line[12]))  # acceleration in y (m/s^2)
    data_tuple.append(float(line[13]))  # acceleration in z (m/s^2)
    if velocity == True:
      data_tuple.append(float(line[8]))
      data_tuple.append(float(line[9]))
      data_tuple.append(float(line[10]))
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
  imu_path = os.path.expanduser(sys.argv[1])
  out_name = 'imu.txt'
  if len(sys.argv) > 2:
    out_name = sys.argv[2]
  aligned_gyro_accel = read_from(imu_path, True, True)
  out = open(out_name, 'w')
  for a_tuple in aligned_gyro_accel:
    line = [str(v) for v in a_tuple]
    out.write(' '.join(line) + '\n')
  out.close()
