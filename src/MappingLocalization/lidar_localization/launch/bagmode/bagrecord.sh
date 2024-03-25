#!/bin/bash
rosbag record /velodyne_points /imudata_deg/rpy_deg /imudata_deg/angular_vel_deg /imu_correct /imu/mag /imu/is_autocalibration_active /imu/data /gps_base/GPS_fix /gps_base/GPS_Base &
gnome-terminal -e "bash -c 'rostopic list; exec bash'" & 
gnome-terminal -- rostopic echo /gps_base/GPS_fix &
gnome-terminal -- rostopic echo /imu/data