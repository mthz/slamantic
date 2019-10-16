#!/usr/bin/env bash

#bin=/home/schoerghuberm/work/thirdparty/ORB_SLAM2/Examples/RGB-D/rgbd_tum
#voc=/home/schoerghuberm/work/thirdparty/ORB_SLAM2/Vocabulary/ORBvoc.txt
#config=/home/schoerghuberm/work/thirdparty/ORB_SLAM2/Examples/RGB-D/TUM3.yaml
#tum=/data/datasets/tum


#
#ait
#
#$bin $voc $config $tum/rgbd_dataset_freiburg3_walking_halfsphere $tum/rgbd_dataset_freiburg3_walking_halfsphere/associate.txt
#mv CameraTrajectory.txt w_halfsphere_df.txt
#
#$bin $voc $config $tum/rgbd_dataset_freiburg3_walking_xyz $tum/rgbd_dataset_freiburg3_walking_xyz/associate.txt
#mv CameraTrajectory.txt w_halfsphere_df.txt
#
#$bin $voc $config $tum/rgbd_dataset_freiburg3_walking_rpy $tum/rgbd_dataset_freiburg3_walking_rpy/associate.txt
#mv CameraTrajectory.txt w_halfsphere_df.txt
#
#$bin $voc $config $tum/rgbd_dataset_freiburg3_walking_static $tum/rgbd_dataset_freiburg3_walking_static/associate.txt
#mv CameraTrajectory.txt w_halfsphere_df.txt
#
#$bin $voc $config $tum/rgbd_dataset_freiburg3_sitting_halfsphere $tum/rgbd_dataset_freiburg3_sitting_halfsphere/associate.txt
#mv CameraTrajectory.txt w_halfsphere_df.txt
#
#$bin $voc $config $tum/rgbd_dataset_freiburg3_sitting_xyz $tum/rgbd_dataset_freiburg3_sitting_xyz/associate.txt
#mv CameraTrajectory.txt w_halfsphere_df.txt
#

#cd /data/datasets/tum/rgbd_dataset_freiburg3_walking_halfsphere
#python2.7 /data/datasets/tum/rgbd_benchmark_tools/associate.py rgb.txt depth.txt > associations.txt
#cd /data/datasets/tum/rgbd_dataset_freiburg3_walking_xyz
#python2.7 /data/datasets/tum/rgbd_benchmark_tools/associate.py rgb.txt depth.txt > associations.txt
#cd /data/datasets/tum/rgbd_dataset_freiburg3_walking_rpy
#python2.7 /data/datasets/tum/rgbd_benchmark_tools/associate.py rgb.txt depth.txt > associations.txt
#cd /data/datasets/tum/rgbd_dataset_freiburg3_walking_static
#python2.7 /data/datasets/tum/rgbd_benchmark_tools/associate.py rgb.txt depth.txt > associations.txt
#cd /data/datasets/tum/rgbd_dataset_freiburg3_sitting_halfsphere
#python2.7 /data/datasets/tum/rgbd_benchmark_tools/associate.py rgb.txt depth.txt > associations.txt
#cd /data/datasets/tum/rgbd_dataset_freiburg3_sitting_static
#python2.7 /data/datasets/tum/rgbd_benchmark_tools/associate.py rgb.txt depth.txt > associations.txt
cd /data/datasets/tum/rgbd_dataset_freiburg3_sitting_xyz
python2.7 /data/datasets/tum/rgbd_benchmark_tools/associate.py rgb.txt depth.txt > associations.txt