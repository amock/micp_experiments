# micp_hilti

The following instructions explain how MICP-L accuracy was evaluated on Hilti Datasets. All evaluations were done on a CPU only part of MICP-L.

## System Requirements

We tested these scripts on:
- Ubuntu 20.04
- ROS noetic
- AMD Ryzen 7

## Download

Download/Clone this package anywhere in your system. Proceed with next step.

## Dependencies

Put the following dependencies in your ROS-workspace:

### RMCL

- Link: https://github.com/uos/rmcl
- Branch: main

### pointcloud_motion_deskew

Since MICP-L does not interpolate between start and end of a scan we use this package to compensate for that.

- Link: https://github.com/norlab-ulaval/pointcloud_motion_deskew
- Branch: melodic

### imu_filter_madgwick

For IMU as prior odometry estimation:

```bash
user@pc:~$ sudo apt-get install ros-noetic-imu-filter-madgwick
```
### micp_experiments_helper

Some tools for tf-preprocessing of Hilti Bags. Move from "packages" folder to your workspace.

### Optional: Mesh visualization with rmagine_ros and mesh_tools

For visualization of the mesh map in RViz

- Link: https://github.com/uos/rmagine_ros
- Branch: main

## Hilti Bag Files

Download [Hilti](https://hilti-challenge.com/dataset-2021.html) bagfiles
- [`LAB_Survey_1.bag`](https://storage.googleapis.com/hilti_challenge/LAB_Survey_1.bag)
- [`LAB_Survey_2.bag`](https://storage.googleapis.com/hilti_challenge/LAB_Survey_2.bag)
- [`uzh_tracking_area_run2.bag`](https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag)
- [`uzh_tracking_area_run5.bag`](https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run5.bag)

into the root folder of this repository. Extract them with `rosbag decompress`.

## Execution

Go to the root folder of this repository and execute

### LAB Survey 1

```bash
user@pc:~/micp_hilti$ roslaunch LAB_Survey_1_micp_imu.launch
```

### LAB Survey 2

```bash
user@pc:~/micp_hilti$ roslaunch LAB_Survey_2_micp_imu.launch
```


### UZH Tracking Area 2

```bash
user@pc:~/micp_hilti$ roslaunch uzh_tracking_area_run2_micp_imu.launch
```


### UZH Tracking Area 5

```bash
user@pc:~/micp_hilti$ roslaunch uzh_tracking_area_run5_micp_imu.launch
```


micp_hilti_uzh_x2.gif:

![UZH](img/micp_hilti_uzh_x2.gif "Loading 'micp_hilti_uzh_x2.gif' ...")
