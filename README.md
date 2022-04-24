# Lidar-Mapping-using-LeGO-LOAM-and-SC-LeGO-LOAM

This repository contains code for lightweight and ground-optimized lidar odometry and mapping (LeGO-LOAM) and Scan context LeGO-LOAM system. The system takes in point cloud from a Velodyne VLP-16 Lidar (placed horizontally) and optional IMU data as inputs. It outputs 6D pose estimation in real-time. 
<!--
[![Watch the video](/LeGO-LOAM/launch/demo32x.mp4)](

https://user-images.githubusercontent.com/97980444/164317160-3aa4f9b2-c125-40e7-8017-ca20d84800e4.mp4

)
-->
# LeGO LOAM
<p align='center'>
    <img src="/LeGO-LOAM/launch/demo32x.gif" alt="drawing" width="800"/>
    <p align='left'>
        <img src="/LeGO-LOAM/launch/cam0_32x.gif" alt="drawing" width="200"/>
    </p>
</p>
<p align='center'>
    <img src="/LeGO-LOAM/launch/cam0_32x.gif" alt="drawing" width="800"/>
</p>
# SC LeGO LOAM
<p align='center'>
    <img src="/LeGO-LOAM/launch/demosc32x.gif" alt="drawing" width="800"/>
</p>

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with noetic)
- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)
  ```
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make check (optional, runs unit tests)
    $ make install
# Prerequisites:
    Boost >= 1.43 (Ubuntu: sudo apt-get install libboost-all-dev)
    CMake >= 3.0 (Ubuntu: sudo apt-get install cmake)
    # Add PPA
    sudo add-apt-repository ppa:borglab/gtsam-develop
    sudo apt update  # not necessary since Bionic
# Install:
    sudo apt install libgtsam-dev libgtsam-unstable-dev
# Add PPA
    sudo add-apt-repository ppa:borglab/gtsam-release-4.0
    sudo apt update  # not necessary since Bionic
# Install:
    sudo apt install libgtsam-dev libgtsam-unstable-dev 

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/pagidik/Lidar-Mapping-using-LeGO-LOAM-and-SC-LeGO-LOAM.git
cd ..
catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.

## The system

LeGO-LOAM is specifically optimized for a horizontally placed VLP-16 on a ground vehicle. It assumes there is always a ground plane in the scan. The vehicle we are using is Northeastern's Autonomous Car NUANCE. It has a built-in IMU. 

<p align='center'>
    <img src="/LeGO-LOAM/launch/nuance.jpg" alt="drawing" width="400"/>
</p>

Lidar odometry performs two-step Levenberg Marquardt optimization to get 6D transformation.

<p align='center'>
    <img src="/LeGO-LOAM/launch/odometry.jpg" alt="drawing" width="400"/>
</p>

## New Lidar
The NUANCE car is equipped with three different Lidars. We chose Velodyne VLP-16 as it is calibrated with all the other sensors. Make sure before your collect the data you are using the correct address to collect data. The key thing to adapting the code to a new sensor is making sure the point cloud can be properly projected to a range image and the ground can be correctly detected. For example, VLP-16 has an angular resolution of 0.2&deg; and 2&deg; along with two directions. It has 16 beams. The angle of the bottom beam is -15&deg. Thus, the parameters in "utility.h" are listed below. When you implement a new sensor, make sure that the ground_cloud has enough points for matching. Before you post any issues, please read this.

```
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0;
extern const int groundScanInd = 7;
```
In case you are using Ouster OS1-64, Use the below configuration in the "utility.h" file and change the lidar topic name accordingly. 
Note:  ```//cloudHeader.stamp = ros::Time::now();``` // Ouster lidar users may need to uncomment this line 
```
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1024;
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 33.2/float(N_SCAN-1);
extern const float ang_bottom = 16.6+0.1;
extern const int groundScanInd = 15;
```

**New**: The existing LeGO-LOAM algorithm is tested on kinetic, melodic. We have tested the algorithm on ROS Noetic and modified the code accordingly.

If you are using your lidar with an IMU, make sure your IMU is aligned properly with the lidar. The algorithm uses IMU data to correct the point cloud distortion caused by sensor motion. If the IMU is not aligned, the usage of IMU data will deteriorate the result. 

## Run the package

1. Run the launch file:
```
roslaunch lego_loam run.launch
```
Notes: The parameter "/use_sim_time" is set to "true" for simulation, "false" to real robot usage.

2. Play existing bag files:
```
rosbag play *.bag --clock --topic /ns1/velodyne_points /imu/imu
```
Notes: Though /imu/imu is optional, it can improve estimation accuracy greatly if provided. 


## Loop Closure

The loop-closure method implemented in this package is a naive ICP-based method. It often fails when the odometry drift is too large. For this reason, we have used more advanced loop-closure methods, Scan Context LeGO-LOAM, which features utilizing point cloud descriptor. To run this we need first remove the current lego loam from the catkin_ws and add the SC-LeGO-LOAM in the src folder. Use catkin_make to make the files. This algorithm takes two inputs one is the query feature points and the second is the existing map. The performance of this algorithm can be best visualized if there is atleast one loop closure. 


## Cite *LeGO-LOAM*

```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```
