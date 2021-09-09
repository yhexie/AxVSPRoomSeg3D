# AxVSPRoomSeg3D

AxVSPRoomSeg3D is a three-dimensional room segmentation method.

#### Development

Visual Studio 2015 Update3

OpenVDB 6.0.0

PCL 1.8.1

Boost 1.74

#### Usage

The program AxVSPRoomSeg3DPose.exe is for TLS scans. Each frame of the scanned point cloud is associated with a viewpoint.
The program AxVSPRoomSeg3DTraj.exe is for IMLS point cloud. The dataset consist of point cloud, corresponding trajectory information and timestamps. The datasets were pre-processed by align the coordinate points and trajectory with timestamps.

First, set parameters in config file "configPos.ini", then run the program AxVSPRoomSeg3DPose.exe, the scans should formated as "scan1.pcd, scan2.pcd，..., scan250.pcd"
（example D:\AxVSPRoomSeg3D\data\A1apartment1\scan1.pcd）
or

set parameters in config file "configTraj.ini", then run the program AxVSPRoomSeg3DTraj.exe, the point cloud should named "scan1.txt"

#### Acknowledgments 

The authors give special acknowledgement to Zhu, D. [1] for the implementation of VDB-EDT algorithm. Their open source project is available on https://github.com/zhudelong/VDB-EDT.git.

1. Zhu, D.; Wang, C.; Wang, W.; Garg, R.; Scherer, S.; Meng, M.Q.-H. VDB-EDT: An Efficient Euclidean Distance Transform Algorithm Based on VDB Data Structure. 2021.