# MICP MulRan 2


Requires:
- MulRan raw data files
- Mesh-Map
- ROS2 (humble) version of RMCL

## MulRan data

MulRan experiments directly on the raw mulran data, ordered like this:

```console
KAIST01
├── global_pose.csv
└── sensor_data
    ├── data_stamp.csv
    ├── gps.csv
    ├── navtech_top_stamp.csv
    ├── Ouster
    │   ├── 1561000444390857630.bin
    │   ├── ...
    │   └── 1561001266879906087.bin
    ├── ouster_front_stamp.csv
    ├── radar
    │   └── polar
    │       ├── 1561000444438566669.png
    │       ├── ...
    │       └── 1561001266761334791.png
    ├── sick_pointcloud.las
    └── xsens_imu.csv
```


## Mesh Maps

We reconstructed the sick_pointcloud.las of KAIST02 to a mesh and reduced the number of faces using [lvr2](https://github.com/uos/lvr2).
This mesh map can be used for all KAIST sequences.
Caution: In KAIST02 not every region of KAIST01 was mapped. Therefore, in the end of KAIST01 the car leaves the KAIST02 map. In real life, you should take care of having a complete map of your environment using RMCL.


Mesh 1: 
- [Download](https://kos.informatik.uni-osnabrueck.de/micpl/mulran/maps/mesh/kaist02_mesh_v21M_f12M.ply)
- Faces: 12,408,626
- Vertices: 21,723,560

Mesh 2:
- [Download](https://kos.informatik.uni-osnabrueck.de/micpl/mulran/maps/mesh/kaist02_mesh_v6M_f11M.ply)
- Faces: 11,280,391
- Vertices: 6,598,646


## Run

Change all required paths in `launch/`. Compile and run the launch files. If visualization is enabled, sensor data is published on ROS topics. Thus, ROS tools can be used to visualize the results.

