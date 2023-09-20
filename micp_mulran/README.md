# micp_mulran

MICP-L usage and evaluation on Mulran datasets. This is a ROS-package and needs to be placed into a ROS-workspace.

<!-- For each Mulran environment (DCC, KAIST, Riverside, Sejong City) we reconstructed one mesh map using lvr2: -->

<!-- |     Sequence       |   Map     |
|:-------------:|:---------------:|
| DCC | TODO  |
| KAIST | `mulran_kaist02_mesh_lvr2.ply` |
| Riverside | TODO |
| Sejong City | TODO | -->

We reconstructed one mesh map using lvr2 from the second KAIST sequence, named `mulran_kaist02_mesh_lvr2.ply`.


## Results as Bag-Files

We added the resulting localization as tf-frame into newly recorded Bag-Files.
The Bag-Files are available for internals at Rab `datasets/mulran/micp_mulran`

## Reproduction of the results

In this section, we give an overview of how the resulting Bag-Files can be reproduced.

We used a desktop pc with the following specifications:
- Ubuntu 20, ROS noetic
- Ryzen 7 3800X
- Rmagine Version (v2.1.0) - Only the Embree (CPU) backend is used
- RMCL (v1.1.2)
- (optional) For visualizations: https://github.com/aock/mesh_tools

The reproduction of the resulting Bag-File can be done by executing the respective launch-files in this repository. Each launch-file requires two files: a Bag-File for sensor data and a Mesh-Map-file as map for MICP-L.

### Internal

All required files are stored on the Rab-fileserver at the base-path "datasets/mulran".

Prepare the data by doing the following steps:
1. download a Mulran bag file from Rab "mulran/bags",
2. download a mesh map from Rab "micp_mulran",


### External

1. generate the bag file
    - download KAIST sequences from https://sites.google.com/view/mulran-pr/download
    - convert them using https://github.com/aock/mulran2bag
2. generate a mesh map (how I have done it)
    - take sick_pointcloud.las from KAIST02 folder and open in with CloudCompare. Shift the map, to avoid floating point issues and store it as PLY file.
    - convert PLY point cloud to mesh using https://github.com/uos/lvr2
    - cut out dynamic obstacles using MeshLab

It is planned to upload at least the meshes so that you can download them instead. I will announce it here then.

### Launching

Change paths in launch files, so that they are pointing to the right files.
Each KAIST launch file should point to the same map but to different bag files then.
Next, execute a launch file to check if everything went fine, e.g.:

```console
roslaunch micp_mulran kaist01_micp_gps_imu.launch gui:=true
```

A RViz window is opening. Wait a few moments for the mesh file to be loaded and then press SPACE in the console to unpause the bag-file. The results should look like:

![Teaser](dat/micp_mulran.png)

#### Recording

Execute a launch file without GUI (RViz):

```console
roslaunch micp_mulran kaist01_micp_gps_imu.launch gui:=false
```

and pipe the results into a new Bag-File, by running `rosbag record -a` in another terminal.

The resulting Bag-File now consists of an additional transformation between `world` and `map` frame that represents the localization of the autonomous car in the loaded mesh.


#### Evaluation Files

To produce evaluation files you have of execute. 

```console
roslaunch micp_mulran kaist01_micp_gps_imu.launch gui:=false generate_evaluation:=true rate:=0.5
```

Since generating the evaluation metrics requires some time, the bag file rate was halved here, to prevent false results because of doing the evaluation. 

