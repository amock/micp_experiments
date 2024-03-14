# Install KISS-ICP and Plotting tools

import os
from pathlib import Path

import kiss_icp

import matplotlib
matplotlib.use('TKAgg')

import matplotlib.pyplot as plt

import numpy as np
from evo.tools import plot
from kiss_icp.datasets import dataset_factory
from kiss_icp.pipeline import OdometryPipeline
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP

from kiss_icp.deskew import get_motion_compensator
from kiss_icp.mapping import get_voxel_hash_map
from kiss_icp.preprocess import get_preprocessor
from kiss_icp.registration import register_frame
from kiss_icp.threshold import get_threshold_estimator
from kiss_icp.voxelization import voxel_down_sample

import open3d as o3d
import numpy as np
import struct
from os import listdir
from os.path import isfile, join

import trimesh

import copy
import math

from puma.projections import project_scan_to_mesh


def rigid_inv(T):
    Tinv = np.eye(4)
    Tinv[:3, :3] = T[:3, :3].T
    Tinv[:3, 3] = - (Tinv[:3, :3] @ T[:3, 3])
    return Tinv


# PARAMS START
T_ouster_base = np.eye(4)
T_ouster_base[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz((0.0, 0.0, 3.136))
T_ouster_base[0, 3] = 1.704
T_ouster_base[1, 3] = -0.021
T_ouster_base[2, 3] = 1.805

enable_visualization = False
first_cloud_only = False
write_evaluation = True
debug_prints = True
# larger than 5m correspondences are not considered for P2M errors
outlier_dist = 5.0

# scanner range
lidar_min_range = 0.3
lidar_max_range = 80.0


# KAIST or what?
dataset = "KAIST03"


print("Constructing KISS-ICP!")
config = KISSConfig()


if dataset == "KAIST01":
    mesh_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/lvr2/KAIST/kaist02_mesh_red05_cleaned4.ply"
    pcl_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/kaist02_sampled_100p.ply"

    data_root = "/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST01"

    T_base_map_init = np.eye(4, 4)
    T_base_map_init[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz((0.037806, 0.0252877, 2.03837))
    # original: -115.620, 109.790, 20.5
    # better:
    T_base_map_init[0, 3] = -112.45
    T_base_map_init[1, 3] = 108.0
    T_base_map_init[2, 3] = 19.32

    # kiss config
    config.mapping.voxel_size = 1.0
    config.mapping.max_points_per_voxel = 20
    config.data.deskew = False
    config.data.max_range = 80.0
    config.data.min_range = 0.3
    config.adaptive_threshold.initial_threshold = 2.0
    config.adaptive_threshold.min_motion_th = 0.1


if dataset == "KAIST02":
    mesh_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/lvr2/KAIST/kaist02_mesh_red05_cleaned4.ply"
    pcl_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/kaist02_sampled_100p.ply"

    data_root = "/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST02"

    T_base_map_init = np.eye(4, 4)
    T_base_map_init[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz((0.0, 0.0, 2.15))
    # original: -41.0, -35.0, 20.0
    # better:
    T_base_map_init[0, 3] = -40.77
    T_base_map_init[1, 3] = -36.017
    T_base_map_init[2, 3] = 19.24

    # kiss config
    config.mapping.voxel_size = 1.0
    config.mapping.max_points_per_voxel = 20
    config.data.deskew = False
    config.data.max_range = 80.0
    config.data.min_range = 0.3
    config.adaptive_threshold.initial_threshold = 2.0
    config.adaptive_threshold.min_motion_th = 0.1


if dataset == "KAIST03":
    mesh_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/lvr2/KAIST/kaist02_mesh_red05_cleaned4.ply"
    pcl_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/kaist02_sampled_100p.ply"

    data_root = "/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST03"

    T_base_map_init = np.eye(4, 4)
    T_base_map_init[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz((0.0, 0.0, 2.14))
    # original: 2.0, -103, 20.0
    # better:
    T_base_map_init[0, 3] = 1.597
    T_base_map_init[1, 3] = -103.12
    T_base_map_init[2, 3] = 19.35

    config.mapping.voxel_size = 1.0
    config.mapping.max_points_per_voxel = 20
    config.data.deskew = False
    config.data.max_range = 100.0
    config.data.min_range = 5.0
    config.adaptive_threshold.initial_threshold = 2.0
    config.adaptive_threshold.min_motion_th = 0.1


# PARAMS END
def load_velo_scan(file):
    """Load and parse a velodyne binary file."""
    scan = np.fromfile(file, dtype = np.float32)
    return scan.reshape((-1, 4))

def vel2pcd(points, use_intensity=False):
    pcd = o3d.geometry.PointCloud()
    points_xyz = points[:, :3]
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    if use_intensity:
        points_i = points[:, -1].reshape(-1, 1)
        pcd.colors = o3d.utility.Vector3dVector(
            np.full_like(points_xyz, points_i)
        )
    return pcd

T_base_odom_init = np.eye(4)
T_odom_map_init = T_base_map_init @ rigid_inv(T_base_odom_init)

# for iterations
T_base_map = T_base_map_init
T_base_odom = T_base_odom_init
T_odom_map = T_odom_map_init

T_map_odom = rigid_inv(T_odom_map)


print("Loading mesh map...")
mesh = o3d.io.read_triangle_mesh(mesh_filename)
mesh.transform(T_map_odom)
mesh.compute_triangle_normals()


print("O3D Mesh. Vertices:", len(mesh.vertices), ", Faces:", len(mesh.triangles), ", Normals:", len(mesh.triangle_normals))

# for puma
tmesh = trimesh.Trimesh(
        vertices=np.asarray(mesh.vertices), faces=np.asarray(mesh.triangles), process=False
    )
# tmesh = trimesh.load(mesh_filename, process=False)
# print("Trimesh. Vertices:", len(tmesh.vertices), ", Faces:", len(tmesh.faces) )


# T_map_odom = rigid_inv(T_odom_map)
# print(T_map_odom)
# exit()

pcl_map = o3d.io.read_point_cloud(pcl_filename)
print("PCL Map (for KISS): ", len(pcl_map.points) )

print("Transform map to odom frame. at the beginning the robot is (0,0,0) of the map")
pcl_map.transform(T_map_odom)
# pcl_map_in_odom_frame = copy.deepcopy(pcl_map).transform(rigid_inv(T_odom_map))


print("Order scans...")
pcl_dir = data_root + "/sensor_data/Ouster"
pcl_stamps = []

for f in listdir(pcl_dir):
    if isfile(join(pcl_dir, f)):
        if f[-3:] == "bin":
            stamp_str = f[:-4]
            pcl_stamps.append((int(stamp_str), stamp_str))
            
pcl_stamps.sort()

def find_nearest(array, value):
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array) or math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return array[idx-1]
    else:
        return array[idx]
    
gt_data = np.genfromtxt(data_root + '/global_pose.csv', delimiter=',')
gps_data = np.genfromtxt(data_root + "/sensor_data/gps.csv", delimiter=',')
imu_data = np.genfromtxt(data_root + "/sensor_data/xsens_imu.csv", delimiter=',')

imu_data_dict = {}
for elem in imu_data:
    imu_data_dict[int(elem[0])] = elem[1:]

gps_data_dict = {}
for elem in gps_data:
    gps_data_dict[int(elem[0])] = elem[1:]


# IMU
# 1561000444344536407, # timestamp
# 0.0014902208931744098663, -0.010735403746366500854,-0.74799680709838867188,-0.66361379623413085938, # quaternion x, quaternion y, quaternion z, quaternion w,
# 0.80698882421701845225,0.94414453517379370862,96.848543557224772371, # Euler x, Euler y, Euler z
# -0.0014431802555918693542,0.00047535009798593824562,-0.0025875857099890708923, # Gyro x, Gyro y, Gyro z,
# -0.10445132106542588668,0.1200853958725929399,9.91283416748046875, # Acceleration x, Acceleration y, Acceleration z
# 0.56985861063003551141,-0.10887633264064788818,-0.8626144528388978161 # MagnetField x, MagnetField y, MagnetField z

print(gps_data.shape)
print(imu_data.shape)



data_overview = []

for elem in gps_data:
    data_overview.append((int(elem[0]), "GPS"))

for elem in imu_data:
    data_overview.append((int(elem[0]), "IMU"))

for elem in pcl_stamps:
    data_overview.append((int(elem[0]), "PCL"))

data_overview.sort()



# -0.5652699981,-0.8248605171,-0.008657748933,352960.9191,
# 0.8248791602,-0.565135918,-0.01399161292,4026351.581,
# 0.006648324176,-0.01505063568,0.9998646299,18.64625672

print(gt_data.shape)


print("Start registration...")

first_stamp_ns = pcl_stamps[0][0]
# gt_stamps = gt_data[:,0]
# gt_poses = gt_data[:,1:]
# gt_stamps_rel = (gt_poses[:,0] - first_stamp_ns)
# gt_stamps_min_idx = np.abs(gt_stamps_rel).argmin()

# print(first_stamp_ns, gt_poses[gt_stamps_min_idx])

# tf_echo
#     Translation: [1.704, -0.021, 1.805]
# - Rotation: in Quaternion [0.000, -0.000, 1.000, 0.003]
#             in RPY (radian) [-0.000, -0.000, 3.136]

# init transformation
# transformation = T_base_map_init @ T_ouster_base

first_cloud = True


# get odometry from 

# borrowed from robot_localization
RADIANS_PER_DEGREE = np.pi / 180.0
DEGREES_PER_RADIAN = 180.0 / np.pi

# WGS84 Parameters
WGS84_A = 6378137.0         # major axis
WGS84_B = 6356752.31424518  # minor axis
WGS84_F = 0.0033528107      # ellipsoid flattening
WGS84_E = 0.0818191908      # first eccentricity
WGS84_EP = 0.0820944379     # second eccentricity

# UTM Parameters
UTM_K0 = 0.9996                   # scale factor
UTM_FE = 500000.0                 # false easting
UTM_FN_N = 0.0                    # false northing, northern hemisphere
UTM_FN_S = 10000000.0             # false northing, southern hemisphere
UTM_E2 = (WGS84_E * WGS84_E)      # e^2
UTM_E4 = (UTM_E2 * UTM_E2)        # e^4
UTM_E6 = (UTM_E4 * UTM_E2)        # e^6
UTM_EP2 = (UTM_E2 / (1 - UTM_E2)) # e'^2


def utm(lat, lon, alt):
    # constants
    m0 = (1 - UTM_E2 / 4 - 3 * UTM_E4 / 64 - 5 * UTM_E6 / 256)
    m1 = -(3 * UTM_E2 / 8 + 3 * UTM_E4 / 32 + 45 * UTM_E6 / 1024)
    m2 = (15 * UTM_E4 / 256 + 45 * UTM_E6 / 1024)
    m3 = -(35 * UTM_E6 / 3072)

    # compute the central meridian
    cm = int(lon)
    if lon >= 0.0:
        cm -= int(lon) % 6 + 3
    else:
        cm -= int(lon) % 6 - 3

    # convert degrees into radians
    rlat = lat * RADIANS_PER_DEGREE
    rlon = lon * RADIANS_PER_DEGREE
    rlon0 = cm * RADIANS_PER_DEGREE

    # compute trigonometric functions
    slat = np.sin(rlat)
    clat = np.cos(rlat)
    tlat = np.tan(rlat)

    # decide the false northing at origin
    fn = None
    if lat > 0:
        fn = UTM_FN_N
    else:
        fn = UTM_FN_S

    T = tlat * tlat
    C = UTM_EP2 * clat * clat
    A = (rlon - rlon0) * clat
    M = WGS84_A * (m0 * rlat + m1 * np.sin(2 * rlat) + m2 * np.sin(4 * rlat) + m3 * np.sin(6 * rlat))
    V = WGS84_A / np.sqrt(1 - UTM_E2 * slat * slat)

    # compute the easting-northing coordinates
    x = UTM_FE + UTM_K0 * V * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * UTM_EP2) * pow(A, 5) / 120)
    y = fn + UTM_K0 * (M + V * tlat * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + ((61 - 58 * T + T * T + 600 * C - 330 * UTM_EP2) * pow(A, 6) / 720)))
    z = alt

    return np.array([x, y, z])

    # rm::Vector3d gps_glob_current;
    # gps_glob_current.z = gps_msg->altitude;
    # robot_localization::navsat_conversions::UTM(
    #     gps_msg->latitude, gps_msg->longitude, 
    #     &gps_glob_current.x, &gps_glob_current.y);

gps_last = np.array([0.0, 0.0, 0.0])
gps_stamp_last = 0
gps_vel_est = 0.0

first_run = True

# IMU & GPS
first_msg_imu_gps = True
first_stamp_imu_gps = None

# GPS
n_gps_messages = 0
gps_glob_stamp_start = None
gps_glob_start = None
gps_heading_start = None
gps_stamp_last = None
gps_last = None
gps_lin_vel = None
gps_vel = 0.0
gps_cov = None
gps_heading_found = False

# ODOM STATE
pos = np.array([0.0, 0.0, 0.0])

# IMU
n_imu_messages = 0
imu_pos = np.array([0.0, 0.0, 0.0])
imu_lin_vel = np.array([0.0, 0.0, 0.0])
imu_lin_acc = np.array([0.0, 0.0, 0.0])
imu_vel = 0.0
n_imu_since_last = 0
last_imu_msg = None
last_imu_stamp = None
imu_heading_start = None
imu_orientation = None

# T_base_odom = np.eye(4)
# T_odom_map = np.eye(4)

# Fusion
# 0: speed from gps, orientation from imu
# 1: speed from gps and imu lin acc (kalman filter), orientation from imu
fuse_mode = 0

max_acc = 1.0
max_dec = 2.0
max_speed = 50.0 / 3.6


def gps_cb(stamp, data):
    global first_msg_imu_gps, first_stamp_imu_gps, n_gps_messages, gps_glob_stamp_start, gps_glob_start, gps_last, gps_vel, gps_stamp_last
    global max_acc, max_dec, max_speed, fuse_mode

    print(stamp, "GPS")

    if first_msg_imu_gps:
        first_stamp_imu_gps = stamp
        first_msg_imu_gps = False

    time_since_start = stamp - first_stamp_imu_gps
    gps_glob_current = utm(data[0], data[1], data[2])

    if n_gps_messages == 0:
        gps_glob_stamp_start = stamp
        gps_glob_start = gps_glob_current

    gps = gps_glob_current - gps_glob_start

    if n_gps_messages > 0:
        gps_local = gps - gps_last
        gps_dt = (stamp - gps_stamp_last) / (1000.0 * 1000.0 * 1000.0)
        gps_lin_vel = gps_local / gps_dt

        gps_vel_old = gps_vel
        gps_vel = np.linalg.norm(gps_lin_vel)


        if gps_vel - gps_vel_old > max_acc:
            gps_vel = gps_vel_old + max_acc
        
        if gps_vel - gps_vel_old < -max_dec:
            gps_vel = gps_vel_old - max_dec

        if gps_vel > max_speed:
            gps_vel = max_speed

        if gps_vel < -max_speed:
            gps_vel = max_speed

        if fuse_mode == 1:
            print("fuse_mode 1 - NOT IMPLEMENTED!")
            exit()

    gps_last = gps
    gps_stamp_last = stamp
    n_gps_messages += 1

# IMU data
# 0.0014902208931744098663, -0.010735403746366500854,-0.74799680709838867188,-0.66361379623413085938, # quaternion x, quaternion y, quaternion z, quaternion w,
# 0.80698882421701845225,0.94414453517379370862,96.848543557224772371, # Euler x, Euler y, Euler z
# -0.0014431802555918693542,0.00047535009798593824562,-0.0025875857099890708923, # Gyro x, Gyro y, Gyro z,
# -0.10445132106542588668,0.1200853958725929399,9.91283416748046875, # Acceleration x, Acceleration y, Acceleration z
# 0.56985861063003551141,-0.10887633264064788818,-0.8626144528388978161 # MagnetField x, MagnetField y, MagnetField z
def imu_cb(stamp, data):
    global first_msg_imu_gps, first_stamp_imu_gps, n_imu_messages, pos, imu_heading_start, last_imu_stamp, T_base_odom

    print(stamp, "IMU")

    if first_msg_imu_gps:
        first_stamp_imu_gps = stamp
        first_msg_imu_gps = False

    qx = data[0]
    qy = data[1]
    qz = data[2]
    qw = data[3]

    if n_imu_messages == 0:
        pos[0] = 0.0
        pos[1] = 0.0
        pos[2] = 0.0

        # quaternion
        imu_heading_start = o3d.geometry.get_rotation_matrix_from_quaternion([qw, qx, qy, qz])
        # imu_heading_start = data[:4]
    
    orient_glob = o3d.geometry.get_rotation_matrix_from_quaternion([qw, qx, qy, qz])

    imu_orientation = imu_heading_start.T @ orient_glob


    if n_imu_messages > 0:
        dt = (stamp - last_imu_stamp) / (1000.0 * 1000.0 * 1000.0)
        dist = gps_vel * dt
        next_pos_delta = imu_orientation @ np.array([dist, 0.0, 0.0])
        pos = pos + next_pos_delta

        # pos is intregrated along IMU orientation
        # IMU orientation stays absolute

        # actually, T_imu_odom. If it doesnt work check this first
        T_base_odom = np.eye(4)
        T_base_odom[:3,:3] = imu_orientation
        T_base_odom[:3,3] = pos

    last_imu_stamp = stamp
    n_imu_messages = n_imu_messages + 1


vis = None
if enable_visualization:
    vis = o3d.visualization.Visualizer()

    vis.create_window()
    vis.add_geometry(mesh)

    vis_ctr = vis.get_view_control()
    vis_ctr.set_zoom(1.0/16.0)
    vis.poll_events()
    vis.update_renderer()

eval_file = None
if write_evaluation:
    eval_file = open("KISS_" + dataset + "_eval.txt", "w")

cloud_count = 0

print("Constructing KISS-ICP!")
kiss_icp = KissICP(config)

print("KISS: Map")
# pcl_map_np = np.asarray(pcl_map.points)
kiss_icp.initialize_local_map(np.asarray(pcl_map.points))
kiss_icp.disable_map_updates()

print("Done")

def pcl_cb(stamp, velo):
    global T_base_odom # initial guess
    global T_odom_map, T_ouster_base, method, debug_prints, max_iterations, max_dist, vis, vis_ctr
    global eval_file, enable_visualization
    global cloud_count
    global lidar_min_range, lidar_max_range

    # filter points in range
    npoints = velo.shape[0]

    # re-use fourth dim (intensity) as homegenous dimension
    velo[:,3] = np.ones(velo.shape[0])

    velo_base = (T_ouster_base @ velo.T).T
    pcl_filtered, pcl_filtered_2 = kiss_icp.register_frame(velo_base[:,:3], timestamps = None, initial_guess = None)
    # corrected
    T_base_odom_kiss = kiss_icp.poses[-1]
    T_ouster_odom_kiss = T_base_odom_kiss @ T_ouster_base

    if enable_visualization:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(velo[:,:3])
        pcd.transform(T_ouster_odom_kiss)
        vis.add_geometry(pcd)

        # render
        vis_ctr = vis.get_view_control()
        vis_ctr.set_lookat(T_ouster_odom_kiss[:3,3])
        vis_ctr.set_zoom(1.0/16.0)

        vis.poll_events()
        vis.update_renderer()

        vis.remove_geometry(pcd)

    # overwrite
    T_base_odom = T_base_odom_kiss


    if write_evaluation:
        Tbo = T_base_odom_kiss.flatten()
        eval_str = ("{}," * 16 + "{}\n").format(stamp, Tbo[0], Tbo[1], Tbo[2], Tbo[3], Tbo[4], Tbo[5], Tbo[6], Tbo[7], Tbo[8], Tbo[9], Tbo[10], Tbo[11], Tbo[12], Tbo[13], Tbo[14], Tbo[15])
    #     # since RCC correspondences were filtered by max_dist
    #     # For P2M error we need even the bad ones
    #     # I did the same for MICP-L
    #     source_red, target = project_scan_to_mesh(tmesh, source, outlier_dist)
    #     # distances = source_red.compute_point_cloud_distance(target)

    #     p2ms = []

    #     for (sp, tp, tn) in zip(source_red.points, target.points, target.normals):
    #         signed_dist = (sp - tp).dot(tn)
    #         # print(sp, tp, tn, "->", signed_dist)
    #         dist = np.abs(signed_dist)
    #         p2ms.append(dist)

    #     p2ms = np.array(p2ms)

    #     p2m_mean = np.mean(p2ms)
    #     p2m_median = np.median(p2ms)


    #     Tbm = T_base_map.flatten()
    #     eval_str = ("{},"*21 + "{}\n").format(stamp, Tbm[0], Tbm[1], Tbm[2], Tbm[3], Tbm[4], Tbm[5], Tbm[6], Tbm[7], Tbm[8], Tbm[9], Tbm[10], Tbm[11], Tbm[12], Tbm[13], Tbm[14], Tbm[15], npoints, nvalidpoints, ninlier, p2m_mean, p2m_median)
        print("Writing: ", eval_str)
        eval_file.write(eval_str)

    cloud_count += 1


for i, (stamp, sensor_type) in enumerate(data_overview):
    if sensor_type == "GPS":
        gps_cb(stamp, gps_data_dict[stamp])
    elif sensor_type == "IMU":
        imu_cb(stamp, imu_data_dict[stamp])
    elif sensor_type == "PCL":
        # source = None
        pcl_filename = pcl_dir + "/" + str(stamp) + ".bin"
        print("Loading PCL from '" + pcl_filename + "'")
        velo = load_velo_scan(pcl_filename)


        # endless loop to align only the first cloud
        if first_cloud_only:
            while True:
                pcl_cb(stamp, copy.deepcopy(velo) )

        pcl_cb(stamp, velo)
    else:
        print("ERROR sensor_type wrong:", sensor_type)
        exit()

if enable_visualization:
    vis.destroy_window()
