
import open3d as o3d
from puma.registration import register_scan_to_mesh, run_icp
import numpy as np
import struct
from os import listdir
from os.path import isfile, join
import trimesh

from puma.projections import project_scan_to_mesh
from puma.registration import get_te_method
import copy
import math

import time



def load_velo_scan(file):
    """Load and parse a velodyne binary file."""
    scan = np.fromfile(file, dtype=np.float32)
    return scan.reshape((-1, 4))

def vel2pcd(points, use_intensity=False):
    pcd = o3d.geometry.PointCloud()
    points_xyz = points[:, :3]
    points_i = points[:, -1].reshape(-1, 1)
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    if use_intensity:
        pcd.colors = o3d.utility.Vector3dVector(
            np.full_like(points_xyz, points_i)
        )
    return pcd

def align_clouds(source, target, method):
    """Align 2 PointCloud objects assuming 1-1 correspondences."""
    assert len(source.points) == len(target.points), "N of points must match!"
    corr = np.zeros((len(source.points), 2))
    corr[:, 0] = np.arange(len(source.points))
    corr[:, 1] = np.arange(len(target.points))
    te = get_te_method(method)
    return te.compute_transformation(
        source, target, o3d.utility.Vector2iVector(corr)
    )

print("Loading mesh map...")
mesh_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/lvr2/KAIST/kaist02_mesh_red05_cleaned.ply"
mesh = o3d.io.read_triangle_mesh(mesh_filename)
mesh.compute_triangle_normals()

print("Vertices:", len(mesh.vertices), ", Faces:", len(mesh.triangles), ", Normals:", len(mesh.triangle_normals))
    
tmesh = trimesh.Trimesh(
        vertices=np.asarray(mesh.vertices), faces=np.asarray(mesh.triangles)
    )

print("Order scans...")
pcl_dir = "/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST01/sensor_data/Ouster"
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
    
gt_data = np.genfromtxt('/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST01/global_pose.csv', delimiter=',')

# -0.5652699981,-0.8248605171,-0.008657748933,352960.9191,
# 0.8248791602,-0.565135918,-0.01399161292,4026351.581,
# 0.006648324176,-0.01505063568,0.9998646299,18.64625672

print(gt_data.shape)


print("Start registration...")

first_stamp_ns = pcl_stamps[0][0]
gt_stamps = gt_data[:,0]
gt_poses = gt_data[:,1:]
gt_stamps_rel = (gt_poses[:,0] - first_stamp_ns)
gt_stamps_min_idx = np.abs(gt_stamps_rel).argmin()

print(first_stamp_ns, gt_poses[gt_stamps_min_idx])



# tf_echo
#     Translation: [1.704, -0.021, 1.805]
# - Rotation: in Quaternion [0.000, -0.000, 1.000, 0.003]
#             in RPY (radian) [-0.000, -0.000, 3.136]

T_ouster_base = np.eye(4)
T_ouster_base[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz((0.0, 0.0, 3.136))
T_ouster_base[0, 3] = 1.704
T_ouster_base[1, 3] = -0.021
T_ouster_base[2, 3] = 1.805

T_base_map_init = np.eye(4, 4)
T_base_map_init[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz((0.0172851, 0.00124588, 2.03213))
T_base_map_init[0, 3] = -116.685
T_base_map_init[1, 3] = 116.442
T_base_map_init[2, 3] = 19.2952



# init transformation
transformation = T_base_map_init @ T_ouster_base


# transformation = 
# transformation = [-116.685, 116.442, 19.2952, 0.0172851, 0.00124588, 2.03213]



# transformation[0][0] = 

max_dist = 2.0
max_iterations=30
method = "p2l" # p2p, p2l, gicp
debug = True
tolerance = 0.00001

first_cloud = True

poses_file = open("puma_poses.txt", "r")

poses_dict = {}

for line in poses_file.readlines():
    line = line.strip()
    elems = line.split(", ")
    stamp = elems[0]


    T = [float(elems[1]), float(elems[2]), float(elems[3]), float(elems[4]), float(elems[5]), float(elems[6]), float(elems[7]), float(elems[8]), float(elems[9]), float(elems[10]), float(elems[11]), float(elems[12]), float(elems[13]), float(elems[14]), float(elems[15]), float(elems[16])]
    T = np.array(T).reshape((4,4))
    npoints = int(elems[17])
    ninliers = int(elems[18])
    p2m_mean = float(elems[19])
    p2m_median = float(elems[20])

    entry = (T, npoints, ninliers, p2m_mean, p2m_median)
    # print("E:", entry)
    # exit()

    poses_dict[stamp] = (T, npoints, ninliers, p2m_mean, p2m_median)



vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)

source = None

for i, (pcl_stamp_ns, pcl_stamp_str) in enumerate(pcl_stamps):

    # vis.poll_events()

    if not source is None:
        vis.remove_geometry(source)

    # vis.poll_events()

    pcl_filename = pcl_dir + "/" + pcl_stamp_str + ".bin"
    print("Loading PCL from '" + pcl_filename + "'")
    velo = load_velo_scan(pcl_filename)
    source = vel2pcd(velo)
    source.estimate_normals()
    # vis.poll_events()

    # 1561000444390857630 [-4.59286812e-01 -8.88104044e-01  1.80784840e-02  3.53050983e+05
#   8.88283970e-01 -4.59251113e-01  6.32477280e-03  4.02606103e+06
#   2.68550759e-03  1.89637123e-02  9.99816566e-01  1.93249802e+01]

    time_since_start_ns = pcl_stamp_ns - first_stamp_ns
    time_since_start_s = time_since_start_ns / (1000*1000*1000)

    poses_dict_entry = poses_dict[pcl_stamp_str]
    print("Got:", poses_dict_entry)
    transformation = poses_dict_entry[0]
    source.transform(transformation)

    # vis.poll_events()

    # time.sleep(0.1)
    # vis.poll_events()
    # o3d.visualization.draw_geometries([mesh, source])

    vis.add_geometry(source)

    vis.poll_events()
    vis.update_renderer()



vis.destroy_window()
    # looks good!
    # 1561000444390857630

