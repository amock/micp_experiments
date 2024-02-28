
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


dataset = "KAIST03"


if dataset == "KAIST01":
    mesh_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/lvr2/KAIST/kaist02_mesh_red05_cleaned3.ply"
    data_root = "/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST01"
    eval_file = "KAIST01_eval.txt"

if dataset == "KAIST02":
    mesh_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/lvr2/KAIST/kaist02_mesh_red05_cleaned3.ply"
    data_root = "/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST02"
    eval_file = "KAIST02_eval.txt"

if dataset == "KAIST03":
    mesh_filename = "/media/amock/OricoAlex/uni/datasets/mulran_meshes/lvr2/KAIST/kaist02_mesh_red05_cleaned3.ply"
    data_root = "/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST03"
    eval_file = "KAIST03_eval.txt"



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
mesh = o3d.io.read_triangle_mesh(mesh_filename)
mesh.compute_triangle_normals()

print("Vertices:", len(mesh.vertices), ", Faces:", len(mesh.triangles), ", Normals:", len(mesh.triangle_normals))
    
# tmesh = trimesh.Trimesh(
#         vertices=np.asarray(mesh.vertices), faces=np.asarray(mesh.triangles)
#     )

print("Order scans...")
pcl_dir = "/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST01/sensor_data/Ouster"
pcl_stamps = []

for f in listdir(pcl_dir):
    if isfile(join(pcl_dir, f)):
        if f[-3:] == "bin":
            stamp_str = f[:-4]
            pcl_stamps.append((int(stamp_str), stamp_str))
            
pcl_stamps.sort()

T_ouster_base = np.eye(4)
T_ouster_base[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz((0.0, 0.0, 3.136))
T_ouster_base[0, 3] = 1.704
T_ouster_base[1, 3] = -0.021
T_ouster_base[2, 3] = 1.805


poses_file = open("KAIST01_eval.txt", "r")
poses_dict = {}

for line in poses_file.readlines():
    line = line.strip()
    elems = line.split(",")
    stamp = elems[0]

    T = [float(elems[1]), float(elems[2]), float(elems[3]), float(elems[4]), float(elems[5]), float(elems[6]), float(elems[7]), float(elems[8]), float(elems[9]), float(elems[10]), float(elems[11]), float(elems[12]), float(elems[13]), float(elems[14]), float(elems[15]), float(elems[16])]
    T = np.array(T).reshape((4,4))
    npoints = int(elems[17])
    nvalidpoints = int(elems[18])
    ninliers = int(elems[19])

    p2m_mean = float(elems[20])
    p2m_median = float(elems[21])

    entry = (T, npoints, ninliers, p2m_mean, p2m_median)
    
    poses_dict[stamp] = (T, npoints, ninliers, p2m_mean, p2m_median)



vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh, False)

source = None

for i, (pcl_stamp_ns, pcl_stamp_str) in enumerate(pcl_stamps):
    # vis.poll_events()

    if not source is None:
        vis.remove_geometry(source)

    # vis.poll_events()
    pcl_filename = pcl_dir + "/" + pcl_stamp_str + ".bin"
    print("Loading PCL from '" + pcl_filename + "'")
    velo = load_velo_scan(pcl_filename)
    print("Points:", velo.shape)
    source = vel2pcd(velo)


    poses_dict_entry = poses_dict[pcl_stamp_str]
    print("Got:", poses_dict_entry)
    T_base_map = poses_dict_entry[0]
    
    T_ouster_map = T_base_map @ T_ouster_base
    source.transform(T_ouster_map)

    vis.add_geometry(source)

    vis_ctr = vis.get_view_control()
    vis_ctr.set_lookat(T_ouster_map[:3,3])
    vis_ctr.set_zoom(1.0/10.0)

    vis.poll_events()
    vis.update_renderer()


vis.destroy_window()
    # looks good!
    # 1561000444390857630

