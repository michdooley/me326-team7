""" Demo to show prediction results.
    Author: chenxi-wang
"""

import os
import sys
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image
import time

import torch
import viser
import trimesh
from graspnetAPI import GraspGroup

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from graspnet_dataset import GraspNetDataset
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path')
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
cfgs = parser.parse_args()


def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)"%(cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net

def get_and_process_data(data_dir):
    # load data
    color = np.array(Image.open(os.path.join(data_dir, 'color.png')), dtype=np.float32) / 255.0
    depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
    workspace_mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png')))
    meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
    intrinsic = meta['intrinsic_matrix']
    factor_depth = meta['factor_depth']

    # generate cloud
    camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    mask = (workspace_mask & (depth > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud

def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg

def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg

def create_gripper_mesh(center, rotation, width, depth, score):
    """
    Create a gripper mesh for visualization.

    Args:
        center: (3,) grasp center position
        rotation: (3,3) rotation matrix
        width: gripper opening width
        depth: grasp depth
        score: grasp score for coloring (0-1)

    Returns:
        trimesh.Trimesh: gripper mesh
    """
    # Gripper dimensions (matching graspnetAPI)
    finger_width = 0.004
    finger_height = 0.004
    finger_depth = depth
    tail_length = 0.04
    depth_base = 0.02

    # Create gripper parts in local coordinates
    # The gripper consists of: left finger, right finger, bottom, tail

    # Left finger
    left_finger = trimesh.creation.box([finger_depth, finger_width, finger_height])
    left_finger.apply_translation([finger_depth / 2, -width / 2 - finger_width / 2, 0])

    # Right finger
    right_finger = trimesh.creation.box([finger_depth, finger_width, finger_height])
    right_finger.apply_translation([finger_depth / 2, width / 2 + finger_width / 2, 0])

    # Bottom connecting the fingers
    bottom = trimesh.creation.box([finger_width, width + 2 * finger_width, finger_height])
    bottom.apply_translation([0, 0, 0])

    # Tail (support)
    tail = trimesh.creation.box([tail_length, finger_width, finger_height])
    tail.apply_translation([-tail_length / 2, 0, 0])

    # Combine all parts
    gripper = trimesh.util.concatenate([left_finger, right_finger, bottom, tail])

    # Apply rotation and translation
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = center
    gripper.apply_transform(transform)

    # Color based on score: interpolate from blue (low) to green to red (high)
    if score < 0.5:
        r = int(score * 2 * 255)
        g = int(score * 2 * 255)
        b = int((1 - score * 2) * 255)
    else:
        r = int(255)
        g = int((1 - (score - 0.5) * 2) * 255)
        b = 0

    gripper.visual.face_colors = [r, g, b, 200]

    return gripper


def vis_grasps(gg, cloud):
    """Visualize grasps using viser browser-based visualization."""
    gg.nms()
    gg.sort_by_score()
    gg = gg[:50]

    # Create viser server
    server = viser.ViserServer()
    print("Viser server started at http://localhost:8080")

    # Extract point cloud data from Open3D object
    points = np.asarray(cloud.points).astype(np.float32)
    colors = (np.asarray(cloud.colors) * 255).astype(np.uint8)

    # Add point cloud to scene
    server.scene.add_point_cloud(
        name="/scene/pointcloud",
        points=points,
        colors=colors,
        point_size=0.003,
    )

    # Add each gripper mesh
    for i in range(len(gg)):
        center = gg.translations[i]
        rotation = gg.rotation_matrices[i]
        width = gg.widths[i]
        depth = gg.depths[i]
        score = gg.scores[i]

        # Create gripper mesh
        mesh = create_gripper_mesh(center, rotation, width, depth, score)

        # Add to viser scene
        server.scene.add_mesh_trimesh(
            name=f"/scene/gripper_{i}",
            mesh=mesh,
        )

    # Add coordinate frame for reference
    server.scene.add_frame(
        name="/scene/origin",
        axes_length=0.1,
        axes_radius=0.003,
    )

    print(f"Visualizing {len(gg)} grasps. Open http://localhost:8080 in your browser.")
    print("Press Ctrl+C to exit.")

    # Keep server running (blocking)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nShutting down viser server...")

def demo(data_dir):
    net = get_net()
    end_points, cloud = get_and_process_data(data_dir)
    gg = get_grasps(net, end_points)
    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    vis_grasps(gg, cloud)

if __name__=='__main__':
    data_dir = 'doc/example_data'
    demo(data_dir)
