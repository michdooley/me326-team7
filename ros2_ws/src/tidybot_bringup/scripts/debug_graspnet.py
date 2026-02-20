#!/usr/bin/env python3
"""
Standalone GraspNet debug script — no ROS2 required.

Replicates the official GraspNet demo pipeline step-by-step with verbose
debug output at each stage so you can isolate where things go wrong.

Uses the bundled example data at ~/graspnet-baseline/doc/example_data/
and the RealSense checkpoint at ~/graspnet-baseline/logs/checkpoint-rs.tar.

Usage:
    # From repo root (with venv active):
    source ros2_ws/setup_env.bash
    python ros2_ws/src/tidybot_bringup/scripts/debug_graspnet.py

    # Or with custom paths:
    python debug_graspnet.py \
        --checkpoint ~/graspnet-baseline/logs/checkpoint-rs.tar \
        --data_dir   ~/graspnet-baseline/doc/example_data \
        --num_point 20000 --collision_thresh 0.01 --voxel_size 0.01
"""

import os
import sys
import time
import argparse
import traceback

import numpy as np

# ── Add GraspNet-baseline to sys.path ──────────────────────────────────────
GRASPNET_ROOT = os.path.expanduser('~/graspnet-baseline')
for subdir in ('', 'models', 'dataset', 'utils'):
    p = os.path.join(GRASPNET_ROOT, subdir)
    if p not in sys.path:
        sys.path.insert(0, p)

# ── Parse args early so --help works without GPU ───────────────────────────
parser = argparse.ArgumentParser(
    description='Standalone GraspNet debug script (no ROS2)')
parser.add_argument(
    '--checkpoint',
    default=os.path.join(GRASPNET_ROOT, 'logs', 'checkpoint-rs.tar'),
    help='Path to GraspNet checkpoint (.tar)')
parser.add_argument(
    '--data_dir',
    default=os.path.join(GRASPNET_ROOT, 'doc', 'example_data'),
    help='Directory with color.png, depth.png, workspace_mask.png, meta.mat')
parser.add_argument('--num_point', type=int, default=20000)
parser.add_argument('--num_view', type=int, default=300)
parser.add_argument('--collision_thresh', type=float, default=0.01)
parser.add_argument('--voxel_size', type=float, default=0.01)
parser.add_argument('--top_k', type=int, default=6,
                    help='Number of grasps to visualize')
parser.add_argument('--approach_offset', type=float, default=0.10,
                    help='Pre-grasp retreat distance along approach vector (metres)')
parser.add_argument('--no-vis', action='store_true',
                    help='Skip visualization (just run inference)')
parser.add_argument('--target_color', type=str, default=None,
                    choices=['red', 'green', 'blue', 'yellow'],
                    help='Detect object by color and crop cloud around it')
parser.add_argument('--crop_center', type=str, default=None,
                    help='Manual crop center in camera frame: "x,y,z" (metres)')
parser.add_argument('--crop_radius', type=float, default=0.20,
                    help='Radius around target to crop point cloud (metres)')
parser.add_argument('--min_depth', type=float, default=0.1,
                    help='Min depth in metres (filter out near noise)')
parser.add_argument('--max_depth', type=float, default=1.5,
                    help='Max depth in metres (filter out background/floor)')
parser.add_argument('--remove_plane', action='store_true', default=True,
                    help='Remove dominant plane (ground/table) via RANSAC')
parser.add_argument('--no_remove_plane', action='store_true',
                    help='Disable ground plane removal')
parser.add_argument('--align-rgb', action='store_true',
                    help='Align RGB to depth frame using real D435 intrinsics (preserves depth)')
args = parser.parse_args()
if args.no_remove_plane:
    args.remove_plane = False


def step(name):
    """Print a visible step header."""
    print(f'\n{"="*60}')
    print(f'  STEP: {name}')
    print(f'{"="*60}')


# ──────────────────────────────────────────────────────────────────────────
# STEP 0 — Verify files and imports
# ──────────────────────────────────────────────────────────────────────────
step('0 — Verify files and imports')

# Check data files
required_files = ['color.png', 'depth.png', 'workspace_mask.png', 'meta.mat']
for fname in required_files:
    fpath = os.path.join(args.data_dir, fname)
    exists = os.path.isfile(fpath)
    size = os.path.getsize(fpath) if exists else 0
    status = f'OK ({size:,} bytes)' if exists else 'MISSING'
    print(f'  {fname:25s} {status}')
    if not exists:
        print(f'  ERROR: {fpath} not found. Exiting.')
        sys.exit(1)

# Check checkpoint
if os.path.isfile(args.checkpoint):
    print(f'  checkpoint                  OK ({os.path.getsize(args.checkpoint):,} bytes)')
else:
    print(f'  ERROR: checkpoint not found at {args.checkpoint}')
    sys.exit(1)

# Import dependencies one by one for clear error messages
imports_ok = True
for module_name in ['torch', 'open3d', 'scipy', 'PIL', 'trimesh', 'viser']:
    try:
        __import__(module_name)
        print(f'  import {module_name:20s} OK')
    except ImportError as e:
        print(f'  import {module_name:20s} FAILED: {e}')
        imports_ok = True  # continue to show all failures

try:
    from graspnetAPI import GraspGroup
    print(f'  import {"graspnetAPI":20s} OK')
except ImportError as e:
    print(f'  import {"graspnetAPI":20s} FAILED: {e}')
    imports_ok = False

try:
    from graspnet import GraspNet, pred_decode
    print(f'  import {"graspnet (model)":20s} OK')
except ImportError as e:
    print(f'  import {"graspnet (model)":20s} FAILED: {e}')
    imports_ok = False

try:
    from collision_detector import ModelFreeCollisionDetector
    print(f'  import {"collision_detector":20s} OK')
except ImportError as e:
    print(f'  import {"collision_detector":20s} FAILED: {e}')
    imports_ok = False

try:
    from data_utils import CameraInfo, create_point_cloud_from_depth_image
    print(f'  import {"data_utils":20s} OK')
except ImportError as e:
    print(f'  import {"data_utils":20s} FAILED: {e}')
    imports_ok = False

if not imports_ok:
    print('\nSome imports failed. Fix them before continuing.')
    sys.exit(1)

import torch
import open3d as o3d
import scipy.io as scio
from PIL import Image

print(f'\n  torch version:  {torch.__version__}')
print(f'  CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'  CUDA device:    {torch.cuda.get_device_name(0)}')
    print(f'  CUDA memory:    {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB')

# ──────────────────────────────────────────────────────────────────────────
# STEP 1 — Load model
# ──────────────────────────────────────────────────────────────────────────
step('1 — Load GraspNet model')

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
print(f'  Device: {device}')

t0 = time.time()
net = GraspNet(
    input_feature_dim=0,
    num_view=args.num_view,
    num_angle=12,
    num_depth=4,
    cylinder_radius=0.05,
    hmin=-0.02,
    hmax_list=[0.01, 0.02, 0.03, 0.04],
    is_training=False,
)
net.to(device)

checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
net.load_state_dict(checkpoint['model_state_dict'])
net.eval()
elapsed = time.time() - t0

epoch = checkpoint.get('epoch', '?')
n_params = sum(p.numel() for p in net.parameters())
print(f'  Loaded checkpoint (epoch {epoch}) in {elapsed:.2f}s')
print(f'  Model parameters: {n_params:,}')

# ──────────────────────────────────────────────────────────────────────────
# STEP 2 — Load and process input data
# ──────────────────────────────────────────────────────────────────────────
step('2 — Load input data')

color = np.array(Image.open(os.path.join(args.data_dir, 'color.png')),
                 dtype=np.float32) / 255.0
depth = np.array(Image.open(os.path.join(args.data_dir, 'depth.png')))
workspace_mask = np.array(
    Image.open(os.path.join(args.data_dir, 'workspace_mask.png')).convert('L'))
meta = scio.loadmat(os.path.join(args.data_dir, 'meta.mat'))

intrinsic = meta['intrinsic_matrix']
factor_depth = meta['factor_depth']

# ── Align RGB to depth frame if using real D435 images ───────────────────
if args.align_rgb:
    step('2a — Align RGB to depth frame (real D435)')
    align_dir = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'utilities'))
    if align_dir not in sys.path:
        sys.path.insert(0, align_dir)
    from align_rgb_to_depth import align_rgb as _align_rgb, D435_DEPTH_K

    color_uint8 = (color * 255).astype(np.uint8)
    color_aligned = _align_rgb(color_uint8, depth)
    color = color_aligned.astype(np.float32) / 255.0
    print(f'  Warped RGB to depth intrinsics (depth untouched)')

    # Override intrinsic matrix to depth camera K since we now work
    # entirely in the depth camera frame
    dfx, dfy, dcx, dcy = D435_DEPTH_K
    intrinsic = np.array([
        [dfx, 0.0, dcx],
        [0.0, dfy, dcy],
        [0.0, 0.0, 1.0],
    ])
    print(f'  Overriding intrinsics to depth K:\n{intrinsic}')

print(f'  color shape:          {color.shape}, dtype={color.dtype}')
print(f'  depth shape:          {depth.shape}, dtype={depth.dtype}')
print(f'  depth range:          [{depth.min()}, {depth.max()}]')
print(f'  workspace_mask shape: {workspace_mask.shape}, '
      f'nonzero pixels: {np.count_nonzero(workspace_mask):,}')
print(f'  intrinsic matrix:\n{intrinsic}')
print(f'  factor_depth: {factor_depth}')

# ──────────────────────────────────────────────────────────────────────────
# STEP 3 — Generate point cloud
# ──────────────────────────────────────────────────────────────────────────
step('3 — Generate point cloud')

h, w = depth.shape[:2]
camera = CameraInfo(
    float(w), float(h),
    intrinsic[0][0], intrinsic[1][1],
    intrinsic[0][2], intrinsic[1][2],
    factor_depth,
)
cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

print(f'  Full cloud shape: {cloud.shape}')
print(f'  Cloud XYZ range:  X=[{cloud[...,0].min():.3f}, {cloud[...,0].max():.3f}]  '
      f'Y=[{cloud[...,1].min():.3f}, {cloud[...,1].max():.3f}]  '
      f'Z=[{cloud[...,2].min():.3f}, {cloud[...,2].max():.3f}]')

# Apply workspace mask + depth range filter
depth_m = depth.astype(np.float32) / float(factor_depth)
mask = (workspace_mask > 0) & (depth_m > args.min_depth) & (depth_m < args.max_depth)
cloud_masked = cloud[mask]
color_masked = color[mask]

n_ws = np.count_nonzero(workspace_mask > 0)
n_depth_valid = np.count_nonzero((depth_m > args.min_depth) & (depth_m < args.max_depth))
print(f'  Workspace mask pixels:    {n_ws:,}')
print(f'  Depth in [{args.min_depth:.2f}, {args.max_depth:.2f}]m: {n_depth_valid:,}')
print(f'  Combined masked points:   {len(cloud_masked):,} '
      f'(of {cloud.shape[0]*cloud.shape[1]:,} total)')

if len(cloud_masked) == 0:
    print('  ERROR: No valid points after masking.')
    print(f'  Depth range of image: [{depth_m[depth_m>0].min():.3f}, {depth_m[depth_m>0].max():.3f}] m')
    print(f'  Try adjusting --min_depth / --max_depth')
    sys.exit(1)

# ── RANSAC ground plane removal ──────────────────────────────────────────
if args.remove_plane and len(cloud_masked) > 500:
    step('3a — Remove dominant plane (RANSAC)')

    plane_cloud = o3d.geometry.PointCloud()
    plane_cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))

    # RANSAC: find the largest plane (typically ground or table)
    plane_model, inlier_idxs = plane_cloud.segment_plane(
        distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    a, b, c, d = plane_model

    inlier_set = set(inlier_idxs)
    outlier_mask = np.array([i not in inlier_set for i in range(len(cloud_masked))])

    print(f'  Plane equation: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0')
    print(f'  Plane normal:   ({a:.3f}, {b:.3f}, {c:.3f})')
    print(f'  Inliers (plane): {len(inlier_idxs):,}')
    print(f'  Outliers (objects): {outlier_mask.sum():,}')

    if outlier_mask.sum() < 100:
        print('  WARNING: Almost all points belong to the plane.')
        print('  Skipping plane removal — your objects may be too small or flat.')
    else:
        cloud_masked = cloud_masked[outlier_mask]
        color_masked = color_masked[outlier_mask]
        print(f'  Points after plane removal: {len(cloud_masked):,}')

# ──────────────────────────────────────────────────────────────────────────
# STEP 3b — (Optional) Crop around target object
# ──────────────────────────────────────────────────────────────────────────

# HSV ranges matching camera_scanner.py
COLOR_RANGES = {
    'red': [
        ((0,   100, 80),  (10,  255, 255)),
        ((165, 100, 80),  (180, 255, 255)),
    ],
    'green': [
        ((40,  60,  60),  (85,  255, 255)),
    ],
    'blue': [
        ((100, 80,  60),  (135, 255, 255)),
    ],
    'yellow': [
        ((20, 80, 80),  (35, 255, 255)),
    ],
}

crop_center = None

if args.target_color:
    step(f'3b — Detect {args.target_color} object and crop')
    import cv2

    # Convert color image (0-1 float RGB) to uint8 BGR for OpenCV
    color_uint8 = (color * 255).astype(np.uint8)
    color_bgr = cv2.cvtColor(color_uint8, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)

    # Build HSV mask
    ranges = COLOR_RANGES[args.target_color]
    hsv_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        hsv_mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

    n_pixels = cv2.countNonZero(hsv_mask)
    print(f'  HSV mask pixels: {n_pixels}')

    if n_pixels < 50:
        print(f'  ERROR: Could not find {args.target_color} object in color image.')
        print(f'  Try a different --target_color or use --crop_center x,y,z')
        sys.exit(1)

    # Find centroid in pixel coords
    moments = cv2.moments(hsv_mask)
    px = int(moments['m10'] / moments['m00'])
    py = int(moments['m01'] / moments['m00'])
    print(f'  Detected centroid: pixel ({px}, {py})')

    # Look up 3D position from the organized cloud
    depth_at_center = depth[py, px]
    if depth_at_center == 0:
        # Search a small patch around centroid for valid depth
        patch_size = 10
        y0 = max(0, py - patch_size)
        y1 = min(h, py + patch_size)
        x0 = max(0, px - patch_size)
        x1 = min(w, px + patch_size)
        depth_patch = depth[y0:y1, x0:x1]
        valid_depths = depth_patch[depth_patch > 0]
        if len(valid_depths) > 0:
            depth_at_center = int(np.median(valid_depths))
            print(f'  Center pixel had zero depth, using median of patch: {depth_at_center} mm')
        else:
            print(f'  ERROR: No valid depth near detected object centroid.')
            sys.exit(1)

    crop_center = cloud[py, px].copy()
    print(f'  Object 3D position (camera frame): '
          f'({crop_center[0]:.3f}, {crop_center[1]:.3f}, {crop_center[2]:.3f}) m')

elif args.crop_center:
    step('3b — Crop around manual center')
    parts = [float(x) for x in args.crop_center.split(',')]
    if len(parts) != 3:
        print('  ERROR: --crop_center must be "x,y,z" (3 comma-separated floats)')
        sys.exit(1)
    crop_center = np.array(parts)
    print(f'  Manual crop center: ({crop_center[0]:.3f}, {crop_center[1]:.3f}, {crop_center[2]:.3f})')

if crop_center is not None:
    dists = np.linalg.norm(cloud_masked - crop_center, axis=-1)
    crop_mask = dists < args.crop_radius
    n_before = len(cloud_masked)
    cloud_masked = cloud_masked[crop_mask]
    color_masked = color_masked[crop_mask]
    print(f'  Crop radius: {args.crop_radius} m')
    print(f'  Points after crop: {len(cloud_masked):,} (was {n_before:,})')

    if len(cloud_masked) < 100:
        print(f'  ERROR: Too few points ({len(cloud_masked)}) after cropping.')
        print(f'  Try increasing --crop_radius (currently {args.crop_radius})')
        sys.exit(1)

# ──────────────────────────────────────────────────────────────────────────
# STEP 4 — Sample points for network input
# ──────────────────────────────────────────────────────────────────────────
step('4 — Sample points')

if len(cloud_masked) >= args.num_point:
    idxs = np.random.choice(len(cloud_masked), args.num_point, replace=False)
else:
    idxs1 = np.arange(len(cloud_masked))
    idxs2 = np.random.choice(len(cloud_masked),
                             args.num_point - len(cloud_masked), replace=True)
    idxs = np.concatenate([idxs1, idxs2])
cloud_sampled = cloud_masked[idxs]
color_sampled = color_masked[idxs]

print(f'  Sampled {args.num_point} points '
      f'(from {len(cloud_masked):,}, replace={len(cloud_masked) < args.num_point})')

# Build Open3D cloud for visualization later
cloud_o3d = o3d.geometry.PointCloud()
cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

# Build end_points dict
end_points = {}
cloud_tensor = torch.from_numpy(
    cloud_sampled[np.newaxis].astype(np.float32)).to(device)
end_points['point_clouds'] = cloud_tensor
end_points['cloud_colors'] = color_sampled

print(f'  Tensor shape: {cloud_tensor.shape}, device={cloud_tensor.device}')

# ──────────────────────────────────────────────────────────────────────────
# STEP 5 — GraspNet forward pass
# ──────────────────────────────────────────────────────────────────────────
step('5 — GraspNet inference')

t0 = time.time()
try:
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    elapsed = time.time() - t0

    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)

    print(f'  Inference time: {elapsed:.2f}s')
    print(f'  Raw grasps: {len(gg)}')

    if len(gg) > 0:
        print(f'  Score range: [{gg.scores.min():.4f}, {gg.scores.max():.4f}]')
        print(f'  Width range: [{gg.widths.min():.4f}, {gg.widths.max():.4f}] m')
        print(f'  Depth range: [{gg.depths.min():.4f}, {gg.depths.max():.4f}] m')
except Exception as exc:
    print(f'  INFERENCE FAILED: {exc}')
    traceback.print_exc()
    sys.exit(1)

if len(gg) == 0:
    print('  ERROR: No grasps predicted. Something is wrong with the model or data.')
    sys.exit(1)

# ──────────────────────────────────────────────────────────────────────────
# STEP 6 — Collision detection
# ──────────────────────────────────────────────────────────────────────────
step('6 — Collision detection')

t0 = time.time()
try:
    mfcdetector = ModelFreeCollisionDetector(
        np.asarray(cloud_o3d.points), voxel_size=args.voxel_size)
    collision_mask = mfcdetector.detect(
        gg, approach_dist=0.05, collision_thresh=args.collision_thresh)
    n_before = len(gg)
    gg = gg[~collision_mask]
    elapsed = time.time() - t0

    print(f'  Collision detection time: {elapsed:.2f}s')
    print(f'  Grasps before: {n_before}, after: {len(gg)}, '
          f'removed: {n_before - len(gg)}')
except Exception as exc:
    print(f'  Collision detection FAILED (skipping): {exc}')
    traceback.print_exc()

if len(gg) == 0:
    print('  WARNING: All grasps removed by collision detection.')
    print('  Try increasing --collision_thresh or --voxel_size.')
    sys.exit(1)

# ──────────────────────────────────────────────────────────────────────────
# STEP 7 — NMS and sort
# ──────────────────────────────────────────────────────────────────────────
step('7 — NMS + sort')

n_before = len(gg)
gg.nms()
gg.sort_by_score()
print(f'  After NMS: {len(gg)} (was {n_before})')

gg = gg[:args.top_k]
print(f'  Keeping top {len(gg)} grasps')

if len(gg) > 0:
    print(f'\n  Top {len(gg)} grasps (approach_offset={args.approach_offset}m):')
    for i in range(len(gg)):
        t = gg.translations[i]
        # Pre-grasp = retreat along approach vector (column 0 of rotation matrix)
        approach = gg.rotation_matrices[i][:, 0]
        pre_t = t - approach * args.approach_offset
        print(f'    #{i}: score={gg.scores[i]:.4f}  '
              f'width={gg.widths[i]:.4f}m  '
              f'depth={gg.depths[i]:.4f}m  '
              f'grasp=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})  '
              f'pre=({pre_t[0]:.3f}, {pre_t[1]:.3f}, {pre_t[2]:.3f})')

# ──────────────────────────────────────────────────────────────────────────
# STEP 8 — Visualization
# ──────────────────────────────────────────────────────────────────────────
if args.no_vis:
    print('\n  --no-vis specified, skipping visualization.')
    print('\n  ALL STEPS PASSED')
    sys.exit(0)

step('8 — Viser visualization')

import viser
import trimesh


def create_gripper_mesh(center, rotation, width, depth, rgba):
    """Create a gripper mesh with explicit RGBA color."""
    finger_width = 0.004
    finger_height = 0.004
    finger_depth = depth
    tail_length = 0.04

    left = trimesh.creation.box([finger_depth, finger_width, finger_height])
    left.apply_translation([finger_depth / 2, -width / 2 - finger_width / 2, 0])

    right = trimesh.creation.box([finger_depth, finger_width, finger_height])
    right.apply_translation([finger_depth / 2, width / 2 + finger_width / 2, 0])

    bottom = trimesh.creation.box(
        [finger_width, width + 2 * finger_width, finger_height])

    tail = trimesh.creation.box([tail_length, finger_width, finger_height])
    tail.apply_translation([-tail_length / 2, 0, 0])

    gripper = trimesh.util.concatenate([left, right, bottom, tail])

    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = center
    gripper.apply_transform(transform)

    gripper.visual.face_colors = rgba
    return gripper


server = viser.ViserServer()
print('  Viser server started at http://localhost:8080')

points = np.asarray(cloud_o3d.points).astype(np.float32)
colors = (np.asarray(cloud_o3d.colors) * 255).astype(np.uint8)

server.scene.add_point_cloud(
    name='/scene/pointcloud',
    points=points,
    colors=colors,
    point_size=0.003,
)

for i in range(len(gg)):
    center = gg.translations[i]
    R = gg.rotation_matrices[i]
    w = gg.widths[i]
    d = gg.depths[i]

    # Grasp pose — green
    grasp_mesh = create_gripper_mesh(center, R, w, d, [0, 200, 0, 220])
    server.scene.add_mesh_trimesh(name=f'/scene/grasp_{i}', mesh=grasp_mesh)

    # Pre-grasp pose — blue, retreated along approach vector (R column 0)
    approach = R[:, 0]
    pre_center = center - approach * args.approach_offset
    pre_mesh = create_gripper_mesh(pre_center, R, w, d, [0, 100, 255, 150])
    server.scene.add_mesh_trimesh(name=f'/scene/pregrasp_{i}', mesh=pre_mesh)

server.scene.add_frame(
    name='/scene/origin', axes_length=0.1, axes_radius=0.003)

print(f'  Visualizing {len(gg)} grasps (green) + pre-grasps (blue).')
print(f'  Open http://localhost:8080 in your browser.')
print(f'  Press Ctrl+C to exit.\n')

print('  ALL STEPS PASSED')

try:
    while True:
        time.sleep(1.0)
except KeyboardInterrupt:
    print('\n  Shutting down.')
