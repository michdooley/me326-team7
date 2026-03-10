#!/usr/bin/env python3
"""
IK Workspace Scan Visualizer

Loads ik_scan_right_arm.csv and/or ik_scan_left_arm.csv and visualises
reachability as coloured point clouds in Viser, with the robot meshes
rendered at the home configuration for spatial reference.

Usage:
    python3 ik_scan_visualizer.py                        # looks for CSVs in cwd
    python3 ik_scan_visualizer.py --dir /path/to/csvs    # custom directory
    python3 ik_scan_visualizer.py --right my_right.csv --left my_left.csv

Then open http://localhost:8080 in a browser.
"""

import argparse
import csv
import time
from pathlib import Path

import mujoco
import numpy as np
import trimesh
import viser


# ── Robot mesh loading via MuJoCo ────────────────────────────────────────────

REPO = Path(__file__).resolve().parent.parent.parent.parent.parent
MODEL_PATH = REPO / 'simulation' / 'assets' / 'mujoco' / 'tidybot_wx250s_bimanual.xml'


def load_robot_meshes(model_path: Path) -> list[trimesh.Trimesh]:
    """Load MuJoCo model at home keyframe and extract all visual geom meshes."""
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # Set home keyframe if available
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    meshes = []
    for g in range(model.ngeom):
        geom_type = model.geom_type[g]
        geom_group = model.geom_group[g]

        # Skip collision-only geoms (group >= 3) and non-visual
        if geom_group >= 3:
            continue

        pos = data.geom_xpos[g].copy()
        mat = data.geom_xmat[g].reshape(3, 3).copy()
        size = model.geom_size[g].copy()
        rgba = model.geom_rgba[g].copy()

        # Skip fully transparent
        if rgba[3] < 0.01:
            continue

        mesh = None

        if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_id = model.geom_dataid[g]
            if mesh_id < 0:
                continue
            # Extract mesh data
            vert_start = model.mesh_vertadr[mesh_id]
            vert_count = model.mesh_vertnum[mesh_id]
            face_start = model.mesh_faceadr[mesh_id]
            face_count = model.mesh_facenum[mesh_id]

            verts = model.mesh_vert[vert_start:vert_start + vert_count].copy()
            faces = model.mesh_face[face_start:face_start + face_count].copy()

            mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)

        elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            mesh = trimesh.creation.box(extents=size * 2)

        elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            mesh = trimesh.creation.icosphere(subdivisions=2, radius=size[0])

        elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            mesh = trimesh.creation.cylinder(radius=size[0], height=size[1] * 2)

        elif geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
            mesh = trimesh.creation.capsule(radius=size[0], height=size[1] * 2)

        if mesh is None:
            continue

        # Transform to world frame
        transform = np.eye(4)
        transform[:3, :3] = mat
        transform[:3, 3] = pos
        mesh.apply_transform(transform)

        # Apply colour
        color_rgba = (rgba * 255).astype(np.uint8)
        # Make slightly transparent so point clouds show through
        color_rgba[3] = min(color_rgba[3], 180)
        mesh.visual.face_colors = color_rgba

        meshes.append(mesh)

    return meshes


# ── CSV loading ──────────────────────────────────────────────────────────────

def load_scan_csv(path: Path) -> dict:
    """Load an IK scan CSV and return arrays."""
    xs, ys, zs = [], [], []
    successes, pos_errs, conds = [], [], []

    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            zs.append(float(row['z']))
            successes.append(int(row['success']))
            pos_errs.append(float(row['pos_err']))
            conds.append(float(row['cond']))

    return {
        'points': np.column_stack([xs, ys, zs]),
        'success': np.array(successes, dtype=bool),
        'pos_err': np.array(pos_errs),
        'cond': np.array(conds),
    }


# ── Colouring ────────────────────────────────────────────────────────────────

def color_by_success(success: np.ndarray) -> np.ndarray:
    """Green = reachable, red = unreachable."""
    colors = np.zeros((len(success), 3), dtype=np.uint8)
    colors[success, 1] = 200   # green
    colors[~success, 0] = 200  # red
    return colors


def color_by_condition(cond: np.ndarray, success: np.ndarray,
                       vmin: float = 0, vmax: float = 500) -> np.ndarray:
    """Blue (low cond) -> yellow (high cond) for successes; grey for failures."""
    colors = np.full((len(cond), 3), 60, dtype=np.uint8)  # grey default
    mask = success & (cond > 0)
    if mask.any():
        t = np.clip((cond[mask] - vmin) / (vmax - vmin + 1e-9), 0, 1)
        # Blue (good) -> green -> yellow (near-singular)
        colors[mask, 0] = (t * 255).astype(np.uint8)
        colors[mask, 1] = (np.clip(1 - abs(2 * t - 1), 0, 1) * 255).astype(np.uint8)
        colors[mask, 2] = ((1 - t) * 255).astype(np.uint8)
    return colors


def color_by_pos_error(pos_err: np.ndarray, success: np.ndarray) -> np.ndarray:
    """Green (low error) -> red (high error) for successes; grey for failures."""
    colors = np.full((len(pos_err), 3), 60, dtype=np.uint8)
    mask = success & (pos_err >= 0)
    if mask.any():
        errs = pos_err[mask]
        vmax = max(errs.max(), 0.01)
        t = np.clip(errs / vmax, 0, 1)
        colors[mask, 0] = (t * 255).astype(np.uint8)
        colors[mask, 1] = ((1 - t) * 255).astype(np.uint8)
        colors[mask, 2] = 0
    return colors


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='IK Scan Visualizer')
    parser.add_argument('--dir', type=str, default='.',
                        help='Directory containing CSV files')
    parser.add_argument('--right', type=str, default=None,
                        help='Path to right arm CSV')
    parser.add_argument('--left', type=str, default=None,
                        help='Path to left arm CSV')
    parser.add_argument('--model', type=str, default=None,
                        help='MuJoCo XML model path (default: auto-detect)')
    args = parser.parse_args()

    # Resolve CSV paths
    csv_dir = Path(args.dir)
    datasets = {}
    right_path = Path(args.right) if args.right else csv_dir / 'ik_scan_right_arm.csv'
    left_path = Path(args.left) if args.left else csv_dir / 'ik_scan_left_arm.csv'

    if right_path.exists():
        print(f'Loading right arm: {right_path}')
        datasets['right'] = load_scan_csv(right_path)
    if left_path.exists():
        print(f'Loading left arm: {left_path}')
        datasets['left'] = load_scan_csv(left_path)

    if not datasets:
        print(f'No CSV files found. Looked for:\n  {right_path}\n  {left_path}')
        return

    for name, d in datasets.items():
        n_ok = d['success'].sum()
        n_total = len(d['success'])
        print(f'  {name}: {n_ok}/{n_total} reachable '
              f'({100 * n_ok / n_total:.1f}%)')

    # Load robot meshes
    model_path = Path(args.model) if args.model else MODEL_PATH
    print(f'Loading robot model: {model_path}')
    robot_meshes = load_robot_meshes(model_path)
    print(f'  Loaded {len(robot_meshes)} geom meshes')

    # ── Viser server ─────────────────────────────────────────────────────
    server = viser.ViserServer()

    # Grid + origin
    server.scene.add_grid(
        name='/scene/grid',
        width=1.0, height=1.0,
        cell_size=0.05,
        plane='xy',
    )
    server.scene.add_frame(
        name='/scene/origin',
        axes_length=0.15,
        axes_radius=0.004,
    )

    # Robot meshes
    for i, mesh in enumerate(robot_meshes):
        server.scene.add_mesh_trimesh(
            name=f'/robot/geom_{i:03d}',
            mesh=mesh,
        )

    # ── State ────────────────────────────────────────────────────────────
    cloud_handles: dict[str, viser.PointCloudHandle | None] = {}

    # ── GUI ───────────────────────────────────────────────────────────────
    with server.gui.add_folder('Display'):
        show_right = server.gui.add_checkbox('Right arm', initial_value='right' in datasets)
        show_left = server.gui.add_checkbox('Left arm', initial_value='left' in datasets)
        show_robot = server.gui.add_checkbox('Robot model', initial_value=True)
        show_failures = server.gui.add_checkbox('Show failures', initial_value=True)

    with server.gui.add_folder('Coloring'):
        color_mode = server.gui.add_dropdown(
            'Mode',
            options=['Success / Fail', 'Condition Number', 'Position Error'],
            initial_value='Success / Fail',
        )
        cond_max_slider = server.gui.add_slider(
            'Cond max', min=50, max=2000, step=50, initial_value=500)

    with server.gui.add_folder('Filter'):
        point_size_slider = server.gui.add_slider(
            'Point size', min=0.002, max=0.03, step=0.001, initial_value=0.008)
        z_min_slider = server.gui.add_slider(
            'Z min', min=-0.1, max=0.5, step=0.01, initial_value=-0.1)
        z_max_slider = server.gui.add_slider(
            'Z max', min=0.0, max=0.6, step=0.01, initial_value=0.6)

    with server.gui.add_folder('Actions'):
        refresh_btn = server.gui.add_button('Refresh')

    # ── Rendering ────────────────────────────────────────────────────────
    robot_handles: list = []
    for i, mesh in enumerate(robot_meshes):
        h = server.scene.add_mesh_trimesh(
            name=f'/robot/geom_{i:03d}',
            mesh=mesh,
        )
        robot_handles.append(h)

    def update_clouds():
        """Recompute and display point clouds."""
        # Remove old clouds
        for key in list(cloud_handles.keys()):
            h = cloud_handles.pop(key)
            if h is not None:
                h.remove()

        for arm_name, data in datasets.items():
            if arm_name == 'right' and not show_right.value:
                continue
            if arm_name == 'left' and not show_left.value:
                continue

            pts = data['points']
            success = data['success']
            cond = data['cond']
            pos_err = data['pos_err']

            # Z filter
            z_mask = (pts[:, 2] >= z_min_slider.value) & (pts[:, 2] <= z_max_slider.value)

            # Success filter
            if show_failures.value:
                mask = z_mask
            else:
                mask = z_mask & success

            if mask.sum() == 0:
                continue

            pts_f = pts[mask]
            success_f = success[mask]
            cond_f = cond[mask]
            pos_err_f = pos_err[mask]

            # Colour
            mode = color_mode.value
            if mode == 'Condition Number':
                colors = color_by_condition(cond_f, success_f,
                                            vmax=cond_max_slider.value)
            elif mode == 'Position Error':
                colors = color_by_pos_error(pos_err_f, success_f)
            else:
                colors = color_by_success(success_f)

            cloud_handles[arm_name] = server.scene.add_point_cloud(
                name=f'/scan/{arm_name}',
                points=pts_f.astype(np.float32),
                colors=colors,
                point_size=point_size_slider.value,
            )

    def update_robot_visibility():
        vis = show_robot.value
        for h in robot_handles:
            h.visible = vis

    # Wire up callbacks
    def _cb(_):
        update_clouds()

    show_right.on_update(_cb)
    show_left.on_update(_cb)
    show_failures.on_update(_cb)
    color_mode.on_update(_cb)
    cond_max_slider.on_update(_cb)
    point_size_slider.on_update(_cb)
    z_min_slider.on_update(_cb)
    z_max_slider.on_update(_cb)
    refresh_btn.on_click(lambda _: update_clouds())
    show_robot.on_update(lambda _: update_robot_visibility())

    # Initial render
    update_clouds()

    print('\nOpen http://localhost:8080 in your browser.')
    print('Press Ctrl+C to stop.\n')

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
