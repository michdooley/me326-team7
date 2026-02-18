#!/usr/bin/env python3
"""
Grasp Reachability Map

Sweeps a grid of (x, y) positions in base_link and tests whether the IK
solver can find a valid top-down grasp at each point.  Produces a heatmap
showing reachable vs unreachable regions for each arm.

This is a pure offline tool — no ROS nodes or simulation required.
It loads the MuJoCo model directly and runs Mink IK in a tight loop.

Usage:
    # From repo root (source setup_env.bash first for PYTHONPATH):
    python3 ros2_ws/src/tidybot_bringup/scripts/grasp_reachability_map.py

    # Custom grid / arm:
    python3 .../grasp_reachability_map.py --arm left --z 0.03 --step 0.01

    # Save to file instead of displaying:
    python3 .../grasp_reachability_map.py --save reachability.png
"""

import argparse
import time
from pathlib import Path

import numpy as np
import mujoco
import mink
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap


# ── Robot constants ───────────────────────────────────────────────────────

# Fingers-down orientation (wxyz) for top-down grasp in base_link
ORIENT_FINGERS_DOWN = np.array([0.5, 0.5, 0.5, -0.5])

# Default base pose: robot at origin facing +X world (base_th = pi/2)
BASE_TH = np.pi / 2

# Joint limits (from tidybot_wx250s_bimanual.xml)
JOINT_LIMITS = {
    'waist': (-3.14159, 3.14159),
    'shoulder': (-1.8849, 1.9897),
    'elbow': (-2.1468, 1.6057),
    'forearm_roll': (-3.14159, 3.14159),
    'wrist_angle': (-1.7453, 2.1468),
    'wrist_rotate': (-3.14159, 3.14159),
}

DEFAULT_SEED = np.array([0.0, -1.0, 0.8, 0.0, 0.5, 0.0])

ARM_JOINTS = {
    'right': ['right_waist', 'right_shoulder', 'right_elbow',
              'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate'],
    'left': ['left_waist', 'left_shoulder', 'left_elbow',
             'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate'],
}

EE_SITES = {
    'right': 'right_pinch_site',
    'left': 'left_pinch_site',
}


# ── IK Solver (standalone, no ROS) ───────────────────────────────────────

class OfflineIKSolver:
    """Minimal IK solver extracted from MotionPlannerNode."""

    def __init__(self, model_path: str, arm: str = 'right'):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.configuration = mink.Configuration(self.model)
        self.arm = arm

        # Joint address lookup
        self.joint_qpos_addrs = {}
        for jname in ARM_JOINTS[arm]:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            self.joint_qpos_addrs[jname] = self.model.jnt_qposadr[jid]

        # Base joint addresses
        self.base_addrs = {}
        for jname in ['joint_x', 'joint_y', 'joint_th']:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            self.base_addrs[jname] = self.model.jnt_qposadr[jid]

        # Build velocity limit to freeze all non-arm joints
        arm_joint_set = set(ARM_JOINTS[arm])
        vel_limits = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name and name not in arm_joint_set:
                jtype = self.model.jnt_type[i]
                if jtype != mujoco.mjtJoint.mjJNT_FREE:
                    vel_limits[name] = np.array([1e-10])
        self.freeze_limit = mink.VelocityLimit(self.model, vel_limits)

    def _set_base(self):
        self.data.qpos[self.base_addrs['joint_x']] = 0.0
        self.data.qpos[self.base_addrs['joint_y']] = 0.0
        self.data.qpos[self.base_addrs['joint_th']] = BASE_TH

    def _base_link_to_world(self, pos_base, quat_base_wxyz):
        """Convert base_link pose to world frame (same as MotionPlannerNode.pose_to_se3)."""
        c, s = np.cos(BASE_TH), np.sin(BASE_TH)
        R_base = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        world_pos = R_base @ pos_base

        base_quat = np.array([np.cos(BASE_TH / 2), 0.0, 0.0, np.sin(BASE_TH / 2)])
        base_rot = mink.SO3(base_quat)
        target_rot = mink.SO3(quat_base_wxyz)
        world_rot = base_rot @ target_rot

        return mink.SE3.from_rotation_and_translation(
            rotation=world_rot, translation=world_pos)

    def solve(self, x, y, z, max_iters=300, dt=0.002,
              pos_tol=0.01, ori_tol=0.1):
        """
        Test IK for a top-down grasp at (x, y, z) in base_link.

        Returns dict with: success, pos_error, ori_error, condition_number, joint_positions
        """
        target_se3 = self._base_link_to_world(
            np.array([x, y, z]), ORIENT_FINGERS_DOWN)

        # Reset config with seed
        self.data.qpos[:] = 0
        self._set_base()
        for i, jname in enumerate(ARM_JOINTS[self.arm]):
            self.data.qpos[self.joint_qpos_addrs[jname]] = DEFAULT_SEED[i]
        mujoco.mj_forward(self.model, self.data)
        self.configuration.update(self.data.qpos)

        # Frame task
        ee_task = mink.FrameTask(
            frame_name=EE_SITES[self.arm],
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
        )
        ee_task.set_target(target_se3)

        # Solve
        for _ in range(max_iters):
            vel = mink.solve_ik(
                self.configuration, [ee_task],
                dt=dt, solver="quadprog", damping=1e-3,
                limits=[self.freeze_limit],
            )
            self.configuration.integrate_inplace(vel, dt)
            if np.linalg.norm(vel) < 1e-6:
                break

        # Extract + clamp solution
        solution = np.zeros(6)
        for i, jname in enumerate(ARM_JOINTS[self.arm]):
            addr = self.joint_qpos_addrs[jname]
            raw = self.configuration.q[addr]
            solution[i] = (raw + np.pi) % (2 * np.pi) - np.pi

        joint_names = ['waist', 'shoulder', 'elbow',
                       'forearm_roll', 'wrist_angle', 'wrist_rotate']
        for i, jn in enumerate(joint_names):
            lo, hi = JOINT_LIMITS[jn]
            solution[i] = np.clip(solution[i], lo, hi)

        # FK with clamped solution
        self._set_base()
        for i, jname in enumerate(ARM_JOINTS[self.arm]):
            self.data.qpos[self.joint_qpos_addrs[jname]] = solution[i]
        mujoco.mj_forward(self.model, self.data)

        site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, EE_SITES[self.arm])
        actual_pos = self.data.site_xpos[site_id].copy()
        actual_mat = self.data.site_xmat[site_id].reshape(3, 3)

        target_pos = target_se3.translation()
        pos_err = np.linalg.norm(actual_pos - target_pos)

        target_mat = target_se3.rotation().as_matrix()
        R_diff = target_mat.T @ actual_mat
        cos_angle = np.clip((np.trace(R_diff) - 1) / 2, -1, 1)
        ori_err = np.arccos(cos_angle)

        # Condition number
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, site_id)
        arm_cols = []
        for jname in ARM_JOINTS[self.arm]:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            arm_cols.append(self.model.jnt_dofadr[jid])
        J = jacp[:, arm_cols]
        try:
            cond = np.linalg.cond(J @ J.T)
        except Exception:
            cond = float('inf')

        success = (pos_err < pos_tol and ori_err < ori_tol and cond < 100.0)

        return {
            'success': success,
            'pos_error': pos_err,
            'ori_error': ori_err,
            'condition_number': cond,
            'joint_positions': solution,
        }


# ── Grid sweep + plotting ────────────────────────────────────────────────

def run_sweep(solver, x_range, y_range, z, pos_tol, ori_tol):
    """Run IK at every grid point. Returns 2D arrays of results."""
    nx, ny = len(x_range), len(y_range)
    success_map = np.zeros((ny, nx), dtype=bool)
    pos_err_map = np.full((ny, nx), np.nan)
    cond_map = np.full((ny, nx), np.nan)

    total = nx * ny
    t0 = time.time()
    done = 0

    for iy, y in enumerate(y_range):
        for ix, x in enumerate(x_range):
            r = solver.solve(x, y, z, pos_tol=pos_tol, ori_tol=ori_tol)
            success_map[iy, ix] = r['success']
            pos_err_map[iy, ix] = r['pos_error']
            cond_map[iy, ix] = r['condition_number']

            done += 1
            if done % 50 == 0 or done == total:
                elapsed = time.time() - t0
                rate = done / elapsed
                eta = (total - done) / rate if rate > 0 else 0
                print(f'  {done}/{total}  ({rate:.0f} pts/s, ETA {eta:.0f}s)')

    elapsed = time.time() - t0
    n_ok = int(success_map.sum())
    print(f'\nDone: {n_ok}/{total} reachable ({100*n_ok/total:.1f}%) in {elapsed:.1f}s')
    return success_map, pos_err_map, cond_map


def plot_results(success_map, pos_err_map, cond_map,
                 x_range, y_range, arm, z, save_path=None):
    """Plot reachability heatmap and position error map."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    extent = [x_range[0], x_range[-1], y_range[0], y_range[-1]]

    # 1. Success map (binary)
    ax = axes[0]
    cmap_binary = LinearSegmentedColormap.from_list('rg', ['#d32f2f', '#4caf50'])
    ax.imshow(success_map.astype(float), origin='lower', extent=extent,
              cmap=cmap_binary, vmin=0, vmax=1, aspect='equal')
    ax.set_title(f'{arm.capitalize()} arm: Reachable (green) vs Not (red)\nz={z:.3f}m')
    ax.set_xlabel('X (base_link) [m]  ← right | left →')
    ax.set_ylabel('Y (base_link) [m]  ↓ forward | back ↑')
    ax.axhline(0, color='white', linewidth=0.5, alpha=0.5)
    ax.axvline(0, color='white', linewidth=0.5, alpha=0.5)

    # Mark arm shoulder position
    shoulder_x = -0.15 if arm == 'right' else 0.15
    ax.plot(shoulder_x, -0.12, 'w*', markersize=12, label='shoulder')
    ax.legend(loc='upper right', fontsize=8)

    # 2. Position error (continuous)
    ax = axes[1]
    pe = pos_err_map.copy()
    pe[~success_map] = np.nan  # only show reachable points
    im = ax.imshow(pe * 1000, origin='lower', extent=extent,
                   cmap='viridis_r', aspect='equal')
    ax.set_title('Position Error (mm)\n(reachable points only)')
    ax.set_xlabel('X (base_link) [m]')
    ax.set_ylabel('Y (base_link) [m]')
    plt.colorbar(im, ax=ax, label='mm')

    # 3. Condition number (log scale)
    ax = axes[2]
    cn = np.log10(np.clip(cond_map, 1, 1e6))
    im = ax.imshow(cn, origin='lower', extent=extent,
                   cmap='inferno', aspect='equal')
    ax.set_title('Jacobian Condition (log10)\nlower = further from singularity')
    ax.set_xlabel('X (base_link) [m]')
    ax.set_ylabel('Y (base_link) [m]')
    plt.colorbar(im, ax=ax, label='log10(cond)')

    plt.suptitle(f'Grasp Reachability Map — {arm} arm, z={z:.3f}m in base_link',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'Saved to {save_path}')
    else:
        plt.show()


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Generate grasp reachability heatmap for TidyBot2 arms')
    parser.add_argument('--arm', default='right', choices=['right', 'left', 'both'])
    parser.add_argument('--z', type=float, default=0.025,
                        help='Grasp height in base_link (m). Default: 0.025 (floor object)')
    parser.add_argument('--step', type=float, default=0.02,
                        help='Grid step size (m). Default: 0.02 (2cm)')
    parser.add_argument('--x-min', type=float, default=-0.35)
    parser.add_argument('--x-max', type=float, default=0.35)
    parser.add_argument('--y-min', type=float, default=-0.55)
    parser.add_argument('--y-max', type=float, default=-0.15)
    parser.add_argument('--pos-tol', type=float, default=0.01,
                        help='Position tolerance (m). Default: 0.01')
    parser.add_argument('--ori-tol', type=float, default=0.1,
                        help='Orientation tolerance (rad). Default: 0.1')
    parser.add_argument('--save', type=str, default=None,
                        help='Save plot to file instead of displaying')
    parser.add_argument('--model', type=str, default=None,
                        help='Path to MuJoCo XML model')
    args = parser.parse_args()

    # Find model
    if args.model:
        model_path = args.model
    else:
        model_path = str(Path(__file__).resolve().parent.parent.parent.parent.parent /
                         'simulation/assets/mujoco/tidybot_wx250s_bimanual.xml')

    print(f'Model: {model_path}')

    x_range = np.arange(args.x_min, args.x_max + args.step / 2, args.step)
    y_range = np.arange(args.y_min, args.y_max + args.step / 2, args.step)
    total = len(x_range) * len(y_range)

    arms = ['right', 'left'] if args.arm == 'both' else [args.arm]

    for arm in arms:
        print(f'\n{"="*50}')
        print(f'Sweeping {arm} arm: {len(x_range)}x{len(y_range)} = {total} points')
        print(f'  x: [{args.x_min:.2f}, {args.x_max:.2f}]  y: [{args.y_min:.2f}, {args.y_max:.2f}]  z: {args.z:.3f}')
        print(f'  step: {args.step:.3f}m  pos_tol: {args.pos_tol:.3f}m  ori_tol: {args.ori_tol:.2f}rad')
        print(f'{"="*50}')

        solver = OfflineIKSolver(model_path, arm=arm)
        success_map, pos_err_map, cond_map = run_sweep(
            solver, x_range, y_range, args.z,
            pos_tol=args.pos_tol, ori_tol=args.ori_tol)

        save_path = None
        if args.save:
            base, ext = Path(args.save).stem, Path(args.save).suffix
            save_path = f'{base}_{arm}{ext}' if args.arm == 'both' else args.save

        plot_results(success_map, pos_err_map, cond_map,
                     x_range, y_range, arm, args.z, save_path=save_path)


if __name__ == '__main__':
    main()
