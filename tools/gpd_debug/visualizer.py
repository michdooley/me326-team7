#!/usr/bin/env python3
"""
GPD Debug Visualizer — Web-based 3D grasp viewer.

Loads debug captures from gpd_grasp_demo.py (save_debug:=true) and
displays point clouds + grasp candidates in an interactive Plotly 3D view.

Usage:
    python visualizer.py --data-dir ../../gpd_debug_captures
    # Opens http://localhost:5555
"""

import argparse
import json
import os
import sys

import numpy as np
from flask import Flask, jsonify, render_template, send_file

app = Flask(__name__)
DATA_DIR = None  # set by CLI


# ── Helpers ───────────────────────────────────────────────────────────

def list_sessions():
    """Return sorted list of session directory names."""
    if not os.path.isdir(DATA_DIR):
        return []
    sessions = [
        d for d in os.listdir(DATA_DIR)
        if os.path.isdir(os.path.join(DATA_DIR, d)) and d.startswith('session_')
    ]
    return sorted(sessions, reverse=True)


def list_captures(session):
    """Return sorted list of capture directory names within a session."""
    session_dir = os.path.join(DATA_DIR, session)
    if not os.path.isdir(session_dir):
        return []
    captures = [
        d for d in os.listdir(session_dir)
        if os.path.isdir(os.path.join(session_dir, d)) and d.startswith('capture_')
    ]
    return sorted(captures)


def load_pointcloud(session, capture, kind='point_cloud'):
    """Load a .npy point cloud and subsample for web transfer."""
    path = os.path.join(DATA_DIR, session, capture, f'{kind}.npy')
    if not os.path.isfile(path):
        return None
    pts = np.load(path)
    # Subsample large clouds
    max_pts = 20000 if kind == 'point_cloud_full' else 50000
    if len(pts) > max_pts:
        pts = pts[::len(pts) // max_pts]
    return pts


# ── Routes ────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/sessions')
def api_sessions():
    return jsonify(list_sessions())


@app.route('/api/session/<session>')
def api_session(session):
    captures = list_captures(session)
    # Load session-level meta if it exists
    meta_path = os.path.join(DATA_DIR, session, 'session_meta.json')
    meta = {}
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
    return jsonify({'captures': captures, 'meta': meta})


@app.route('/api/capture/<session>/<capture>')
def api_capture(session, capture):
    capture_dir = os.path.join(DATA_DIR, session, capture)

    # Load meta
    meta = {}
    meta_path = os.path.join(capture_dir, 'meta.json')
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)

    # Load grasps
    grasps = []
    grasps_path = os.path.join(capture_dir, 'grasps.json')
    if os.path.isfile(grasps_path):
        with open(grasps_path) as f:
            grasps = json.load(f)

    return jsonify({'meta': meta, 'grasps': grasps})


@app.route('/api/pointcloud/<session>/<capture>')
def api_pointcloud(session, capture):
    # Load cropped object cloud
    obj_pts = load_pointcloud(session, capture, 'point_cloud')
    # Load full scene cloud
    full_pts = load_pointcloud(session, capture, 'point_cloud_full')

    result = {}
    if obj_pts is not None:
        result['object'] = {
            'x': obj_pts[:, 0].tolist(),
            'y': obj_pts[:, 1].tolist(),
            'z': obj_pts[:, 2].tolist(),
        }
    if full_pts is not None:
        result['scene'] = {
            'x': full_pts[:, 0].tolist(),
            'y': full_pts[:, 1].tolist(),
            'z': full_pts[:, 2].tolist(),
        }
    return jsonify(result)


@app.route('/api/rgb/<session>/<capture>')
def api_rgb(session, capture):
    path = os.path.join(DATA_DIR, session, capture, 'rgb.png')
    if not os.path.isfile(path):
        return 'Not found', 404
    return send_file(path, mimetype='image/png')


@app.route('/api/depth/<session>/<capture>')
def api_depth(session, capture):
    """Serve depth as a colorized PNG for preview."""
    npy_path = os.path.join(DATA_DIR, session, capture, 'depth.npy')
    if not os.path.isfile(npy_path):
        return 'Not found', 404

    import cv2
    depth = np.load(npy_path)
    # Normalize to 0-255 for visualization
    valid = depth[depth > 0]
    if len(valid) == 0:
        return 'Empty depth', 404
    d_min, d_max = np.percentile(valid, [2, 98])
    depth_norm = np.clip((depth.astype(float) - d_min) / (d_max - d_min), 0, 1)
    depth_u8 = (depth_norm * 255).astype(np.uint8)
    colored = cv2.applyColorMap(depth_u8, cv2.COLORMAP_VIRIDIS)
    # Mark invalid pixels as black
    colored[depth == 0] = 0

    tmp_path = os.path.join(DATA_DIR, session, capture, '_depth_viz.png')
    cv2.imwrite(tmp_path, colored)
    return send_file(tmp_path, mimetype='image/png')


# ── Main ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='GPD Debug Visualizer')
    parser.add_argument('--data-dir', required=True,
                        help='Path to gpd_debug_captures/ directory')
    parser.add_argument('--port', type=int, default=5555)
    parser.add_argument('--host', default='0.0.0.0')
    args = parser.parse_args()

    global DATA_DIR
    DATA_DIR = os.path.abspath(args.data_dir)

    if not os.path.isdir(DATA_DIR):
        print(f'Error: {DATA_DIR} does not exist', file=sys.stderr)
        sys.exit(1)

    sessions = list_sessions()
    print(f'Found {len(sessions)} session(s) in {DATA_DIR}')
    for s in sessions[:5]:
        caps = list_captures(s)
        print(f'  {s}: {len(caps)} capture(s)')

    print(f'\nStarting server at http://localhost:{args.port}')
    app.run(host=args.host, port=args.port, debug=True)


if __name__ == '__main__':
    main()
