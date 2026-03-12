#!/usr/bin/env python3
"""
Run AprilTag, Face Recognition, and YOLO detectors on a single image
and draw bounding boxes for each detector with different colors.
"""

import cv2
import numpy as np
import os
import sys

# Fix face_recognition_models import (needs pkg_resources which may be missing)
_models_dir = os.path.join(
    sys.prefix, 'lib', f'python{sys.version_info.major}.{sys.version_info.minor}',
    'site-packages', 'face_recognition_models', 'models')
if os.path.isdir(_models_dir):
    import types
    frm = types.ModuleType('face_recognition_models')
    frm.pose_predictor_model_location = lambda: os.path.join(
        _models_dir, 'shape_predictor_68_face_landmarks.dat')
    frm.pose_predictor_five_point_model_location = lambda: os.path.join(
        _models_dir, 'shape_predictor_5_face_landmarks.dat')
    frm.face_recognition_model_location = lambda: os.path.join(
        _models_dir, 'dlib_face_recognition_resnet_model_v1.dat')
    frm.cnn_face_detector_model_location = lambda: os.path.join(
        _models_dir, 'mmod_human_face_detector.dat')
    sys.modules['face_recognition_models'] = frm

# -- Colors (BGR) --
COLOR_APRILTAG = (0, 255, 0)     # Green
COLOR_FACE     = (255, 0, 0)     # Blue
COLOR_YOLO     = (0, 0, 255)     # Red

IMAGE_PATH = '/media/psf/Developer/GitHub/me326-team7/Ref Image.jpg'
OUTPUT_PATH = '/media/psf/Developer/GitHub/me326-team7/detections_output.jpg'


def run_apriltag(image_bgr):
    """Detect AprilTags and return list of (label, x1, y1, x2, y2, score)."""
    from pupil_apriltags import Detector
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    detector = Detector(families='tag36h11', nthreads=2,
                        quad_decimate=1.0, quad_sigma=0.0,
                        decode_sharpening=0.25)
    detections = detector.detect(gray)
    results = []
    for det in detections:
        corners = det.corners
        xs, ys = corners[:, 0], corners[:, 1]
        x1, x2 = int(np.min(xs)), int(np.max(xs))
        y1, y2 = int(np.min(ys)), int(np.max(ys))
        label = f'Tag {det.tag_id} ({det.decision_margin:.1f})'
        results.append((label, x1, y1, x2, y2))
    return results


def run_face(image_bgr):
    """Detect and recognize faces, return list of (label, x1, y1, x2, y2)."""
    import face_recognition
    import os

    KNOWN_FACES_DIR = '/media/psf/Developer/GitHub/me326-team7/ros2_ws/src/tidybot_bringup/face'
    TOLERANCE = 0.6

    # Load known faces from subdirectories
    known_encodings = []
    known_names = []
    for subdir in os.scandir(KNOWN_FACES_DIR):
        if not subdir.is_dir() or subdir.name.startswith('.'):
            continue
        person = subdir.name
        for entry in os.scandir(subdir.path):
            if not entry.is_file():
                continue
            if os.path.splitext(entry.name)[1].lower() not in {'.jpg', '.jpeg', '.png'}:
                continue
            img = face_recognition.load_image_file(entry.path)
            encs = face_recognition.face_encodings(img)
            if encs:
                known_encodings.append(encs[0])
                known_names.append(person)

    print(f'  Loaded {len(known_encodings)} reference faces for: {sorted(set(known_names))}')

    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    locations = face_recognition.face_locations(rgb, model='hog')

    results = []
    if not locations:
        return results

    encodings = face_recognition.face_encodings(rgb, locations)
    for (top, right, bottom, left), enc in zip(locations, encodings):
        if known_encodings:
            distances = face_recognition.face_distance(known_encodings, enc)
            best_idx = int(np.argmin(distances))
            best_dist = float(distances[best_idx])
            if best_dist <= TOLERANCE:
                name = known_names[best_idx]
                conf = 1.0 - best_dist
            else:
                name = 'unknown'
                conf = 1.0 - best_dist
            label = f'{name} ({conf:.2f})'
        else:
            label = 'Face'
        results.append((label, left, top, right, bottom))
    return results


def run_yolo(image_bgr):
    """Detect objects with YOLO and return list of (label, x1, y1, x2, y2)."""
    from ultralytics import YOLO
    model = YOLO('yolov8n.pt')
    results_yolo = model(image_bgr, conf=0.05, verbose=False)
    results = []
    for r in results_yolo:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            cls_name = model.names[cls_id]
            if cls_name != 'banana':
                continue
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            label = f'{cls_name} {conf:.2f}'
            results.append((label, x1, y1, x2, y2))
    return results


def draw_boxes(image, detections, color, thickness=3, font_scale=0.7):
    """Draw bounding boxes with labels on the image."""
    for (label, x1, y1, x2, y2) in detections:
        cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
        # Background for text
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                       font_scale, 2)
        cv2.rectangle(image, (x1, y1 - th - 10), (x1 + tw + 4, y1),
                      color, -1)
        cv2.putText(image, label, (x1 + 2, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)


def main():
    image = cv2.imread(IMAGE_PATH)
    if image is None:
        print(f'ERROR: Could not load image: {IMAGE_PATH}')
        return

    print(f'Image loaded: {image.shape[1]}x{image.shape[0]}')
    output = image.copy()

    # AprilTag
    print('Running AprilTag detector...')
    try:
        apriltag_dets = run_apriltag(image)
        print(f'  AprilTags found: {len(apriltag_dets)}')
        draw_boxes(output, apriltag_dets, COLOR_APRILTAG)
    except Exception as e:
        print(f'  AprilTag error: {e}')

    # Face
    print('Running Face detector...')
    try:
        face_dets = run_face(image)
        print(f'  Faces found: {len(face_dets)}')
        draw_boxes(output, face_dets, COLOR_FACE)
    except Exception as e:
        print(f'  Face detection error: {e}')

    # YOLO
    print('Running YOLO detector...')
    try:
        yolo_dets = run_yolo(image)
        print(f'  YOLO objects found: {len(yolo_dets)}')
        draw_boxes(output, yolo_dets, COLOR_YOLO)
    except Exception as e:
        print(f'  YOLO error: {e}')

    # Draw legend
    y = 30
    for label, color in [('Green = AprilTag', COLOR_APRILTAG),
                          ('Blue = Face', COLOR_FACE),
                          ('Red = YOLO', COLOR_YOLO)]:
        cv2.putText(output, label, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        y += 35

    cv2.imwrite(OUTPUT_PATH, output)
    print(f'\nOutput saved to: {OUTPUT_PATH}')


if __name__ == '__main__':
    main()
