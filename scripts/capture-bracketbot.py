#!/usr/bin/env python3
"""Read existing BBOS sensors into one runner; acquires no motor/control writers."""
import argparse
import base64
import contextlib
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
from bracketbot_capture import trajectory_correction
from bracketbot_capture import (CAMERA_FRAME, RIGHT_CAMERA_FRAME, CaptureMailbox, NewSessionRequired, PoseGate,
                               calibration, fresh_jpeg, optical_points, sampled_points, timestamp,
                               wrist_camera_spec, right_camera_transform, head_camera_specs)


def acquire(Reader, mailbox, emit, stopped, parent):
    """Only copy sensors and publish dense poses here; no OpenCV or point transforms."""
    gate = PoseGate()
    pending_correction = None
    latest_pose = latest_health = None
    last_seen, writers = {}, {}
    clock_offset = time.time_ns()-time.monotonic_ns()
    topics = ['slam.health', 'slam.pose', 'camera.head.jpeg', 'camera.points',
              'camera.left.jpeg', 'camera.right.jpeg']
    with contextlib.ExitStack() as stack:
        # Wheel positions tell a restarted mapper whether the robot stood still.
        # Optional: the base daemon may restart or be absent without ending capture.
        wheels = last_wheels = None
        try:
            wheels = Reader('drive.state', keeptime=False)
            wheels.__enter__()
            stack.callback(wheels.__exit__, None, None, None)
        except Exception:
            wheels = None
        readers = {}
        for name in topics:
            reader = Reader(name, keeptime=False)
            reader.__enter__()
            # BBOS Reader.__exit__ returns True; do not let it swallow failures.
            stack.callback(reader.__exit__, None, None, None)
            readers[name] = reader
        while not stopped.is_set() and os.getppid() == parent:
            if abs(time.time_ns()-time.monotonic_ns()-clock_offset) > 250_000_000:
                raise NewSessionRequired('BBOS wall clock changed; new capture session required')
            for name, reader in readers.items():
                path = Path('/dev/shm') / name
                try:
                    stat = path.stat()
                    with path.open('rb') as f:
                        f.seek(8); owner = int.from_bytes(f.read(4), 'little')
                    identity = (stat.st_ino, owner)
                    if name in writers and writers[name] != identity:
                        raise NewSessionRequired(name+' producer restarted')
                    writers[name] = identity
                except FileNotFoundError:
                    continue
                if not reader.ready(): continue
                d = reader.data
                t = timestamp(d)
                if t == last_seen.get(name): continue
                if t < last_seen.get(name, 0): raise NewSessionRequired(name+' timestamp regressed')
                last_seen[name] = t
                if name == 'slam.health': latest_health = d.copy()
                elif name == 'slam.pose': latest_pose = d.copy()
                elif name == 'camera.points':
                    n = int(d['num_points'])
                    if not 0 <= n <= len(d['points']): raise ValueError('invalid BBOS point count')
                    mailbox.points(t, d['points'][:n].copy(), d['colors'][:n].copy())
                else:
                    raw = fresh_jpeg(d, time.time_ns())
                    if raw is not None:
                        mailbox.image(name.removesuffix('.jpeg'), t, raw)
            was_tracking = gate.tracking
            admitted = gate.accept(latest_pose, latest_health, time.time_ns())
            # Close the image admission boundary before publishing a correction.
            mailbox.update_pose(gate, admitted, latest_health)
            if admitted:
                t, pose = admitted
                emit(dict(kind='odometry',timestamp_ns=t,pose=pose))
                try:
                    if wheels is not None and wheels.ready() and t-(last_wheels or 0) >= 250_000_000:
                        positions = [float(v) for v in wheels.data['pos']]
                        if all(math.isfinite(v) for v in positions):
                            emit(dict(kind='wheels', positions=positions))
                            last_wheels = t
                except Exception:
                    wheels = None
            elif was_tracking and not gate.tracking and gate.last_timestamp:
                emit(dict(kind='tracking_lost',timestamp_ns=gate.last_timestamp+1))
            stopped.wait(.015)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=0, help='Bounded capture for validation')
    args = ap.parse_args()
    output = os.fdopen(os.dup(1), 'w', buffering=1)
    os.dup2(2, 1)  # BBOS/native diagnostics stay off the framed observation pipe.
    import cv2
    from bbos import Config, Reader
    cv2.setNumThreads(1)
    context = json.loads(os.environ['SPATIAL_CAPTURE_CONTEXT'])
    # A robot that also runs navigation can trade preview and scan rate for CPU.
    preview_interval_ns = max(100, int(os.environ.get('SPATIAL_PREVIEW_INTERVAL_MS', '200')))*1_000_000
    scan_interval = max(100, int(os.environ.get('SPATIAL_SCAN_INTERVAL_MS', '500')))/1000
    cfg = Config('depth')
    if getattr(cfg, 'imu_pitch', 0):
        raise ValueError('Dynamic depth extrinsics require a capture-time transform stream')
    source = Path(cfg.calib_path)
    source_bytes = source.read_bytes()
    signature = hashlib.sha256(source_bytes).hexdigest()
    k_left, d_left, k_right, d_right, r_left, r_right, p1, p2, _, _, _, stereo_r, stereo_t = cfg.camera_cal()
    width, height = int(cfg.input_width)//2, int(cfg.input_height)
    # Keep the camera's native per-eye resolution for the 15 cm Portal. Depth
    # points use the same rectified optical frame at their lower raster size.
    maps = cv2.fisheye.initUndistortRectifyMap(k_left, d_left[:4].reshape(4, 1),
        r_left, p1, (width, height), cv2.CV_32FC1)
    right_maps = cv2.fisheye.initUndistortRectifyMap(k_right, d_right[:4].reshape(4, 1),
        r_right, p2, (width, height), cv2.CV_32FC1)
    def decode(jpeg):
        full = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if full is None or full.shape != (height, width*2, 3):
            raise NewSessionRequired('head stereo camera shape changed')
        return full
    def rectify(full, right=False):
        left = cv2.remap(full[:, width:] if right else full[:, :width],
                         *(right_maps if right else maps), cv2.INTER_LINEAR)
        return cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
    transform = np.asarray(cfg.camera_to_base_3x4, dtype=float).reshape(3, 4)
    k, calibrated = calibration(p1, transform, width, height,
                               cfg.input_width//2, cfg.input_height, source_bytes)
    right_transform = right_camera_transform(transform, r_left, r_right, stereo_r, stereo_t)
    right_k, right_calibrated = calibration(p2, right_transform, width, height,
        cfg.input_width//2, cfg.input_height, source_bytes, RIGHT_CAMERA_FRAME)
    eyes = [('camera.head', k, calibrated), ('camera.head.right', right_k, right_calibrated)]
    head_specs = head_camera_specs(context, width, height, eyes)
    pose_contracts = {s['id']: s['metadata']['pose_contract'] for s in head_specs
                      if s['id'].endswith('.pose')}
    output_lock = threading.Lock()
    def emit(value):
        line = 'SENSOR '+json.dumps(value, separators=(',', ':'), allow_nan=False)+'\n'
        with output_lock:
            output.write(line)
    def sample(sensor_id, t, data='', raw=b''):
        emit(dict(kind='sensor_sample', sensor_id=sensor_id, timestamp_ns=t,
                  data=data, bytes=base64.b64encode(raw).decode()))
    for spec in head_specs:
        emit(dict(kind='register_sensor', spec=spec))
    for side in ('left', 'right'):
        emit(dict(kind='register_sensor', spec=wrist_camera_spec(context, side)))
    stopped = threading.Event()
    def stop(*_):
        stopped.set()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    parent = os.getppid()
    mailbox = CaptureMailbox()
    last_image = last_mapping = last_preview = last_status = 0
    right_eye = False
    wrist_preview = {}
    started = time.monotonic()
    failures = []
    def capture():
        try:
            acquire(Reader, mailbox, emit, stopped, parent)
        except BaseException as e:
            failures.append(e)
            stopped.set()
    worker = threading.Thread(target=capture, name='bbos-acquisition', daemon=True)
    worker.start()
    # Reuse the installed arm numerical runtime without mixing its packages into
    # the camera runtime. This worker opens only feedback Readers, never Writers.
    volume = subprocess.Popen(['/usr/bin/python3', str(Path(__file__).with_name('bracketbot-python.py')),
        '--daemon', 'arm_left', str(Path(__file__).with_name('capture-bracketbot-volume.py'))],
        stdout=subprocess.PIPE, text=True)
    def volume_output():
        for line in volume.stdout:
            if line.startswith('SENSOR '):
                with output_lock: output.write(line)
        if not stopped.is_set():
            print('Robot volume worker exited; camera/mapping capture continues', file=sys.stderr)
    volume_worker = threading.Thread(target=volume_output, name='body-volume', daemon=True)
    volume_worker.start()
    try:
        while not stopped.is_set() and os.getppid() == parent:
            monotonic = time.monotonic()
            if args.seconds and monotonic-started >= args.seconds: break
            if monotonic-last_mapping >= scan_interval:
                pair = mailbox.take_scan(last_image, time.time_ns())
                if pair:
                    t, raw, (points, colors) = pair
                    eye_transform = right_transform if right_eye else transform
                    eye_k = right_k if right_eye else k
                    eye_calibration = right_calibrated if right_eye else calibrated
                    hits, rgb = sampled_points(optical_points(points, eye_transform), colors, t)
                    gray = cv2.cvtColor(rectify(decode(raw), right_eye), cv2.COLOR_RGB2GRAY)
                    if mailbox.scan_is_current(t, time.time_ns()):
                        emit(dict(kind='camera', sample=dict(timestamp_ns=t, rgb_timestamp_ns=t,
                            transform_timestamp_ns=t, width=width, height=height,
                            fx=float(eye_k[0, 0]), fy=float(eye_k[1, 1]), cx=float(eye_k[0, 2]), cy=float(eye_k[1, 2]),
                            gray_base64=base64.b64encode(gray.tobytes()).decode(), calibration=eye_calibration,
                            hit_points_m=hits, hit_colors_srgb=rgb)))
                        mailbox.emitted(t)
                        right_eye = not right_eye
                    last_image, last_mapping = t, monotonic
            state = mailbox.snapshot()
            for name, (t, raw) in state['previews'].items():
                if not -100_000_000 <= time.time_ns()-t <= MAX_CAMERA_AGE_NS: continue
                if name != 'camera.head':
                    if t-wrist_preview.get(name, 0) >= preview_interval_ns:
                        sample(name, t, raw=raw)
                        wrist_preview[name] = t
                elif t-last_preview >= preview_interval_ns:
                    full = decode(raw)
                    for index, (camera_id, _, eye_calibration) in enumerate(eyes):
                        image = rectify(full, right=index == 1)
                        ok, jpeg = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
                        if not ok: raise ValueError('JPEG encoding failed')
                        sample(camera_id+'.pose', t, json.dumps(dict(contract=pose_contracts[camera_id+'.pose'],
                            tracking=dict(status='valid', pose=eye_calibration['sensor_to_base']))))
                        sample(camera_id, t, raw=jpeg.tobytes())
                    last_preview = t
            if monotonic-last_status >= 1:
                if hashlib.sha256(source.read_bytes()).hexdigest() != signature:
                    raise NewSessionRequired('stereo calibration changed')
                state = mailbox.snapshot()
                detail = ('BBOS stereo + fresh SLAM pose' if state['tracking'] else
                          'BBOS camera live; mapping paused: SLAM tracking lost, stale, or unavailable')
                latest_health = state['health']
                if latest_health is not None and bool(latest_health['degraded']):
                    detail += '; BBOS reports degraded tracking quality'
                age = ((time.time_ns()-state['last_emitted_scan'])//1_000_000
                       if state['last_emitted_scan'] else None)
                detail += (f"; pose-graph correction {state['pgo_count']}; poses {state['poses']}"
                           f"; paired scans {state['matched_scans']}; emitted scans {state['emitted_scans']}"
                           f"; stale scans {state['stale_scans']}; last scan age_ms {age}"
                           f"; rejected after processing {state['rejected_after_processing']}"
                           f"; max pose gap_ms {state['max_pose_gap_ms']:.0f}")
                emit(dict(kind='status', detail=detail))
                last_status = monotonic
            time.sleep(.005)
    finally:
        stopped.set()
        volume.terminate()
        try: volume.wait(timeout=5)
        except subprocess.TimeoutExpired:
            volume.kill(); volume.wait()
        volume_worker.join(timeout=5)
        worker.join(timeout=5)
    if worker.is_alive(): raise RuntimeError('sensor acquisition did not stop')
    if failures: raise failures[0]
    output.close()


MAX_CAMERA_AGE_NS = 800_000_000
if __name__ == '__main__':
    main()
