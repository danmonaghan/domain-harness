"""BBOS stereo/pose contract helpers; no device access or motor commands."""
from collections import OrderedDict
import hashlib
import json
import math
import threading
from pathlib import Path
import numpy as np

CAMERA_FRAME = 'bracketbot_head_rectified_optical'
RIGHT_CAMERA_FRAME = 'bracketbot_head_right_rectified_optical'
CALIBRATION_PREFIX = 'BBOS calibrated stereo RGB-D; sha256:'
MAX_POINTS = 384
MAX_AGE_NS = 800_000_000


def head_camera_specs(context, width, height, eyes):
    """Camera/rigid-mount contracts; independent of SLAM and depth availability."""
    specs = []
    for camera_id, k, calibrated in eyes:
        frame = dict(context['camera_frame_definition'])
        frame['frame_id'] = calibrated['sensor_frame']
        contract = dict(from_frame=frame, to_frame=context['base_frame_definition'],
                        clock=context['clock'], session_id=context['session_id'])
        specs.append(dict(id=camera_id+'.pose', clock=context['clock'],
            frame_definition=context['base_frame_definition'],
            payload=dict(kind='structured', datatype='auki.odometry-pose',
                schema='auki.odometry-pose/v1', modality='pose', observes=frame['frame_id'], unit=None),
            metadata=dict(pose_contract=contract, source='BBOS configured rigid head-camera mount')))
        specs.append(dict(id=camera_id, clock=context['clock'], frame_definition=frame,
            payload=dict(kind='camera', datatype='video_frame', schema='auki.video-frame/v1',
                encoding='jpeg', width=width, height=height, nominal_frame_rate_hz=5,
                observes=f'Bracketbot rectified head camera ({camera_id})'),
            metadata=dict(calibration=dict(width=width, height=height, K=k.reshape(-1).tolist(),
                D=[0., 0., 0., 0., 0.], distortion_model='plumb_bob'),
                pose_product_id=camera_id+'.pose-history', image_geometry=frame['frame_id'],
                calibration_source=calibrated['calibration_source'],
                timestamp_basis='BBOS capture timestamp; device-local; clock jumps require new session')))
    return specs


def wrist_camera_spec(context, side):
    if side not in ('left', 'right'):
        raise ValueError('unknown wrist camera')
    frame = dict(context['camera_frame_definition'])
    frame['frame_id'] = f'bracketbot_{side}_wrist_optical'
    return dict(id=f'camera.{side}', clock=context['clock'], frame_definition=frame,
                payload=dict(kind='camera', datatype='video_frame', schema='auki.video-frame/v1',
                             encoding='jpeg', width=640, height=480, nominal_frame_rate_hz=5,
                             observes=f'Bracketbot {side} wrist camera'),
                metadata=dict(source=f'BBOS camera.{side}.jpeg',
                              image_geometry='raw_uncalibrated', calibration_status='unavailable',
                              timestamp_basis='BBOS capture timestamp; device-local'))


def fresh_jpeg(record, now_ns):
    """Copy one fresh frame; never republish a frozen producer as live video."""
    if not -100_000_000 <= now_ns-timestamp(record) <= MAX_AGE_NS:
        return None
    count = int(record['jpeg_len'])
    if not 0 < count <= len(record['jpeg']):
        raise ValueError('invalid JPEG length')
    return record['jpeg'][:count].tobytes()


class NewSessionRequired(RuntimeError):
    """A source changed its clock or map frame; start a new runner session."""


def timestamp(record):
    return int(record['timestamp'].view('i8'))


def rotation_quaternion(r):
    r = np.asarray(r, dtype=float)
    if r.shape != (3, 3) or not np.isfinite(r).all():
        raise ValueError('invalid camera rotation')
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or abs(np.linalg.det(r)-1) > 1e-5:
        raise ValueError('camera extrinsics must be a rigid transform')
    # Symmetric quaternion eigenproblem, including rotations near 180 degrees.
    xx, xy, xz = r[0]; yx, yy, yz = r[1]; zx, zy, zz = r[2]
    k = np.array([[xx-yy-zz, yx+xy, zx+xz, yz-zy],
                  [yx+xy, yy-xx-zz, zy+yz, zx-xz],
                  [zx+xz, zy+yz, zz-xx-yy, xy-yx],
                  [yz-zy, zx-xz, xy-yx, xx+yy+zz]]) / 3
    _, vectors = np.linalg.eigh(k)
    # The eigenproblem above describes the inverse rotation.
    q = vectors[:, -1] * np.array([-1., -1., -1., 1.])
    if q[3] < 0: q = -q
    return q.tolist()


def right_camera_transform(left_to_base, r1, r2, rotation, translation_m):
    """OpenCV stereo X_right = R X_left + T, with both rectifications applied."""
    left_to_base = np.asarray(left_to_base, float).reshape(3, 4)
    r1, r2, rotation = [np.asarray(r, float).reshape(3, 3) for r in (r1, r2, rotation)]
    t = np.asarray(translation_m, float).reshape(3)
    for r in (r1, r2, rotation, left_to_base[:, :3]):
        rotation_quaternion(r)
    if not np.isfinite(t).all() or not .001 < np.linalg.norm(t) < 1:
        raise ValueError('invalid metric stereo baseline')
    right_to_left = r1 @ rotation.T @ r2.T
    center = -r1 @ rotation.T @ t
    return np.column_stack((left_to_base[:, :3] @ right_to_left,
                            left_to_base[:, :3] @ center + left_to_base[:, 3]))


def calibration(p1, camera_to_base, width, height, source_width, source_height, source_bytes,
                sensor_frame=CAMERA_FRAME):
    k = np.asarray(p1, dtype=float)[:3, :3].copy()
    k[0] *= width/source_width
    k[1] *= height/source_height
    transform = np.asarray(camera_to_base, dtype=float).reshape(3, 4)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0 or not np.isfinite(transform).all():
        raise ValueError('invalid stereo intrinsics/extrinsics')
    q = rotation_quaternion(transform[:, :3])
    digest = hashlib.sha256(source_bytes + json.dumps({
        'K': k.tolist(), 'camera_to_base': transform.tolist(),
        'width': width, 'height': height,
    }, sort_keys=True).encode()).hexdigest()
    return k, {
        'sensor_frame': sensor_frame,
        'sensor_to_base': {'translation': transform[:, 3].tolist(), 'rotation_xyzw': q},
        'calibration_source': CALIBRATION_PREFIX + digest,
    }


def optical_points(points_base, camera_to_base):
    """BBOS already applied this calibrated rectified-optical -> base transform."""
    transform = np.asarray(camera_to_base, dtype=float).reshape(3, 4)
    return (np.asarray(points_base, dtype=float)-transform[:, 3]) @ transform[:, :3]


def sampled_points(points, colors, capture_ns):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    colors = np.asarray(colors)
    if colors.shape != points.shape or colors.dtype != np.uint8:
        raise ValueError('point/color shape or RGB encoding mismatch')
    keep = (np.isfinite(points).all(axis=1) & (points[:, 2] >= .2)
            & (points[:, 2] <= 5.) & (np.sum(points*points, axis=1) <= 25.))
    points, colors = points[keep], colors[keep]
    if len(points) > MAX_POINTS:
        stride = math.ceil(len(points)/MAX_POINTS)
        offset = (capture_ns//1_000_000) % stride
        points, colors = points[offset::stride][:MAX_POINTS], colors[offset::stride][:MAX_POINTS]
    return points.tolist(), colors.tolist()


def pose(record):
    # BBOS mapping consumes pos/quat as world_from_base in X-right/Y-forward/Z-up.
    # Corrections are explicit mapper barriers, not runner/session failures.
    p = np.asarray(record['pos'], dtype=float)
    q = np.asarray(record['quat'], dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('invalid BBOS pose')
    norm = np.linalg.norm(q)
    if abs(norm-1) > .001:
        raise ValueError('BBOS pose quaternion is not unit length')
    return {'translation': p.tolist(), 'rotation_xyzw': (q/norm).tolist()}


class PoseGate:
    def __init__(self):
        self.last_timestamp = 0
        self.pgo_count = None
        self.tracking = False
        self.correction = None

    def accept(self, record, health, now_ns):
        self.correction = None
        if health is None or record is None:
            self.tracking = False
            return None
        t, h = timestamp(record), timestamp(health)
        if (not -100_000_000 <= now_ns-t <= MAX_AGE_NS
                or not -100_000_000 <= now_ns-h <= MAX_AGE_NS
                # The SLAM daemon publishes this pose only when pose_valid is
                # true. BBOS's own mapper gates on lost/localized, not degraded;
                # retain that quality warning in status without inventing loss.
                or any(bool(health[key]) for key in ('vo_lost', 'stalled'))
                or not bool(health['localized'])):
            self.tracking = False
            return None
        if t < self.last_timestamp:
            raise NewSessionRequired('SLAM capture clock regressed')
        pgo = int(record['pgo_count'])
        if pgo < 0 or (self.pgo_count is not None and pgo < self.pgo_count):
            raise NewSessionRequired('SLAM pose graph reset; new map session required')
        if t == self.last_timestamp:
            return None
        result = pose(record)
        # Ordinary PGO is part of BBOS's live pose stream, not a reset of our
        # frame or a request to rewrite previously posed measurements.
        self.pgo_count = pgo
        self.tracking = True
        self.last_timestamp = t
        return t, result


POSE_HISTORY_DTYPE = np.dtype([('ts_ns','<i8'),('pos','<f4',3),('quat','<f4',4),('pad','u1',4)])


def corrected_portal_poses(raw, expected_pgo, evidence):
    """Interpolate optimized keyframes at the original detector captures.

    No extrapolation, no nearest-pose substitution, no new detector observation.
    Missing/incompatible history deliberately leaves calibration unavailable.
    """
    if len(raw) < 40 or raw[:8] != b'BBPOSE1\0': return {}
    size, count, generation, pgo = np.frombuffer(raw[8:40], '<i8').tolist()
    if size != POSE_HISTORY_DTYPE.itemsize or not 0 < count <= 1000000 or pgo != expected_pgo:
        return {}
    if len(raw) != 40+size*count: return {}
    rows = np.frombuffer(raw[40:], POSE_HISTORY_DTYPE)
    stamps = rows['ts_ns']
    if np.any(stamps[1:] <= stamps[:-1]): return {}
    corrected = {}
    for identity, original in evidence.items():
        t = original.get('capture_timestamp_ns')
        if not isinstance(t,int) or isinstance(t,bool) or not original.get('base_odometry_pose'): continue
        at = int(np.searchsorted(stamps,t))
        if at < len(rows) and stamps[at] == t:
            p = pose(rows[at])
        elif 0 < at < len(rows) and int(stamps[at]-stamps[at-1]) <= 1_000_000_000:
            left, right = pose(rows[at-1]), pose(rows[at])
            f = (t-int(stamps[at-1]))/int(stamps[at]-stamps[at-1])
            q0, q1 = np.array(left['rotation_xyzw']), np.array(right['rotation_xyzw'])
            dot = float(np.dot(q0,q1))
            if dot < 0: q1 = -q1; dot = -dot
            if dot > .9995: q = (1-f)*q0+f*q1
            else:
                theta=math.acos(min(1.,dot))
                q=(math.sin((1-f)*theta)*q0+math.sin(f*theta)*q1)/math.sin(theta)
            q /= np.linalg.norm(q)
            p=dict(translation=((1-f)*np.array(left['translation'])+f*np.array(right['translation'])).tolist(),rotation_xyzw=q.tolist())
        else: continue
        corrected[identity] = dict(timestamp_ns=t,pose=p)
    return corrected


def trajectory_correction(correction, control_path, history_path):
    try:
        control = json.loads(Path(control_path).read_text())
        evidence = control['registries']['local_mapping']['observed_portals']
        raw = Path(history_path).read_bytes()
        ready = (len(raw)>=40 and raw[:8]==b'BBPOSE1\0'
                 and int(np.frombuffer(raw[8:40],'<i8')[-1]) == correction['pgo_count'])
        if not ready: return dict(correction,history_ready=False)
        size,count,_,_=np.frombuffer(raw[8:40],'<i8').tolist()
        if size!=POSE_HISTORY_DTYPE.itemsize or not 0<count<=1000000 or len(raw)!=40+size*count:
            return dict(correction,history_ready=False)
        rows=np.frombuffer(raw[40:],POSE_HISTORY_DTYPE)
        times=control['registries']['local_mapping'].get('retained_depth_timestamps',[])[:2048]
        targets=times+[v['capture_timestamp_ns'] for v in evidence.values()]
        if targets and int(rows['ts_ns'][-1])<max(targets):
            return dict(correction,history_ready=False)
        corrected = corrected_portal_poses(raw, correction['pgo_count'], evidence)
        depth_evidence = {str(t):dict(capture_timestamp_ns=t,base_odometry_pose=True) for t in times}
        depth = corrected_portal_poses(raw, correction['pgo_count'], depth_evidence)
        return dict(correction,kind='trajectory_correction',corrected_portals=corrected,
                    corrected_depth={key:value['pose'] for key,value in depth.items()},history_ready=True)
    except (OSError,KeyError,ValueError,TypeError):
        return dict(correction,history_ready=False)


class FramePairs:
    """Bounded exact-capture join; never pair the nearest RGB and depth frames."""
    def __init__(self, capacity=16):
        self.capacity = capacity
        self.frames = OrderedDict()

    def add(self, kind, capture_ns, value):
        row = self.frames.setdefault(capture_ns, {})
        row[kind] = value
        while len(self.frames) > self.capacity:
            self.frames.popitem(last=False)

    def take(self, after_ns):
        candidates = [t for t, row in self.frames.items()
                      if t > after_ns and {'rgb', 'points'} <= row.keys()]
        if not candidates:
            return None
        t = max(candidates)
        row = self.frames[t]
        for old in list(self.frames):
            if old <= t: del self.frames[old]
        return t, row['rgb'], row['points']


class CaptureMailbox:
    """One acquisition owner; bounded copied frames shared with a slow image worker.

    Poses bypass this mailbox entirely. Image processing must never throttle
    the reader that retains capture-time pose brackets.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.pairs = FramePairs(48)
        self.previews = {}
        self.tracking = False
        self.boundary = 0
        self.counts = dict(poses=0, matched_scans=0, emitted_scans=0, stale_scans=0,
                           rejected_after_processing=0)
        self.last_pose = 0
        self.max_pose_gap_ns = 0
        self.last_emitted_scan = 0
        self.health = None
        self.pgo_count = None

    def image(self, camera, t, raw):
        with self.lock:
            self.previews[camera] = (t, raw)
            if camera == 'camera.head':
                self.pairs.add('rgb', t, raw)

    def points(self, t, points, colors):
        with self.lock:
            self.pairs.add('points', t, (points, colors))

    def update_pose(self, gate, admitted, health):
        with self.lock:
            self.tracking = gate.tracking
            self.health = health
            self.pgo_count = gate.pgo_count
            if admitted:
                t = admitted[0]
                if self.last_pose:
                    self.max_pose_gap_ns = max(self.max_pose_gap_ns, t-self.last_pose)
                self.last_pose = t
                self.counts['poses'] += 1
            if gate.correction:
                self.boundary = gate.correction['timestamp_ns']
                self.pairs = FramePairs(48)

    def pause_for_correction(self,t):
        with self.lock:
            self.tracking=False
            self.boundary=max(self.boundary,t)
            self.pairs=FramePairs(48)

    def take_scan(self, after, now_ns):
        with self.lock:
            if not self.tracking:
                return None
            pair = self.pairs.take(max(after, self.boundary-1))
            if pair:
                self.counts['matched_scans'] += 1
                if not -100_000_000 <= now_ns-pair[0] <= MAX_AGE_NS:
                    self.counts['stale_scans'] += 1
                    return None
            return pair

    def scan_is_current(self, t, now_ns):
        with self.lock:
            current = self.tracking and t >= self.boundary and -100_000_000 <= now_ns-t <= MAX_AGE_NS
            if not current:
                self.counts['rejected_after_processing'] += 1
            return current

    def emitted(self, t):
        with self.lock:
            self.counts['emitted_scans'] += 1
            self.last_emitted_scan = t

    def snapshot(self):
        with self.lock:
            return dict(previews=dict(self.previews), tracking=self.tracking,
                        health=self.health, pgo_count=self.pgo_count,
                        **self.counts, last_pose=self.last_pose,
                        max_pose_gap_ms=self.max_pose_gap_ns/1e6,
                        last_emitted_scan=self.last_emitted_scan)
