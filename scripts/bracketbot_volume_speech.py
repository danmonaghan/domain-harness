"""Pure volume-entry policy. No network, audio or motor operations."""
import math
from bracketbot_behavior import alignment_valid
from bracketbot_pointing import vector, fresh, point_in_frame


def transform_point(pose, point):
    w, x, y, z = vector(pose['rotation_wxyz'], 4)
    if abs(w*w+x*x+y*y+z*z-1) > 1e-6:
        raise ValueError('non-unit quaternion')
    px, py, pz = vector(point)
    rotated = ((1-2*(y*y+z*z))*px+2*(x*y-w*z)*py+2*(x*z+w*y)*pz,
               2*(x*y+w*z)*px+(1-2*(x*x+z*z))*py+2*(y*z-w*x)*pz,
               2*(x*z-w*y)*px+2*(y*z+w*x)*py+(1-2*(x*x+y*y))*pz)
    return tuple(p+t for p, t in zip(rotated, vector(pose['translation'])))


def contains(volume, point, margin=0):
    center, size = vector(volume['center']), vector(volume['size'])
    if any(s <= 0 for s in size): raise ValueError('invalid size')
    yaw = volume['yaw']
    if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
        raise ValueError('invalid yaw')
    dx, dy, dz = (p-c for p, c in zip(vector(point), center))
    c, s = math.cos(yaw), math.sin(yaw)
    return all(abs(p) <= extent/2+margin for p, extent in
               zip((c*dx-s*dz, dy, s*dx+c*dz), size))


def inputs(scene, packet, profile, *, now_ns, capture_now_ns, scene_age_ms):
    """Return Home-space base position and volumes, or None on stale/mismatched data."""
    try:
        frame = profile['frame']
        if not (packet['schema'] == 'spatial.robot-volume-input/v1'
                and packet['authenticated'] is True
                and packet['home_peer_id'] == profile['peer_id']
                and packet['home_frame'] == frame
                and packet['robot_peer_id'] == scene['authority_peer_id']
                and fresh((now_ns-packet['received_ns'])/1e6, 15000)
                and fresh(scene_age_ms, 500) and alignment_valid(scene, frame)):
            return None
        mapping = scene['registries']['local_mapping']
        robot = mapping['robot_pose']
        if not (mapping['status'] == 'mapping'
                and robot['from_frame']['frame_id'] == 'robot.base'
                and fresh((capture_now_ns-robot['capture_timestamp_ns'])/1e6, 800)):
            return None
        volumes = packet['volumes']
        if volumes is None: return (None, [])
        if volumes['schema'] != 'spatial-harness.semantic-volumes/v1' or volumes['frame'] not in (frame,scene['frame']):
            return None
        # Compare in the collection's declared frame, never assume Home space.
        point = point_in_frame(robot['pose']['translation'],scene['frame'],volumes['frame'],scene['registries']['frame_transform']['transform'])
        return point, volumes['items']
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


class Entries:
    """Half-second dwell and 5 cm exit hysteresis; tracking loss is not an exit."""
    def __init__(self):
        self.inside = set()
        self.since = {}
        self.scope = None

    def update(self, point, volumes, now, scope):
        if scope != self.scope:
            self.inside.clear(); self.since.clear(); self.scope = scope
        if point is None and volumes is None:
            self.since.clear()
            return []
        present = {v['id'] for v in volumes}
        self.inside.intersection_update(present)
        self.since = {k:t for k,t in self.since.items() if k in present}
        announcements = []
        for volume in sorted(volumes, key=lambda v: v['id']):
            identity = volume['id']
            if identity in self.inside:
                if not contains(volume, point, .05):
                    self.inside.remove(identity)
                continue
            if not contains(volume, point):
                self.since.pop(identity, None)
                continue
            entered = self.since.setdefault(identity, now)
            if now-entered >= .5:
                self.inside.add(identity); self.since.pop(identity, None)
                description = volume['description'].strip()
                if description: announcements.append((identity, description))
        return announcements
