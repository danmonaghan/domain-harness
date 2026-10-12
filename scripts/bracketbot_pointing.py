"""Pure phone/arm selection. Produces pointing intent, NEVER motor commands.

Inputs must come from authenticated, scene-scoped Home presence and the local
robot's published scene. Ages are local monotonic receipt ages; capture times
from different peers are never subtracted. Arm origins are measured hand
positions in robot.base, not guessed offsets or target joint positions.

An actuator must independently validate a collision-free swept path, joint and
speed limits, fresh feedback and control ownership before using this intent.
"""
import math


def vector(value, size=3):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError('invalid vector')
    if any(isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in value):
        raise ValueError('nonfinite vector')
    return tuple(value)


def inverse_point(pose, point):
    """Apply inverse of a validated rigid transform (wxyz quaternion)."""
    w, x, y, z = vector(pose['rotation_wxyz'], 4)
    if abs(w*w+x*x+y*y+z*z-1) > 1e-6:
        raise ValueError('non-unit quaternion')
    px, py, pz = [p-t for p, t in zip(point, vector(pose['translation']))]
    return (
        (1-2*(y*y+z*z))*px + 2*(x*y+w*z)*py + 2*(x*z-w*y)*pz,
        2*(x*y-w*z)*px + (1-2*(x*x+z*z))*py + 2*(y*z+w*x)*pz,
        2*(x*z+w*y)*px + 2*(y*z-w*x)*py + (1-2*(x*x+y*y))*pz,
    )


def fresh(age, limit):
    return (not isinstance(age, bool) and isinstance(age, (float, int))
            and math.isfinite(age) and 0 <= age <= limit)


def point_in_frame(point, source, target, transform):
    """Explicitly route a point through the last known robot/Home relationship."""
    point=vector(point)
    if not source or not target: raise ValueError('missing coordinate frame')
    if source==target: return point
    native,home=transform['from_frame_id'],transform['to_frame_id']
    if source==home and target==native: return inverse_point(transform,point)
    if source==native and target==home:
        w,x,y,z=vector(transform['rotation_wxyz'],4)
        inverse=dict(rotation_wxyz=[w,-x,-y,-z],translation=inverse_point(transform,[0,0,0]))
        return inverse_point(inverse,point)
    raise ValueError('unknown coordinate frame')


def select_target(scene, presence, hands, *, scene_age_ms, presence_age_ms,
                  arm_age_ms, robot_pose_age_ms, expected_home_frame, max_phone_age_ms=500):
    """Nearest fresh phone to robot origin, then nearest measured hand to phone.

    All robot ages must be supplied by the local adapter, including the age of
    robot_pose in its own capture clock. Fail closed; never retain old intent.
    Both hands are required to decide which arm is nearer. Ties are stable by
    peer ID, then arm name. A target is a direction to point, not a reach goal.
    """
    blocked = lambda reason: {'status': 'blocked', 'reason': reason, 'actuation_enabled': False}
    if not all((fresh(scene_age_ms, 750), fresh(presence_age_ms, max_phone_age_ms),
                fresh(arm_age_ms, 250), fresh(robot_pose_age_ms, 800))):
        return blocked('stale input')
    try:
        owner, native = scene['authority_peer_id'], scene['frame']
        registries = scene['registries']
        alignment = registries['frame_transform']
        product = registries['frame_transform_product']
        mapping = registries['local_mapping']
        robot = mapping['robot_pose']
        evidence = alignment['evidence']
        measured = mapping['observed_portals'][evidence['portal']]
        if not (alignment['schema'] == 'spatial.frame-transform/v1'
                and alignment['validity'] == 'valid'
                and alignment['authority_peer_id'] == owner
                and product['peer_id'] == owner and bool(product['manifest_hash'])
                and product['product_id'] == 'robot-to-scene-transform'
                and alignment['source_frame']['id'] == native
                and alignment['target_frame']['id'] == expected_home_frame
                and presence['frame'] in (expected_home_frame,native)
                and alignment['transform']['from_frame_id'] == native
                and alignment['transform']['to_frame_id'] == expected_home_frame
                and robot['frame'] == native and robot['from_frame']['frame_id'] == 'robot.base'
                and robot['clock']['peer_id'] == owner
                and evidence['clock'] == measured['clock'] == robot['clock']
                and evidence['capture_timestamp_ns'] == measured['capture_timestamp_ns']):
            return blocked('robot/Home alignment mismatch')
        # The runner represents robot pose using xyzw; transform Products use wxyz.
        x, y, z, w = vector(robot['pose']['rotation_xyzw'], 4)
        robot_pose = {'translation': robot['pose']['translation'], 'rotation_wxyz': [w, x, y, z]}
        hand_positions = {side: vector(hands[side]) for side in ('left', 'right')}
        candidates = []
        for phone in presence['poses']:
            try:
                peer = phone['peer_id']
                clock = phone['clock']
                pose = phone['pose']
                if not (isinstance(peer, str) and peer and peer != owner
                        and clock['peer_id'] == peer and bool(clock['session_id'])
                        and pose['from_frame_id'] == 'phone-camera:'+clock['session_id']
                        and pose['to_frame_id'] == presence['frame']
                        and fresh(phone['received_age_ms'] + presence_age_ms, max_phone_age_ms)):
                    continue
                point = vector(inverse_point(robot_pose, point_in_frame(pose['translation'],presence['frame'],native,alignment['transform'])))
                distance = math.hypot(*point)
                if not math.isfinite(distance):
                    continue
                candidates.append((distance, peer, point, phone['received_age_ms'] + presence_age_ms))
            except (KeyError, ValueError, TypeError, OverflowError):
                continue
        if not candidates:
            return blocked('no fresh localized phone')
        distance, peer, point, phone_age = min(candidates)
        arm = min(hand_positions, key=lambda side: (math.dist(point, hand_positions[side]), side))
        delta = vector(tuple(p-h for p, h in zip(point, hand_positions[arm])))
        length = math.hypot(*delta)
        if not math.isfinite(length) or length < .01:
            return blocked('phone too close to hand to define pointing direction')
        return {'status': 'target_selected', 'actuation_enabled': False,
                'target_kind':'phone','target_id':peer,'target_position_m':list(point),
                'phone_peer_id': peer, 'arm': arm, 'frame': 'robot.base',
                'native_frame': native, 'home_frame': expected_home_frame,
                'distance_from_robot_m': distance, 'phone_position_m': list(point),
                'phone_age_ms': phone_age,
                'hand_position_m': list(hand_positions[arm]),
                'direction': [v/length for v in delta]}
    except (KeyError, ValueError, TypeError, OverflowError):
        return blocked('invalid or unavailable robot state')
