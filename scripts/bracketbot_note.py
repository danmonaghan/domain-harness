"""Operator-selected fixed Home note, transformed into the robot's native base."""
import math
from bracketbot_behavior import alignment_valid
from bracketbot_pointing import fresh,inverse_point,vector,point_in_frame


def select_note(scene,request,hands,home,*,scene_age_ms,robot_pose_age_ms,arm_age_ms):
    if not all((fresh(scene_age_ms,750),fresh(robot_pose_age_ms,800),fresh(arm_age_ms,250))):
        raise ValueError('robot or arm feedback unavailable')
    if not (request['schema']=='spatial.robot-reach-note/v1'
            and request['authority_peer_id']==home['peer_id']
            and request['frame'] in (home['frame'],scene['frame'])
            and request['request_id'] and request['note']['id']
            and request['note']['text'].strip().casefold()==request['requested_name'].strip().casefold()
            and alignment_valid(scene,home['frame'])):
        raise ValueError('note/scene identity mismatch or robot not localized')
    note=request['note'];robot=scene['registries']['local_mapping']['robot_pose']
    if robot['from_frame']['frame_id']!='robot.base':raise ValueError('robot base frame mismatch')
    x,y,z,w=vector(robot['pose']['rotation_xyzw'],4)
    pose=dict(translation=robot['pose']['translation'],rotation_wxyz=[w,x,y,z])
    point=vector(inverse_point(pose,point_in_frame(note['position'],request['frame'],scene['frame'],scene['registries']['frame_transform']['transform'])))
    positions={s:vector(hands[s]) for s in ('left','right')}
    arm=min(positions,key=lambda s:(math.dist(point,positions[s]),s))
    delta=tuple(p-h for p,h in zip(point,positions[arm]));length=math.hypot(*delta)
    if length<.01:raise ValueError('note coincides with hand')
    return dict(status='target_selected',target_kind='note',target_id=note['id'],request_id=request['request_id'],
                target_name=note['text'],target_position_m=list(point),hand_position_m=list(positions[arm]),
                arm=arm,native_frame=scene['frame'],home_frame=home['frame'],frame='robot.base',
                direction=[v/length for v in delta],actuation_enabled=False)
