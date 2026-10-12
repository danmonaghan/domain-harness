#!/usr/bin/env python3
"""Build the deployed runner's correction fix on the laptop, not the robot.

Fetches the current deployed source to preserve colleague changes, patches only
correction handling, then uses a persistent Linux ARM64 Docker build cache.
Does not install/restart anything or touch Home.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tarfile


def once(text, old, new):
    if text.count(old) != 1: raise ValueError('Unexpected deployed source anchor: '+old[:80])
    return text.replace(old,new,1)


def patch_sources(source, repo):
    files={}
    def read(name):
        if name not in files: files[name]=(source/name).read_text()
        return files[name]
    name='src/local_mapper.rs'; text=read(name)
    local=(repo/name).read_text()
    if '    TrajectoryCorrection {' not in text:
        variant='    TrajectoryCorrection {'+local.split('    TrajectoryCorrection {',1)[1].split('    Camera {',1)[0]
        text=once(text,'    Camera {',variant+'    Camera {')
        text=once(text,'pub fn identity() -> Pose {', '#[derive(Clone, Debug, Serialize, Deserialize)]\npub struct HistoricalPose { pub timestamp_ns:u64, pub pose:Pose }\n\npub fn identity() -> Pose {')
        branch='            SensorEvent::TrajectoryCorrection {'+local.split('            SensorEvent::TrajectoryCorrection {',1)[1].split('            SensorEvent::MapCorrection {',1)[0]
        text=once(text,'            SensorEvent::MapCorrection {',branch+'            SensorEvent::MapCorrection {')
        old=next(line for line in text.splitlines() if '"map_correction":self.correction.map' in line)
        lines=local.splitlines(); at=next(i for i,line in enumerate(lines) if '"map_correction":self.correction.map' in line)
        text=once(text,old,'\n'.join(lines[at:at+2]))
        text=once(text,'"latest_measured_pose":measured,"provenance":provenance','"latest_measured_pose":measured,"base_odometry_pose":base,"provenance":provenance')
        files[name]=text
    if 'retained_depth: Vec<RetainedDepth>' not in text:
        if 'corrected_depth: BTreeMap<u64, Pose>' not in text:
            text=once(text,'        corrected_portals: BTreeMap<String, HistoricalPose>,','        corrected_portals: BTreeMap<String, HistoricalPose>,\n        #[serde(default)]\n        corrected_depth: BTreeMap<u64, Pose>,')
        record='#[derive(Clone)]\nstruct RetainedDepth {'+local.split('#[derive(Clone)]\nstruct RetainedDepth {',1)[1].split('pub fn identity()',1)[0]
        text=once(text,'pub fn identity()',record+'pub fn identity()')
        text=once(text,'    recent_voxels: crate::recent_voxels::RecentVoxels,','    recent_voxels: crate::recent_voxels::RecentVoxels,\n    retained_depth: Vec<RetainedDepth>,')
        text=once(text,'            recent_voxels: Default::default(),','            recent_voxels: Default::default(),\n            retained_depth: Vec::new(),')
        text=once(text,'            "observed_portals":self.evidence,','            "observed_portals":self.evidence,\n            "retained_depth_timestamps":self.retained_depth.iter().map(|d|d.observation.timestamp_ns).collect::<std::collections::BTreeSet<_>>(),')
        begin=text.index('            SensorEvent::TrajectoryCorrection {'); end=text.index('            SensorEvent::MapCorrection {',begin)
        branch='            SensorEvent::TrajectoryCorrection {'+local.split('            SensorEvent::TrajectoryCorrection {',1)[1].split('            SensorEvent::MapCorrection {',1)[0]
        text=text[:begin]+branch+text[end:]
        text=once(text,'                self.recent_voxels = Default::default();','                self.recent_voxels = Default::default();\n                self.retained_depth.clear();')
        text=once(text,'recent.observe(&update, &endpoints)?','recent.observe_at(&update, &endpoints, t)?')
        record='        self.retained_depth.push('+local.split('        self.retained_depth.push(',1)[1].split('        self.last_integration',1)[0]
        text=once(text,'        self.recent_voxels = recent;','        self.recent_voxels = recent;\n'+record)
        rebuild='    fn rebuild_depth('+local.split('    fn rebuild_depth(',1)[1].rsplit('\n}\npub fn portal_code',1)[0]
        text=once(text,'\n}\npub fn portal_code','\n'+rebuild+'\n}\npub fn portal_code')
    if '    CorrectionPending {' not in text:
        text=once(text,'    Camera {','    CorrectionPending { timestamp_ns:u64, pgo_count:i64 },\n    Camera {')
    if '            SensorEvent::CorrectionPending {' not in text:
        branch='            SensorEvent::CorrectionPending {'+local.split('            SensorEvent::CorrectionPending {',1)[1].split('            SensorEvent::TrajectoryCorrection {',1)[0]
        text=once(text,'            SensorEvent::TrajectoryCorrection {',branch+'            SensorEvent::TrajectoryCorrection {')
    if 'fn deserialize_depth_poses' not in text:
        if 'deserialize_with = "deserialize_depth_poses"' not in text:
            text=once(text,'        #[serde(default)]\n        corrected_depth:', '        #[serde(default, deserialize_with = "deserialize_depth_poses")]\n        corrected_depth:')
        helper='fn deserialize_depth_poses'+local.split('fn deserialize_depth_poses',1)[1].split('#[derive(Clone, Debug, Serialize, Deserialize)]\npub struct HistoricalPose',1)[0]
        text=once(text,'#[derive(Clone, Debug, Serialize, Deserialize)]\npub struct HistoricalPose',helper+'#[derive(Clone, Debug, Serialize, Deserialize)]\npub struct HistoricalPose')
    files[name]=text
    name='src/recent_voxels.rs'; text=read(name); local=(repo/name).read_text()
    if 'pub fn observe_at(' not in text:
        text=once(text,'    color: Option<[f64; 3]>,','    color: Option<[f64; 3]>,\n    timestamp_ns: u64,')
        begin=text.index('    pub fn observe('); end=text.index('        ensure!(',begin)
        header='    pub fn observe('+local.split('    pub fn observe(',1)[1].split('        ensure!(',1)[0]
        text=text[:begin]+header+text[end:]
        text=once(text,'Vote { hit, color }','Vote { hit, color, timestamp_ns }')
        text=once(text,'        let mut chunks = BTreeMap::<[i32; 3], Vec<VoxelSnapshot>>::new();','        Ok(())\n    }\n    pub fn checkpoint(&self) -> Result<MapUpdate> {\n        let mut chunks = BTreeMap::<[i32; 3], Vec<VoxelSnapshot>>::new();')
    files[name]=text
    name='src/robot_live.rs'; text=read(name)
    if 'SensorEvent::TrajectoryCorrection' not in text:
        text=once(text,'SensorEvent::MapCorrection { timestamp_ns, pose, .. } => {','SensorEvent::MapCorrection { timestamp_ns, pose, .. } | SensorEvent::TrajectoryCorrection { timestamp_ns, pose, .. } => {')
        text=once(text,'        let alignment = &mut value["registries"]["frame_transform"];','        let reprojected = m["map_correction"]["alignment_reprojected"] == true\n            && m["map_correction"]["capture_timestamp_ns"].as_u64().unwrap_or(0) >= self.barrier;\n        let alignment = &mut value["registries"]["frame_transform"];')
        text=once(text,'|| alignment["evidence"]["capture_timestamp_ns"].as_u64().unwrap_or(0) <= self.barrier)','|| (alignment["evidence"]["capture_timestamp_ns"].as_u64().unwrap_or(0) <= self.barrier && !reprojected))')
        files[name]=text
    if 'SensorEvent::CorrectionPending' not in text:
        text=once(text,'            SensorEvent::TrackingLost {','            SensorEvent::CorrectionPending { timestamp_ns, .. } => {\n                self.barrier=self.barrier.max(*timestamp_ns);self.correction=self.correction.max(*timestamp_ns);self.latest=None;\n            }\n            SensorEvent::TrackingLost {')
    files[name]=text
    name='src/fresh_ingress.rs'; text=read(name)
    if '"trajectory_correction"' not in text:
        files[name]=once(text,'if kind == "map_correction" {','if matches!(kind, "map_correction" | "trajectory_correction") {')
    text=files[name]
    if '"correction_pending"' not in text:
        files[name]=once(text,'"map_correction" | "trajectory_correction"','"map_correction" | "trajectory_correction" | "correction_pending"')
    name='src/robot_runner.rs'; text=read(name)
    old='Some(value) => crate::robot_presence::from_live(&value, if hardware == Hardware::Bracketbot { "Bracketbot 0184" } else { "Galbot" })?,'
    if old in text:
        text=once(text,old,'Some(value) => {\n                        let name=std::env::var("SPATIAL_ROBOT_NAME").unwrap_or_else(|_|if hardware==Hardware::Bracketbot { "Bracketbot 0184".into() } else { "Galbot".into() });\n                        crate::robot_presence::from_live(&value,&name)?\n                    },')
    if '"control_state_path"' not in text:
        text=once(text,'        "session_id":session,','        "session_id":session,\n        "control_state_path":path.with_file_name("robot-control.json"),')
    if 'SensorEvent::TrajectoryCorrection' not in text:
        text=once(text,'SensorEvent::Odometry { .. } | SensorEvent::MapCorrection { .. }','SensorEvent::Odometry { .. } | SensorEvent::MapCorrection { .. } | SensorEvent::TrajectoryCorrection { .. }')
    if 'Some("odometry" | "tracking_lost" | "map_correction" | "trajectory_correction" | "correction_pending")' not in text:
        text=once(text,'Some("odometry" | "tracking_lost" | "map_correction")','Some("odometry" | "tracking_lost" | "map_correction" | "trajectory_correction" | "correction_pending")')
    files[name]=text
    name='scripts/bracketbot_capture.py'; text=read(name)
    if 'def trajectory_correction(' not in text:
        local=(repo/name).read_text()
        helpers='POSE_HISTORY_DTYPE ='+local.split('POSE_HISTORY_DTYPE =',1)[1].split('class FramePairs:',1)[0]
        text=once(text,'class FramePairs:',helpers+'class FramePairs:')
        text=once(text,'import threading\n','import threading\nfrom pathlib import Path\n')
        text=once(text,"        pgo = int(record['pgo_count'])\n","        pgo = int(record['pgo_count'])\n        if pgo < 0 or (self.pgo_count is not None and pgo < self.pgo_count):\n            raise NewSessionRequired('SLAM pose graph reset; new map session required')\n")
        files[name]=text
    begin=text.index('def trajectory_correction('); end=text.index('class FramePairs:',begin)
    local=(repo/name).read_text()
    method='def trajectory_correction('+local.split('def trajectory_correction(',1)[1].split('class FramePairs:',1)[0]
    files[name]=text[:begin]+method+text[end:]
    text=files[name]
    if 'def pause_for_correction(' not in text:
        text=once(text,'    def take_scan(','    def pause_for_correction(self,t):\n        with self.lock:\n            self.tracking=False\n            self.boundary=max(self.boundary,t)\n            self.pairs=FramePairs(48)\n\n    def take_scan(')
    files[name]=text
    name='scripts/capture-bracketbot.py'; text=read(name)
    if 'from bracketbot_capture import trajectory_correction' not in text:
        text=once(text,'import numpy as np\n','import numpy as np\nfrom bracketbot_capture import trajectory_correction\n')
        files[name]=text
    text=files[name]
    if '    pending_correction = None' not in text:
        text=once(text,'    gate = PoseGate()','    gate = PoseGate()\n    pending_correction = None')
    begin=text.index('            if admitted:');end=text.index('            elif was_tracking',begin)
    local=(repo/name).read_text()
    block='            if admitted:'+local.split('            if admitted:',1)[1].split('            elif was_tracking',1)[0]
    files[name]=text[:begin]+block+text[end:]
    # Bracket policy: BBOS poses stay live; PGO never rewrites stored samples.
    name='scripts/bracketbot_capture.py';text=files[name];local=(repo/name).read_text()
    begin=text.index('class PoseGate:');end=text.index('POSE_HISTORY_DTYPE =',begin)
    files[name]=text[:begin]+local[local.index('class PoseGate:'):local.index('POSE_HISTORY_DTYPE =')]+text[end:]
    name='src/local_mapper.rs';text=files[name]
    guard='                if self.frame.starts_with(&format!("{BRACKETBOT_FAMILY}:"))'
    for header,body in [
        ('            SensorEvent::CorrectionPending { pgo_count, .. } => {',' { return Ok(()); }'),
        ('            SensorEvent::TrajectoryCorrection { timestamp_ns, pgo_count, pose, corrected_portals, corrected_depth } => {',' {\n                    return self.accept(SensorEvent::Odometry{timestamp_ns,pose});\n                }'),
        ('            SensorEvent::MapCorrection { timestamp_ns, pgo_count, pose } => {',' {\n                    return self.accept(SensorEvent::Odometry{timestamp_ns,pose});\n                }')]:
        if header+'\n'+guard not in text: text=once(text,header,header+'\n'+guard+body)
    if 'if !self.frame.starts_with(&format!("{BRACKETBOT_FAMILY}:")) {\n        self.retained_depth.push(' not in text:
        text=once(text,'        self.retained_depth.push(','        if !self.frame.starts_with(&format!("{BRACKETBOT_FAMILY}:")) {\n        self.retained_depth.push(')
        line='        if self.retained_depth.len()>2048 { self.retained_depth.drain(..self.retained_depth.len()-2048); }'
        text=once(text,line,line+'\n        }')
    files[name]=text
    name='src/robot_live.rs';text=files[name]
    header='            SensorEvent::CorrectionPending { timestamp_ns, .. } => {'
    if header+'\n                if self.preserves_frame' not in text: text=once(text,header,header+'\n                if self.preserves_frame { return; }')
    header='            SensorEvent::MapCorrection { timestamp_ns, pose, .. } | SensorEvent::TrajectoryCorrection { timestamp_ns, pose, .. } => {'
    if header+'\n                if self.preserves_frame' not in text:
        text=once(text,header,header+'\n                if self.preserves_frame {\n                    self.accept(&SensorEvent::Odometry{timestamp_ns:*timestamp_ns,pose:pose.clone()});\n                    return;\n                }')
    text=text.replace('if alignment.is_object() && (!basis_fresh','if alignment.is_object() && !self.preserves_frame && (!basis_fresh')
    files[name]=text
    name='src/frame_transform.rs';text=read(name);local=(repo/name).read_text()
    begin=text.index('        let valid =');end=text.index('        value.validity',begin)
    files[name]=text[:begin]+local[local.index('        let valid ='):local.index('        value.validity')]+text[end:]
    name='scripts/bracketbot_pointing.py';text=read(name);local=(repo/name).read_text()
    if 'def point_in_frame(' not in text:
        helper='def point_in_frame('+local.split('def point_in_frame(',1)[1].split('\n\ndef ',1)[0]+'\n\n\n'
        text=once(text,'def select_phone(',helper+'def select_phone(') if 'def select_phone(' in text else once(text,'def select_target(',helper+'def select_target(')
    text=text.replace("presence['frame'] == expected_home_frame","presence['frame'] in (expected_home_frame,native)")
    text=text.replace("pose['to_frame_id'] == expected_home_frame","pose['to_frame_id'] == presence['frame']")
    text=text.replace("inverse_point(alignment['transform'], vector(pose['translation']))","point_in_frame(pose['translation'],presence['frame'],native,alignment['transform'])")
    files[name]=text
    name='scripts/bracketbot_note.py';text=read(name)
    text=text.replace('from bracketbot_pointing import fresh,inverse_point,vector\n','from bracketbot_pointing import fresh,inverse_point,vector,point_in_frame\n')
    text=text.replace("request['frame']==home['frame']","request['frame'] in (home['frame'],scene['frame'])")
    text=text.replace("inverse_point(scene['registries']['frame_transform']['transform'],vector(note['position']))","point_in_frame(note['position'],request['frame'],scene['frame'],scene['registries']['frame_transform']['transform'])")
    files[name]=text
    name='scripts/bracketbot_volume_speech.py';text=read(name);local=(repo/name).read_text()
    text=text.replace('from bracketbot_pointing import vector, fresh\n','from bracketbot_pointing import vector, fresh, point_in_frame\n')
    begin=text.index('def inputs(');end=text.index('\n\nclass Entries:',begin)
    files[name]=text[:begin]+local[local.index('def inputs('):local.index('\n\nclass Entries:')]+text[end:]
    # Check every anchor before writing; preserve all unrelated deployed code.
    for name,text in files.items(): (source/name).write_text(text)
    return list(files)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build-dir',type=Path,default=Path('/Users/nilspihl/Documents/ChatGPT/harness/data/bracketbot-linux-build'))
    ap.add_argument('--refresh-deps',action='store_true')
    ap.add_argument('--reuse-source',action='store_true',help='Retry the build using the already staged deployed source')
    args=ap.parse_args(); root=args.build_dir.resolve(); root.mkdir(parents=True,exist_ok=True)
    source=root/'source'; source.mkdir(exist_ok=True)
    repo=Path(__file__).resolve().parents[1]
    paths=['Cargo.toml','Cargo.lock','src','scripts','vendor','.cargo','config','web','ios/PortalBridge/Cargo.toml','ios/PortalBridge/src']
    if args.refresh_deps or not (source/'.deps').exists(): paths.append('.deps')
    command=shlex.join(['tar','-C','/home/bracketbot/deployments/domain-harness/runner/source',
        '--exclude=__pycache__','-czf','-',*paths])
    archive_path=root/'source-transfer.tar.gz'
    if not args.reuse_source:
        with archive_path.open('wb') as output:
            subprocess.run(['bb','ssh','0184',command],stdout=output,check=True)
        with tarfile.open(archive_path,'r:gz') as archive:
            archive.extractall(source,filter='data')
    # Crates can have legitimate src/target directories: never drop those.
    missing=[]
    for checksum in (source/'.deps/bracketbot-vendor').glob('*/.cargo-checksum.json'):
        for relative in json.loads(checksum.read_text()).get('files',{}):
            path=checksum.parent/relative
            if not path.exists(): missing.append(str(path.relative_to(source)))
    if missing:
        repair=root/'vendor-repair.tar.gz'
        command=shlex.join(['tar','-C','/home/bracketbot/deployments/domain-harness/runner/source','-czf','-',*missing])
        with repair.open('wb') as output: subprocess.run(['bb','ssh','0184',command],stdout=output,check=True)
        with tarfile.open(repair,'r:gz') as archive: archive.extractall(source,filter='data')
    changed=patch_sources(source,repo)
    target=root/'target'; target.mkdir(exist_ok=True)
    subprocess.run(['docker','run','--rm','--platform','linux/arm64',
        '-v',str(source)+':/work','-v',str(target)+':/build-target','-w','/work',
        '-e','CARGO_TARGET_DIR=/build-target','-e','CARGO_PROFILE_DEV_DEBUG=0',
        'rust:1.91-bullseye','cargo','build','--locked','--offline','--jobs','4','--bin','bracketbot-runner'],check=True)
    binary=target/'debug/bracketbot-runner'
    with tarfile.open(archive_path,'r:gz') as archive:
        originals={name:hashlib.sha256(archive.extractfile(name).read()).hexdigest() for name in changed}
    report=dict(binary=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        changed_paths=changed,original_sha256=originals,home_untouched=True)
    (root/'correction-build.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__': main()
