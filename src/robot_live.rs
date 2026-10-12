//! Small control snapshot: live odometry must not wait for QR/voxel processing.
//! Mapper-owned alignment is never inferred here. Capture barriers invalidate
//! it immediately, even while the mapper is busy processing an older image.
use crate::{local_mapper::{compose, Origin, SensorEvent}, network::CanonicalScene};
use auki_odometry::Pose;
use serde_json::{Value, json};
use std::time::{Duration, Instant};

#[derive(Default)]
pub struct LiveState {
    basis: Option<(Value, Instant)>,
    latest: Option<(u64, Pose)>,
    barrier: u64,
    correction: u64,
    pose_barrier: u64,
    preserves_frame: bool,
}
impl LiveState {
    pub fn new(preserves_frame: bool) -> Self { Self { preserves_frame, ..Self::default() } }
    pub fn accept(&mut self, event: &SensorEvent) {
        match event {
            SensorEvent::CorrectionPending { timestamp_ns, .. } => {
                if self.preserves_frame { return; }
                self.barrier=self.barrier.max(*timestamp_ns);self.correction=self.correction.max(*timestamp_ns);self.latest=None;
            }
            SensorEvent::TrackingLost { timestamp_ns } => {
                self.pose_barrier = self.pose_barrier.max(*timestamp_ns);
                if !self.preserves_frame { self.barrier = self.barrier.max(*timestamp_ns); }
                self.latest = None;
            }
            SensorEvent::MapCorrection { timestamp_ns, pose, .. } | SensorEvent::TrajectoryCorrection { timestamp_ns, pose, .. } => {
                if self.preserves_frame {
                    self.accept(&SensorEvent::Odometry{timestamp_ns:*timestamp_ns,pose:pose.clone()});
                    return;
                }
                self.barrier = self.barrier.max(*timestamp_ns);
                self.correction = self.correction.max(*timestamp_ns);
                self.latest = pose.validate().ok().map(|_| (*timestamp_ns, pose.clone()));
            }
            SensorEvent::Odometry { timestamp_ns, pose } if *timestamp_ns >= self.barrier.max(self.pose_barrier) => {
                if self.latest.as_ref().is_none_or(|(t,_)| timestamp_ns > t) {
                    self.latest = pose.validate().ok().map(|_| (*timestamp_ns, pose.clone()));
                }
            }
            _ => {}
        }
    }
    pub fn set_basis(&mut self, scene: &CanonicalScene) {
        self.basis = Some((json!({"schema":"spatial.robot-control-state/v1",
            "authority_peer_id":scene.authority_peer_id,"frame":scene.frame,
            "registries":{
                "local_mapping":scene.registries["local_mapping"],
                "frame_transform":scene.registries["frame_transform"],
                "frame_transform_product":scene.registries["frame_transform_product"]
            }}), Instant::now()));
    }
    pub fn snapshot(&self) -> Option<Value> {
        let (basis, received) = self.basis.as_ref()?;
        let mut value = basis.clone();
        let m = &mut value["registries"]["local_mapping"];
        let origin: Option<Origin> = serde_json::from_value(m["origin"].clone()).ok();
        let template = m["robot_pose"].clone();
        m["robot_pose"] = Value::Null;
        m["status"] = json!("waiting_for_odometry");
        let correction_applied = m["map_correction"]["capture_timestamp_ns"].as_u64().unwrap_or(0) >= self.correction;
        let basis_fresh = received.elapsed() < Duration::from_secs(3);
        if basis_fresh && correction_applied && template.is_object()
            && let (Some(origin), Some((timestamp, raw))) = (origin, &self.latest)
            && let Ok(pose) = compose(&origin.odometry_to_local, raw)
        {
            let mut robot = template;
            robot["capture_timestamp_ns"] = json!(timestamp);
            robot["pose"] = json!(pose);
            m["robot_pose"] = robot;
            m["status"] = json!("mapping");
        }
        let reprojected = m["map_correction"]["alignment_reprojected"] == true
            && m["map_correction"]["capture_timestamp_ns"].as_u64().unwrap_or(0) >= self.barrier;
        let alignment = &mut value["registries"]["frame_transform"];
        if alignment.is_object() && !self.preserves_frame && (!basis_fresh || !correction_applied || self.latest.is_none()
            || (alignment["evidence"]["capture_timestamp_ns"].as_u64().unwrap_or(0) <= self.barrier
                && !reprojected))
        {
            alignment["validity"] = json!("stale");
            alignment["reason"] = json!("Live capture barrier or unavailable mapper basis");
        }
        Some(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::local_mapper::identity;
    fn state() -> LiveState {
        let origin = crate::local_mapper::establish_origin(1,
            crate::model::clock_ref(&crate::model::clock_entry("robot","session")), &identity()).unwrap();
        let mut state = LiveState::default();
        state.basis = Some((json!({"registries":{
            "local_mapping":{"origin":origin,"robot_pose":{"clock":"preserved","frame":"native"}},
            "frame_transform":{"validity":"valid","evidence":{"capture_timestamp_ns":10}}
        }}),Instant::now()));
        state.accept(&SensorEvent::Odometry{timestamp_ns:20,pose:identity()});
        state
    }
    #[test]
    fn live_pose_advances_without_another_map_snapshot() {
        let mut s=state(); let mut p=identity(); p.translation=[1.,2.,3.];
        s.accept(&SensorEvent::Odometry{timestamp_ns:30,pose:p});
        let v=s.snapshot().unwrap();
        assert_eq!(v["registries"]["local_mapping"]["robot_pose"]["capture_timestamp_ns"],30);
        assert_eq!(v["registries"]["local_mapping"]["robot_pose"]["pose"]["translation"],json!([1.,2.,3.]));
        assert_eq!(v["registries"]["local_mapping"]["robot_pose"]["clock"],"preserved");
        assert_eq!(v["registries"]["frame_transform"]["validity"],"valid");
    }
    #[test]
    fn tracking_loss_invalidates_before_mapper_and_old_basis_cannot_restore_it() {
        let mut s=state();
        s.accept(&SensorEvent::TrackingLost{timestamp_ns:21});
        assert!(s.snapshot().unwrap()["registries"]["local_mapping"]["robot_pose"].is_null());
        s.accept(&SensorEvent::Odometry{timestamp_ns:30,pose:identity()});
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"stale");
        s.basis.as_mut().unwrap().0["registries"]["frame_transform"]["evidence"]["capture_timestamp_ns"]=json!(25);
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"valid");
    }
    #[test]
    fn correction_and_stalled_mapper_fail_closed() {
        let mut s=state();
        s.accept(&SensorEvent::MapCorrection{timestamp_ns:21,pgo_count:1,pose:identity()});
        assert!(s.snapshot().unwrap()["registries"]["local_mapping"]["robot_pose"].is_null());
        s.basis.as_mut().unwrap().1=Instant::now()-Duration::from_secs(4);
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"stale");
    }
    #[test]
    fn corrected_history_restores_alignment_only_after_mapper_applies_it() {
        let mut s=state();
        s.accept(&SensorEvent::TrajectoryCorrection{timestamp_ns:21,pgo_count:2,pose:identity(),corrected_portals:Default::default(),corrected_depth:Default::default()});
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"stale");
        s.basis.as_mut().unwrap().0["registries"]["local_mapping"]["map_correction"]=json!({"capture_timestamp_ns":21,"alignment_reprojected":true});
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"valid");
        s.accept(&SensorEvent::Odometry{timestamp_ns:30,pose:identity()});
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"valid");
        s.accept(&SensorEvent::MapCorrection{timestamp_ns:40,pgo_count:3,pose:identity()});
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"stale");
    }
    #[test]
    fn bbos_tracking_and_pgo_preserve_the_frame_relationship() {
        let mut s=state();s.preserves_frame=true;
        s.accept(&SensorEvent::TrackingLost{timestamp_ns:21});
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"valid");
        s.accept(&SensorEvent::Odometry{timestamp_ns:20,pose:identity()});
        assert!(s.snapshot().unwrap()["registries"]["local_mapping"]["robot_pose"].is_null());
        s.accept(&SensorEvent::Odometry{timestamp_ns:30,pose:identity()});
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"valid");
        s.accept(&SensorEvent::MapCorrection{timestamp_ns:40,pgo_count:1,pose:identity()});
        assert_eq!(s.snapshot().unwrap()["registries"]["frame_transform"]["validity"],"valid");
    }
}
