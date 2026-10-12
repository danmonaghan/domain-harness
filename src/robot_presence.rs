//! Small authenticated robot presence, independent of geometry contributions.
use anyhow::{Result, ensure};
use auki_components::{ContractType, ClockReference};
use auki_scenegraph::RigidTransform;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{collections::BTreeMap, time::{Duration, Instant}};

pub const COMPONENT: &str = "home-robot-presence";
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sample {
    pub name: String,
    pub timestamp_ns: u64,
    pub clock: ClockReference,
    pub native_pose: RigidTransform,
    pub alignment: crate::frame_transform::FrameTransform,
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Update { pub sample: Option<Sample> }
impl ContractType for Update { const DATATYPE: &'static str = "spatial.robot-presence/v1"; }
#[derive(Default)]
pub struct Presence {
    entries: BTreeMap<String, (Sample, RigidTransform, Instant)>,
    relationships: BTreeMap<(String,String,String),crate::frame_transform::FrameTransform>,
}
impl Presence {
    pub fn accept(&mut self, peer: &str, home: &str, frame: &str, update: Update, now: Instant) -> Result<()> {
        ensure!(!peer.is_empty(), "Missing robot identity");
        let Some(sample) = update.sample else { self.entries.remove(peer); return Ok(()); };
        let a = &sample.alignment;
        let t = a.transform.as_ref().ok_or_else(|| anyhow::anyhow!("Missing Home transform"))?;
        let evidence = a.evidence.as_ref().ok_or_else(|| anyhow::anyhow!("Missing alignment evidence"))?;
        ensure!(a.schema == crate::frame_transform::SCHEMA
            && a.validity == crate::frame_transform::Validity::Valid
            && a.authority_peer_id == peer && sample.clock.peer_id == peer
            && evidence.clock == sample.clock && evidence.reference_peer_id == home
            && a.source_frame.id == sample.native_pose.to_frame_id
            && t.from_frame_id == sample.native_pose.to_frame_id
            && t.to_frame_id == frame && a.target_frame.as_ref().is_some_and(|f| f.id == frame)
            && sample.native_pose.from_frame_id == "robot.base"
            && sample.timestamp_ns >= evidence.capture_timestamp_ns,
            "Robot presence identity/frame mismatch");
        ensure!(!sample.name.trim().is_empty() && sample.name.len() <= 120 && sample.timestamp_ns > 0,
            "Invalid robot name/timestamp");
        for pose in [&sample.native_pose, t] {
            ensure!(pose.translation.iter().chain(pose.rotation_wxyz.iter()).all(|v|v.is_finite())
                && (pose.rotation_wxyz.iter().map(|v|v*v).sum::<f64>()-1.).abs() < 1e-6,
                "Invalid robot pose");
        }
        if let Some((old, _, _)) = self.entries.get(peer)
            && old.clock == sample.clock && sample.timestamp_ns <= old.timestamp_ns { return Ok(()); }
        self.entries.retain(|_, (_,_,received)| now.duration_since(*received) < Duration::from_secs(3));
        ensure!(self.entries.len() < 64 || self.entries.contains_key(peer), "Robot presence capacity reached");
        let key=(peer.to_owned(),sample.native_pose.to_frame_id.clone(),serde_json::to_string(&sample.clock)?);
        ensure!(self.relationships.contains_key(&key)||self.relationships.len()<256,"Too many native relationships");
        if self.relationships.get(&key).is_none_or(|old|old.evidence.as_ref().is_none_or(|e|e.capture_timestamp_ns<evidence.capture_timestamp_ns)) {
            self.relationships.insert(key.clone(),a.clone());
        }
        let retained=self.relationships[&key].transform.as_ref().unwrap();
        let composed = crate::local_mapper::compose(&crate::local_mapper::from_rigid(retained),
            &crate::local_mapper::from_rigid(&sample.native_pose))?;
        let pose = crate::local_mapper::rigid(&composed, "robot.base", frame);
        self.entries.insert(peer.into(), (sample, pose, now));
        Ok(())
    }
    pub fn snapshot(&self, now: Instant, frame: &str) -> Value {
        let poses: Vec<_> = self.entries.iter().filter(|(_,(_,_,received))|
            now.duration_since(*received) < Duration::from_secs(3)).map(|(peer,(s,pose,received))|
            json!({"peer_id":peer,"name":s.name,"pose":pose,"native_pose":s.native_pose,
                "clock":s.clock,"capture_timestamp_ns":s.timestamp_ns,
                "received_age_ms":now.duration_since(*received).as_millis()})).collect();
        json!({"frame":frame,"poses":poses,"transforms":self.relationships.values().collect::<Vec<_>>()})
    }
}
pub fn append_labels(view: &mut Value, presence: &Value, frame: &str) {
    if presence["frame"] != frame { return; }
    if let Some(poses) = presence["poses"].as_array() {
        for pose in poses {
            view["notes"].as_array_mut().unwrap().push(json!({
                "id":format!("robot-name:{}",pose["peer_id"].as_str().unwrap_or("")),
                "text":pose["name"],"position":pose["pose"]["translation"],
                "height_m":0,"robot_pose":pose}));
        }
    }
}
pub fn from_live(value: &Value, name: &str) -> Result<Update> {
    let r = &value["registries"];
    let a: crate::frame_transform::FrameTransform = serde_json::from_value(r["frame_transform"].clone())?;
    if a.validity != crate::frame_transform::Validity::Valid || r["local_mapping"]["status"] != "mapping" {
        return Ok(Update { sample: None });
    }
    let p = &r["local_mapping"]["robot_pose"];
    let raw: auki_odometry::Pose = serde_json::from_value(p["pose"].clone())?;
    let frame = p["frame"].as_str().ok_or_else(||anyhow::anyhow!("Missing robot frame"))?;
    ensure!(p["from_frame"]["frame_id"] == "robot.base", "Not a base pose");
    Ok(Update { sample: Some(Sample { name:name.into(), timestamp_ns:p["capture_timestamp_ns"].as_u64().ok_or_else(||anyhow::anyhow!("Missing capture time"))?,
        clock:serde_json::from_value(p["clock"].clone())?, native_pose:crate::local_mapper::rigid(&raw,"robot.base",frame), alignment:a }) })
}
