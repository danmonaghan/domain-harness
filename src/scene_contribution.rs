//! Device-neutral, scene-scoped raw measurements. Coordinates stay in the
//! producer's native frame; only the receiving scene composes the alignment.
use crate::local_mapper::{compose, from_rigid, inverse, rigid};
use anyhow::{ensure, Result};
use auki_components::*;
use auki_scenegraph::{MapSnapshot, RigidTransform};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};

pub const COMPONENT: &str = "scene-contributions";

/// QrAnchor poses use printed-right/up/out axes. The mapper and saved scene
/// give that same physical marker frame different local names.
pub fn portal_measurement(
    measured: &auki_scenegraph::QrAnchor,
    reference: &auki_scenegraph::QrAnchor,
) -> Result<RigidTransform> {
    measured.validate()?;
    reference.validate()?;
    ensure!(
        measured.anchor_id == reference.anchor_id
            && (measured.side_length_m - reference.side_length_m).abs() < 1e-6,
        "Portal identity or size differs"
    );
    let mut pose = measured.pose_in_map.clone();
    pose.from_frame_id = reference.pose_in_map.from_frame_id.clone();
    Ok(pose)
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Contribution {
    pub source: ProductReference,
    pub sequence: u64,
    pub clock: clock::ClockRegistryEntry,
    pub timestamp_ns: u64,
    pub sensor_frame: auki_registry::FrameRegistryEntry,
    pub native_frame: auki_registry::FrameRegistryEntry,
    pub sensor_to_native: RigidTransform,
    pub alignment_timestamp_ns: u64,
    pub portal_id: String,
    pub portal_to_native: RigidTransform,
    pub native_to_scene: RigidTransform,
    pub points: Vec<[f64; 3]>,
    pub colors: Vec<Option<[u8; 3]>>,
}
impl ContractType for Contribution {
    const DATATYPE: &'static str = "spatial.scene-contribution/v1";
}
impl Contribution {
    pub fn validate(&self, caller: &str, reference: &MapSnapshot) -> Result<()> {
        ensure!(
            self.source.peer_id == caller
                && self.clock.peer_id == caller
                && self.sensor_frame.peer_id == caller
                && self.native_frame.peer_id == caller
                && !self.source.product_id.is_empty()
                && !self.source.manifest_hash.is_empty(),
            "Contribution owner mismatch"
        );
        ensure!(
            !self.clock.session_id.is_empty() && !self.clock.clock_id.is_empty(),
            "Invalid capture clock"
        );
        let meta = match &self.clock.body {
            clock::ClockBody::MonotonicClock(m) | clock::ClockBody::UtcClock(m) => m,
        };
        ensure!(
            meta.unit == "nanoseconds" && meta.monotonic,
            "Unsupported capture clock"
        );
        for frame in [&self.sensor_frame, &self.native_frame] {
            frame.validate()?;
            ensure!(
                frame.units == auki_registry::LengthUnit::Meters
                    && frame.handedness == auki_registry::Handedness::Right,
                "Contribution frames must be right-handed meters"
            );
        }
        let target = &reference.scenegraph.map.frame.id;
        ensure!(
            self.sensor_to_native.from_frame_id == self.sensor_frame.frame_id
                && self.sensor_to_native.to_frame_id == self.native_frame.frame_id
                && self.portal_to_native.to_frame_id == self.native_frame.frame_id
                && self.native_to_scene.from_frame_id == self.native_frame.frame_id
                && self.native_to_scene.to_frame_id == *target,
            "Contribution frame mismatch"
        );
        let anchor = reference
            .scenegraph
            .anchors
            .get(&self.portal_id)
            .ok_or_else(|| anyhow::anyhow!("Unknown contribution portal"))?;
        ensure!(
            self.portal_to_native.from_frame_id == anchor.pose_in_map.from_frame_id,
            "Contribution portal frame mismatch"
        );
        ensure!(
            self.alignment_timestamp_ns > 0 && self.timestamp_ns >= self.alignment_timestamp_ns,
            "Contribution predates alignment"
        );
        for pose in [
            &self.sensor_to_native,
            &self.portal_to_native,
            &self.native_to_scene,
        ] {
            from_rigid(pose).validate()?;
        }
        let expected = compose(
            &from_rigid(&anchor.pose_in_map),
            &inverse(&from_rigid(&self.portal_to_native))?,
        )?;
        let delta = compose(&inverse(&expected)?, &from_rigid(&self.native_to_scene))?;
        ensure!(
            delta.translation.iter().map(|v| v * v).sum::<f64>() < 0.0001
                && delta.rotation_xyzw[3].abs() > 0.9999,
            "Contribution alignment disagrees with portal"
        );
        ensure!(
            !self.points.is_empty()
                && self.points.len() <= 2048
                && self.colors.len() == self.points.len()
                && self.points.iter().all(|p| p.iter().all(|v| v.is_finite())
                    && p.iter().map(|v| v * v).sum::<f64>() <= 64.),
            "Invalid contribution points"
        );
        Ok(())
    }
    pub fn localize(
        self,
        reference: &MapSnapshot,
        snapshot: auki_scenegraph::component::SnapshotReference,
    ) -> Result<crate::collaborative_mapper::LocalizedScan> {
        self.validate(&self.source.peer_id, reference)?;
        let frame = &reference.scenegraph.map.frame.id;
        let clock = crate::model::clock_ref(&self.clock);
        let pose = compose(
            &from_rigid(&self.native_to_scene),
            &from_rigid(&self.sensor_to_native),
        )?;
        let origin = crate::local_mapper::canonical_origin(
            self.alignment_timestamp_ns,
            clock.clone(),
            &from_rigid(&self.portal_to_native),
            &reference.scenegraph.anchors[&self.portal_id],
        )?;
        // Reuse the SDK's bounded ray budget, retaining actual measured points.
        let steps = self
            .points
            .iter()
            .map(|p| 3 * (p[0].hypot(p[1]).hypot(p[2]) / 0.1).ceil() as usize + 3)
            .max()
            .unwrap_or(1);
        let count = self
            .points
            .len()
            .min(auki_voxel_map::MAX_POINTS)
            .min(auki_voxel_map::MAX_RAY_SAMPLES / steps);
        ensure!(count > 0, "Contribution exceeds ray budget");
        let indices = (0..count)
            .map(|i| i * self.points.len() / count)
            .collect::<Vec<_>>();
        Ok(crate::collaborative_mapper::LocalizedScan {
            observation: auki_voxel_map::DepthObservation {
                source: self.source.clone(),
                sequence: self.sequence,
                timestamp_ns: self.timestamp_ns,
                clock: clock.clone(),
                sensor_frame_id: self.sensor_frame.frame_id.clone(),
                hit_points_m: indices.iter().map(|i| self.points[*i]).collect(),
                pose: auki_voxel_map::TimedSensorPose {
                    timestamp_ns: self.timestamp_ns,
                    clock: clock.clone(),
                    sensor_to_map: rigid(&pose, &self.sensor_frame.frame_id, frame),
                    portal_snapshot: snapshot,
                },
            },
            binding: auki_voxel_map::SensorBinding {
                sensor_product: self.source,
                sensor_frame: self.sensor_frame,
                observation_clock: clock,
            },
            colors: indices.iter().map(|i| self.colors[*i]).collect(),
            origin,
        })
    }
}

#[derive(Default)]
pub struct Inbox {
    pending: BTreeMap<String, (Contribution, Instant)>,
    watermarks: BTreeMap<String, (u64, u64)>,
}
impl Inbox {
    pub fn accept(
        &mut self,
        caller: &str,
        c: Contribution,
        reference: &MapSnapshot,
    ) -> Result<bool> {
        c.validate(caller, reference)?;
        let key = serde_json::to_string(&(&c.source, &c.clock))?;
        if self
            .watermarks
            .get(&key)
            .is_some_and(|(s, t)| c.sequence <= *s || c.timestamp_ns <= *t)
        {
            return Ok(false);
        }
        ensure!(
            self.watermarks.contains_key(&key) || self.watermarks.len() < 128,
            "Contribution source capacity reached"
        );
        self.watermarks
            .insert(key.clone(), (c.sequence, c.timestamp_ns));
        self.pending.insert(key, (c, Instant::now()));
        Ok(true)
    }
    pub fn take(&mut self) -> Vec<Contribution> {
        std::mem::take(&mut self.pending)
            .into_values()
            .filter(|(_, t)| t.elapsed() < Duration::from_secs(2))
            .map(|(c, _)| c)
            .collect()
    }
}

pub async fn send(
    peer: &auki_sdk::AukiPeer,
    home: &crate::network::HomeConnection,
    c: &Contribution,
) -> Result<bool> {
    crate::mobile_pose::invoke_pinned(peer, home, COMPONENT, "ingest", "scene-contribution", c,
        "Scene contribution receiver unreachable").await
}
