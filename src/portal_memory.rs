//! Portal positions remembered in the robot's persistent SLAM map.
//!
//! BBOS SLAM saves its map and relocalizes into it on boot, so a portal measured
//! once keeps its place in that map. A restarted runner restores it and aligns to
//! Home without seeing the portal again.
//!
//! A fresh SLAM map has a new origin. The memory still carries over when the wheels
//! show the robot stood still across the change: its last pose in the old map and
//! its first pose in the new one are then the same place.
use auki_odometry::Pose;
use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::PathBuf, time::{Duration, Instant, SystemTime, UNIX_EPOCH}};

const SCHEMA: &str = "spatial.robot-portal-memory/v1";
/// BBOS keeps this file for as long as the SLAM map it belongs to.
const DEFAULT_MAP_MARKER: &str = "/home/bracketbot/bbos/bbos/daemons/slam/.maps/slam.history";

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Remembered {
    pub payload: String,
    pub side_length_m: f64,
    pub marker_in_odometry: Pose,
    pub reprojection_rms_px: serde_json::Value,
}
/// Where the robot stood, in the SLAM map and on its wheels, at one moment.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Standing {
    pub base_in_odometry: Pose,
    /// Per-wheel position as the motor controller counts it, in turns.
    pub wheels: [f64; 2],
    pub wall_ns: u64,
}
impl Standing {
    pub fn now(base_in_odometry: Pose, wheels: [f64; 2]) -> Self {
        let wall_ns = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_nanos() as u64);
        Self { base_in_odometry, wheels, wall_ns }
    }
    /// Balancing rocks the wheels a little; a real move turns them further or apart.
    fn same_place(&self, later: &Self) -> bool {
        let moved = [later.wheels[0] - self.wheels[0], later.wheels[1] - self.wheels[1]];
        moved.iter().all(|turns| turns.abs() < 0.1)
            && (moved[0] - moved[1]).abs() < 0.04
            && later.wall_ns.saturating_sub(self.wall_ns) < 30 * 60 * 1_000_000_000
            && later.wall_ns >= self.wall_ns
    }
}
#[derive(Serialize, Deserialize)]
struct Saved {
    schema: String,
    map_identity: String,
    portals: BTreeMap<String, Remembered>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    standing: Option<Standing>,
}
pub struct PortalMemory {
    path: PathBuf,
    map_marker: PathBuf,
    pending: bool,
    written: Option<(String, BTreeMap<String, Remembered>)>,
    checkpointed: Option<Instant>,
}
impl PortalMemory {
    pub fn open(path: PathBuf) -> Self {
        let map_marker = std::env::var_os("SPATIAL_SLAM_MAP_MARKER")
            .map(PathBuf::from)
            .unwrap_or_else(|| DEFAULT_MAP_MARKER.into());
        Self { path, map_marker, pending: true, written: None, checkpointed: None }
    }
    #[cfg(test)]
    fn with_marker(path: PathBuf, map_marker: PathBuf) -> Self {
        Self { path, map_marker, pending: true, written: None, checkpointed: None }
    }
    /// Inode and birth time: a recreated file is a different SLAM map even when
    /// the filesystem reuses the inode.
    fn map_identity(&self) -> Option<String> {
        use std::os::unix::fs::MetadataExt;
        let meta = std::fs::metadata(&self.map_marker).ok()?;
        let born = meta.created().ok()?.duration_since(std::time::UNIX_EPOCH).ok()?;
        Some(format!("{}:{}:{}", meta.dev(), meta.ino(), born.as_nanos()))
    }
    /// Portals remembered for the SLAM map that is live now, in its coordinates.
    /// Yields once per run. `standing` is where the robot is now; with it, portals
    /// from the previous map carry over if the robot has not moved since.
    pub fn restore(&mut self, standing: Option<&Standing>) -> BTreeMap<String, Remembered> {
        if !std::mem::take(&mut self.pending) {
            return BTreeMap::new();
        }
        let Some(identity) = self.map_identity() else {
            return BTreeMap::new();
        };
        let Some(saved) = std::fs::read(&self.path)
            .ok()
            .and_then(|bytes| serde_json::from_slice::<Saved>(&bytes).ok())
            .filter(|saved| saved.schema == SCHEMA)
        else {
            return BTreeMap::new();
        };
        if saved.map_identity == identity {
            return saved.portals;
        }
        let (Some(then), Some(now)) = (&saved.standing, standing) else {
            return BTreeMap::new();
        };
        if !then.same_place(now) {
            return BTreeMap::new();
        }
        // new map <- robot <- old map
        let Ok(carry) = crate::local_mapper::inverse(&then.base_in_odometry)
            .and_then(|old| crate::local_mapper::compose(&now.base_in_odometry, &old))
        else {
            return BTreeMap::new();
        };
        saved
            .portals
            .into_iter()
            .filter_map(|(id, mut portal)| {
                portal.marker_in_odometry =
                    crate::local_mapper::compose(&carry, &portal.marker_in_odometry).ok()?;
                Some((id, portal))
            })
            .collect()
    }
    pub fn remember(&mut self, portals: BTreeMap<String, Remembered>, standing: Option<Standing>) -> Result<()> {
        let Some(identity) = self.map_identity() else {
            return Ok(());
        };
        if portals.is_empty() || self.written.as_ref() == Some(&(identity.clone(), portals.clone())) {
            return Ok(());
        }
        self.write(identity, portals, standing)
    }
    /// Keep the robot's latest standing beside the portals, at most once a second.
    pub fn checkpoint(&mut self, standing: Standing) -> Result<()> {
        if self.checkpointed.is_some_and(|at| at.elapsed() < Duration::from_secs(1)) {
            return Ok(());
        }
        let (Some(identity), Some((written, portals))) = (self.map_identity(), self.written.clone()) else {
            return Ok(());
        };
        if identity != written {
            return Ok(());
        }
        self.write(identity, portals, Some(standing))
    }
    fn write(&mut self, identity: String, portals: BTreeMap<String, Remembered>, standing: Option<Standing>) -> Result<()> {
        let saved = Saved { schema: SCHEMA.into(), map_identity: identity.clone(), portals, standing };
        let temporary = self.path.with_extension("json.new");
        std::fs::write(&temporary, serde_json::to_vec_pretty(&saved)?)?;
        std::fs::rename(&temporary, &self.path)?;
        self.written = Some((identity, saved.portals));
        self.checkpointed = Some(Instant::now());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn portal() -> BTreeMap<String, Remembered> {
        BTreeMap::from([("HNY3JGNLRAE".to_string(), Remembered {
            payload: "HTTPS://R8.HR/HNY3JGNLRAE".into(),
            side_length_m: 0.15,
            marker_in_odometry: Pose { translation: [1., 2., 3.], rotation_xyzw: [0., 0., 0., 1.] },
            reprojection_rms_px: serde_json::json!(0.4),
        })])
    }
    #[test]
    fn a_restart_in_the_same_slam_map_restores_the_portal_once() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let (file, marker) = (dir.path().join("portal-memory.json"), dir.path().join("slam.history"));
        std::fs::write(&marker, b"map")?;
        let mut first = PortalMemory::with_marker(file.clone(), marker.clone());
        assert!(first.restore(None).is_empty(), "nothing remembered yet");
        first.remember(portal(), None)?;
        // The SLAM daemon keeps appending to the same file.
        std::fs::write(&marker, b"map grew")?;
        let mut restarted = PortalMemory::with_marker(file, marker);
        assert_eq!(restarted.restore(None), portal());
        assert!(restarted.restore(None).is_empty(), "restores once per run");
        Ok(())
    }
    #[test]
    fn a_fresh_slam_map_forgets_the_portal() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let (file, marker) = (dir.path().join("portal-memory.json"), dir.path().join("slam.history"));
        std::fs::write(&marker, b"map")?;
        PortalMemory::with_marker(file.clone(), marker.clone()).remember(portal(), None)?;
        std::fs::remove_file(&marker)?;
        std::thread::sleep(std::time::Duration::from_millis(20));
        std::fs::write(&marker, b"fresh map")?;
        assert!(PortalMemory::with_marker(file.clone(), marker.clone()).restore(None).is_empty());
        std::fs::remove_file(&marker)?;
        let mut no_map = PortalMemory::with_marker(file, marker);
        assert!(no_map.restore(None).is_empty(), "no SLAM map, no memory");
        no_map.remember(portal(), None)?;
        Ok(())
    }
    #[test]
    fn a_map_reset_keeps_the_portal_only_if_the_robot_stood_still() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let (file, marker) = (dir.path().join("portal-memory.json"), dir.path().join("slam.history"));
        let at = |x: f64, wheels: [f64; 2]| Standing::now(
            Pose { translation: [x, 0., 0.], rotation_xyzw: [0., 0., 0., 1.] }, wheels);
        let reset = || -> Result<()> {
            std::fs::remove_file(&marker)?;
            std::thread::sleep(std::time::Duration::from_millis(20));
            Ok(std::fs::write(&marker, b"fresh map")?)
        };
        std::fs::write(&marker, b"map")?;
        // The robot stood 4 m along the old map; the portal was at (1, 2, 3).
        PortalMemory::with_marker(file.clone(), marker.clone()).remember(portal(), Some(at(4., [10., 20.])))?;
        reset()?;
        // The new map starts where the robot stands, so the portal lands 4 m back.
        let carried = PortalMemory::with_marker(file.clone(), marker.clone()).restore(Some(&at(0., [10.02, 20.01])));
        let moved = carried["HNY3JGNLRAE"].marker_in_odometry.translation;
        assert!((moved[0] + 3.).abs() < 1e-9 && (moved[1] - 2.).abs() < 1e-9, "{moved:?}");
        for (wheels, why) in [([10.5, 20.5], "drove on"), ([10.05, 19.95], "turned on the spot")] {
            assert!(PortalMemory::with_marker(file.clone(), marker.clone()).restore(Some(&at(0., wheels))).is_empty(), "{why}");
        }
        assert!(PortalMemory::with_marker(file.clone(), marker.clone()).restore(None).is_empty(), "no wheel reading");
        Ok(())
    }
}
