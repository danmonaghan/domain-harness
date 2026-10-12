//! A new physical map frame anchored by the first valid robot pose. Home's
//! references are overlaid after alignment; they never become measurements.
use anyhow::{Result, anyhow, ensure};
use auki_components::*;
use auki_odometry::{Pose, PoseContract, PoseQuery, Tracking, pose_at};
use auki_registry::FrameRegistryEntry;
use auki_scenegraph::{
    MapDefinition, MapSnapshot, QrAnchor, RigidTransform,
    component::{MapComponent, MapComponentConfig, UpsertQr},
};
use auki_voxel_map::{
    DepthObservation, PortalVoxelMapper, SensorBinding, TimedSensorPose, VoxelMapComponent,
    VoxelMapDefinition, VoxelSnapshot,
};
use base64::{Engine, engine::general_purpose::STANDARD};
use prost::Message;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{collections::BTreeMap, sync::Arc};

pub const FAMILY: &str = "galbot-local";
pub const BRACKETBOT_FAMILY: &str = "bracketbot-local";
pub fn is_local_family(id: &str) -> bool {
    matches!(id, FAMILY | BRACKETBOT_FAMILY)
}
pub fn is_local_frame(id: &str) -> bool {
    id.strip_prefix("galbot-local:")
        .or_else(|| id.strip_prefix("bracketbot-local:"))
        .or_else(|| id.strip_prefix("simulator-local:"))
        .or_else(|| id.strip_prefix("home-live:"))
        .and_then(|s| s.strip_suffix(":zup").or_else(|| s.strip_suffix(":opengl")))
        .is_some_and(|s| uuid::Uuid::parse_str(s).is_ok())
}
pub fn display_frame(id: &str) -> Result<String> {
    ensure!(is_local_frame(id), "not a local map frame");
    Ok(id
        .strip_suffix(":zup")
        .map(|s| format!("{s}:opengl"))
        .unwrap_or_else(|| id.into()))
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CameraSample {
    pub timestamp_ns: u64,
    pub rgb_timestamp_ns: u64,
    pub transform_timestamp_ns: u64,
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub gray_base64: String,
    pub calibration: crate::runner_voxels::SensorCalibration,
    pub hit_points_m: Vec<[f64; 3]>,
    pub hit_colors_srgb: Vec<[u8; 3]>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SensorEvent {
    Odometry {
        timestamp_ns: u64,
        pose: Pose,
    },
    TrackingLost {
        timestamp_ns: u64,
    },
    MapCorrection {
        timestamp_ns: u64,
        pgo_count: i64,
        pose: Pose,
    },
    TrajectoryCorrection {
        timestamp_ns: u64,
        pgo_count: i64,
        pose: Pose,
        corrected_portals: BTreeMap<String, HistoricalPose>,
        #[serde(default, deserialize_with = "deserialize_depth_poses")]
        corrected_depth: BTreeMap<u64, Pose>,
    },
    CorrectionPending { timestamp_ns:u64, pgo_count:i64 },
    Camera {
        sample: CameraSample,
    },
    Lidar {
        timestamp_ns: u64,
        reading: crate::runner_voxels::SensorReading,
        sensor_to_base: Pose,
    },
    Status {
        detail: String,
    },
}
impl ContractType for SensorEvent {
    const DATATYPE: &'static str = "spatial-harness.local-sensors/v1";
}
fn deserialize_depth_poses<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<BTreeMap<u64, Pose>, D::Error> {
    BTreeMap::<String, Pose>::deserialize(d)?.into_iter().map(|(timestamp, pose)| {
        timestamp.parse::<u64>().map(|t| (t, pose)).map_err(serde::de::Error::custom)
    }).collect()
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HistoricalPose {
    pub timestamp_ns: u64,
    pub pose: Pose,
}
#[derive(Clone)]
struct RetainedDepth {
    observation: DepthObservation,
    colors: Vec<Option<[u8;3]>>,
    base_pose: Pose,
    sensor_to_base: Pose,
}

pub fn identity() -> Pose {
    Pose {
        translation: [0.; 3],
        rotation_xyzw: [0., 0., 0., 1.],
    }
}
fn spatial(p: &Pose) -> auki_datatypes::pose::SpatialTransform {
    let [x, y, z, w] = p.rotation_xyzw;
    auki_datatypes::pose::SpatialTransform {
        translation: Some(auki_datatypes::pose::Vec3 {
            x: p.translation[0],
            y: p.translation[1],
            z: p.translation[2],
        }),
        orientation: Some(auki_datatypes::pose::Quat { x, y, z, w }),
    }
}
fn pose(p: auki_datatypes::pose::SpatialTransform) -> Pose {
    let t = p.translation.unwrap();
    let q = p.orientation.unwrap();
    Pose {
        translation: [t.x, t.y, t.z],
        rotation_xyzw: [q.x, q.y, q.z, q.w],
    }
}
/// a(b(p)), using the SDK's explicitly ordered composition API.
pub fn compose(a: &Pose, b: &Pose) -> Result<Pose> {
    Ok(pose(auki_geometry::compose_spatial_transforms(
        &spatial(b),
        &spatial(a),
    )?))
}
pub fn inverse(p: &Pose) -> Result<Pose> {
    Ok(pose(auki_geometry::inverse_spatial_transform(&spatial(p))?))
}
pub(crate) fn rigid(p: &Pose, from: &str, to: &str) -> RigidTransform {
    let [x, y, z, w] = p.rotation_xyzw;
    RigidTransform {
        from_frame_id: from.into(),
        to_frame_id: to.into(),
        translation: p.translation,
        rotation_wxyz: [w, x, y, z],
    }
}
pub(crate) fn from_rigid(p: &RigidTransform) -> Pose {
    let [w, x, y, z] = p.rotation_wxyz;
    Pose {
        translation: p.translation,
        rotation_xyzw: [x, y, z, w],
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PortalCalibration {
    pub portal_id: String,
    pub portal_pose_in_shared: Pose,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Origin {
    pub kind: String,
    pub capture_timestamp_ns: u64,
    pub capture_clock: ClockReference,
    pub odometry_to_local: Pose,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub calibration: Option<PortalCalibration>,
}
/// Freeze the initial robot pose as the map origin. Portals never rebase it.
pub fn establish_origin(t: u64, clock: ClockReference, robot_to_odometry: &Pose) -> Result<Origin> {
    Ok(Origin {
        kind: "robot_boot_pose".into(),
        capture_timestamp_ns: t,
        capture_clock: clock,
        odometry_to_local: inverse(robot_to_odometry)?,
        calibration: None,
    })
}

/// T_shared_odometry = T_shared_portal * inverse(T_odometry_portal).
/// Different visible portals therefore yield the same canonical physical frame.
pub fn canonical_origin(
    t: u64,
    clock: ClockReference,
    marker_to_odometry: &Pose,
    reference: &QrAnchor,
) -> Result<Origin> {
    let target = from_rigid(&reference.pose_in_map);
    Ok(Origin {
        kind: "canonical_portals".into(),
        capture_timestamp_ns: t,
        capture_clock: clock,
        odometry_to_local: compose(&target, &inverse(marker_to_odometry)?)?,
        calibration: Some(PortalCalibration {
            portal_id: reference.anchor_id.clone(),
            portal_pose_in_shared: target,
        }),
    })
}
#[derive(Default)]
struct CameraChannel {
    scan_status: Value,
    exported: bool,
    component: Option<Component>,
    output: Option<ConfiguredObservable<VideoFrame>>,
    capture: Option<BufferProductCapture<VideoFrame>>,
    calibration: Option<auki_qr_localizer::Calibration>,
    key: Option<String>,
}
#[derive(Default, Serialize)]
struct ScanDiagnostics {
    received: u64,
    pose_matched: u64,
    expired_without_pose: u64,
    pending_overflow: u64,
    discarded_on_tracking_loss: u64,
    latest_received_timestamp_ns: Option<u64>,
}
pub struct LocalMap {
    pub frame: String,

    pub home_portals: Option<crate::home_alignment::HomePortals>,
    pub(crate) alignment_after_ns: Option<u64>,
    pub origin: Option<Origin>,
    pub detail: String,
    owner: String,
    sensor_owner: String,
    camera_frame: String,
    remote_sources: Option<(ProductReference, ProductReference)>,
    portal_origin: bool,
    required_portal: Option<String>,
    canonical_reference: Option<MapSnapshot>,
    clock: ClockReference,
    contract: PoseContract,
    odometry: auki_odometry::OdometryComponent,
    history: BufferProductCapture<auki_odometry::PoseUpdate>,
    _source_component: Component,
    lidar_output: ConfiguredObservable<SensorEvent>,
    camera_source: ConfiguredObservable<SensorEvent>,
    lidar_capture: BufferProductCapture<SensorEvent>,
    camera_source_capture: BufferProductCapture<SensorEvent>,
    right_source: Option<(
        ConfiguredObservable<SensorEvent>,
        BufferProductCapture<SensorEvent>,
    )>,
    cameras: BTreeMap<String, CameraChannel>,
    pub ar_frame: Option<crate::ar_preview::Frame>,
    detector: auki_qr_detector::QrDetector,
    processor: Component,
    landmark_output: ConfiguredObservable<crate::landmarks::LandmarkObservations>,
    landmark_capture: BufferProductCapture<crate::landmarks::LandmarkObservations>,
    map: MapComponent,
    runtime: ComponentRuntime,
    pub voxels: Option<VoxelSnapshot>,
    recent_voxels: crate::recent_voxels::RecentVoxels,
    retained_depth: Vec<RetainedDepth>,
    emit_scans: bool,
    copy_scans: bool,
    localized_scans: Vec<crate::collaborative_mapper::LocalizedScan>,
    _voxel_component: Option<VoxelMapComponent>,
    sizes: crate::portal_sizes::PortalSizes,
    evidence: BTreeMap<String, Value>,
    pending: Vec<Observation<SensorEvent>>,
    pub latest_pose: Option<(u64, Pose)>,
    correction: Option<(u64, i64)>,
    scan_diagnostics: ScanDiagnostics,
    last_integration: Option<std::time::Instant>,
    last_lidar_calibration: Option<Pose>,
}
impl LocalMap {
    pub fn new(runtime: &ComponentRuntime, owner: &str, session: &str) -> Result<Self> {
        let camera_frame = std::env::var("SPATIAL_MAPPING_CAMERA_FRAME")
            .unwrap_or_else(|_| crate::rgbd::FRAME.into());
        Self::new_for_robot(
            runtime,
            owner,
            session,
            FAMILY,
            &camera_frame,
            serde_json::from_str(include_str!("../config/portals.json"))?,
        )
    }
    pub fn new_for_robot(
        runtime: &ComponentRuntime,
        owner: &str,
        session: &str,
        family: &str,
        camera_frame: &str,
        sizes: crate::portal_sizes::PortalSizes,
    ) -> Result<Self> {
        ensure!(is_local_family(family), "unknown robot map family");
        let clock = crate::model::clock_ref(&crate::model::clock_entry(owner, session));
        let frame = format!("{family}:{session}:zup");
        let body_frame = |id: &str| {
            if family == BRACKETBOT_FAMILY {
                FrameRegistryEntry::in_convention(
                    owner,
                    id,
                    auki_registry::CoordinateConvention::ZUpRightForward,
                )
            } else {
                FrameRegistryEntry::ros_body(owner, id)
            }
        };
        let contract = PoseContract {
            from_frame: body_frame("robot.base"),
            to_frame: body_frame(&format!("robot.odometry.{session}")),
            clock: clock.clone(),
            session_id: session.into(),
        };
        let mut map = Self::with_contract(
            runtime,
            owner,
            session,
            frame,
            contract,
            None,
            false,
            None,
            FrameRegistryEntry::ros_optical(owner, camera_frame),
        )?;
        map.set_portal_sizes(sizes)?;
        Ok(map)
    }
    pub fn set_portal_sizes(&mut self, sizes: crate::portal_sizes::PortalSizes) -> Result<()> {
        sizes.validate()?;
        ensure!(
            self.origin.is_none() && self.evidence.is_empty(),
            "cannot replace sizes after mapping starts"
        );
        self.sizes = sizes;
        Ok(())
    }
    /// Home computes its own map from original, authenticated sensor observations.
    pub fn from_runner(
        runtime: &ComponentRuntime,
        owner: &str,
        session: &str,
        contract: PoseContract,
        camera: ProductReference,
        lidar: ProductReference,
        required_portal: Option<String>,
    ) -> Result<Self> {
        let camera_frame =
            FrameRegistryEntry::ros_optical(&contract.clock.peer_id, crate::rgbd::FRAME);
        Self::from_runner_with_camera(
            runtime,
            owner,
            session,
            contract,
            camera,
            lidar,
            required_portal,
            camera_frame,
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub fn from_runner_with_camera(
        runtime: &ComponentRuntime,
        owner: &str,
        session: &str,
        contract: PoseContract,
        camera: ProductReference,
        lidar: ProductReference,
        required_portal: Option<String>,
        camera_frame: FrameRegistryEntry,
    ) -> Result<Self> {
        contract.validate()?;
        ensure!(
            camera.peer_id == contract.clock.peer_id && lidar.peer_id == contract.clock.peer_id,
            "sensor source owner differs from capture clock"
        );
        Self::with_contract(
            runtime,
            owner,
            session,
            format!("home-live:{session}:zup"),
            contract,
            Some((camera, lidar)),
            true,
            required_portal,
            camera_frame,
        )
    }
    #[allow(clippy::too_many_arguments)]
    fn with_contract(
        runtime: &ComponentRuntime,
        owner: &str,
        session: &str,
        frame: String,
        contract: PoseContract,
        remote_sources: Option<(ProductReference, ProductReference)>,
        portal_origin: bool,
        required_portal: Option<String>,
        camera_frame: FrameRegistryEntry,
    ) -> Result<Self> {
        ensure!(
            crate::rgbd::supported_camera_frame(&camera_frame.frame_id)
                && camera_frame
                    == FrameRegistryEntry::ros_optical(
                        &contract.clock.peer_id,
                        &camera_frame.frame_id
                    ),
            "unsupported or misowned camera optical frame"
        );
        let clock = contract.clock.clone();
        let sensor_owner = contract.clock.peer_id.clone();
        let odometry =
            auki_odometry::OdometryComponent::new(runtime, "local-odometry", contract.clone())?;
        let history = auki_odometry::capture_pose_history(
            runtime,
            "local-odometry-history",
            &odometry,
            BufferLimits::entries(600),
        )?;
        let contract_for = |name: &str| ObservableContract {
            name: name.into(),
            datatype: SensorEvent::DATATYPE.into(),
            schema: SensorEvent::DATATYPE.into(),
            access: vec![ObservationAccess::FollowNew],
            exposure: Exposure::Cluster,
        };
        let mut source_spec = ComponentSpec::new("local-sensor-source")
            .observable(contract_for("lidar"))
            .observable(contract_for("camera"));
        if camera_frame.frame_id == crate::rgbd::BRACKETBOT_FRAME {
            source_spec = source_spec.observable(contract_for("camera_right"));
        }
        let source_component = runtime.component(source_spec)?;
        let output_for = |name: &str, frame: FrameRegistryEntry| -> Result<_> {
            Ok(source_component.configured_observable(
                ConfiguredObservableSpec::new(
                    name,
                    format!("{name}-{session}"),
                    clock.clone(),
                    PayloadContract::Structured(StructuredPayloadContract {
                        modality: "sensor_observation".into(),
                        datatype: SensorEvent::DATATYPE.into(),
                        schema: SensorEvent::DATATYPE.into(),
                        observes: name.into(),
                        unit: Some("meters".into()),
                    }),
                )
                .in_registered_frame(frame),
            )?)
        };
        let lidar_output = output_for(
            "lidar",
            FrameRegistryEntry::ros_body(owner, "lidar_base_link"),
        )?;
        let camera_source = output_for("camera", camera_frame.clone())?;
        let size = |s: &SensorEvent| serde_json::to_vec(s).map_or(usize::MAX, |s| s.len());
        let right_output = if camera_frame.frame_id == crate::rgbd::BRACKETBOT_FRAME {
            Some(output_for(
                "camera_right",
                FrameRegistryEntry::ros_optical(owner, crate::rgbd::BRACKETBOT_RIGHT_FRAME),
            )?)
        } else {
            None
        };
        source_component.expose()?;
        let right_source = if let Some(output) = right_output {
            let capture = runtime.capture_buffer(
                "local-camera-right-observations",
                &output,
                BufferLimits::entries(3),
                size,
            )?;
            Some((output, capture))
        } else {
            None
        };
        let lidar_capture = runtime.capture_buffer(
            "local-lidar-history",
            &lidar_output,
            BufferLimits::entries(3),
            size,
        )?;
        let camera_source_capture = runtime.capture_buffer(
            "local-camera-observations",
            &camera_source,
            BufferLimits::entries(3),
            size,
        )?;
        let writer = owner.to_owned();
        let map = MapComponent::new(
            runtime,
            MapComponentConfig {
                component_id: "observed-portals".into(),
                publication_id: session.into(),
                clock: clock.clone(),
                map: MapDefinition {
                    map_id: format!("observed-{session}"),
                    name: Some(
                        if portal_origin {
                            "Home observed portal map"
                        } else {
                            "Robot observed local map"
                        }
                        .into(),
                    ),
                    domain_reference: None,
                    frame: crate::conventions::frame(&frame)?,
                },
            },
            now,
            |_| true,
            move |caller| {
                caller.caller_peer_id == writer && caller.caller_component_id == "local-mapper"
            },
        )?;
        let processor = runtime.component(ComponentSpec::new("local-mapper").observable(
            ObservableContract {
                name: "landmark_observations".into(),
                datatype: crate::landmarks::OBSERVATIONS_SCHEMA.into(),
                schema: crate::landmarks::OBSERVATIONS_SCHEMA.into(),
                access: vec![ObservationAccess::FollowNew],
                exposure: Exposure::Cluster,
            },
        ))?;
        let landmark_output = processor
            .configured_observable::<crate::landmarks::LandmarkObservations>(
                ConfiguredObservableSpec::new(
                    "landmark_observations",
                    format!("portal-corners-{session}"),
                    clock.clone(),
                    PayloadContract::Structured(StructuredPayloadContract {
                        modality: "detection".into(),
                        observes: "Named Portal corners in camera pixels".into(),
                        unit: Some("pixels".into()),
                        datatype: crate::landmarks::OBSERVATIONS_SCHEMA.into(),
                        schema: crate::landmarks::OBSERVATIONS_SCHEMA.into(),
                    }),
                ),
            )?;
        processor.expose()?;
        let landmark_capture = runtime.capture_buffer(
            "landmark-observation-history",
            &landmark_output,
            BufferLimits::entries(2),
            |batch: &crate::landmarks::LandmarkObservations| {
                serde_json::to_vec(batch).map_or(usize::MAX, |b| b.len())
            },
        )?;
        Ok(Self {
            frame,

            home_portals: None,
            alignment_after_ns: None,
            origin: None,
            detail: if portal_origin {
                "Waiting for UCOF1J40N7O in raw camera observations"
            } else {
                "Waiting for first valid robot pose"
            }
            .into(),
            owner: owner.into(),
            sensor_owner,
            camera_frame: camera_frame.frame_id,
            remote_sources,
            portal_origin,
            required_portal,
            canonical_reference: None,
            clock,
            contract,
            odometry,
            history,
            _source_component: source_component,
            lidar_output,
            camera_source,
            lidar_capture,
            camera_source_capture,
            right_source,
            cameras: BTreeMap::new(),
            ar_frame: None,
            detector: auki_qr_detector::QrDetector::new(Default::default()),
            processor,
            landmark_output,
            landmark_capture,
            map,
            runtime: runtime.clone(),
            voxels: None,
            recent_voxels: Default::default(),
            retained_depth: Vec::new(),
            emit_scans: false,
            localized_scans: Vec::new(),
            copy_scans: false,
            _voxel_component: None,
            sizes: serde_json::from_str(include_str!("../config/portals.json"))?,
            evidence: BTreeMap::new(),
            pending: vec![],
            latest_pose: None,
            correction: None,
            scan_diagnostics: Default::default(),
            last_integration: None,
            last_lidar_calibration: None,
        })
    }
    /// Home uses only the localization stage; one shared writer integrates rays.
    pub fn emit_localized_scans(&mut self) {
        self.emit_scans = true;
    }
    /// Export bounded raw scans without changing native mapping or its origin.
    pub fn copy_native_scans(&mut self) { self.copy_scans = true; }
    pub fn calibrate_against(&mut self, canonical: &MapSnapshot) -> Result<()> {
        ensure!(
            self.origin.is_none(),
            "cannot replace alignment after mapping starts"
        );
        self.canonical_reference = Some(crate::collaborative_mapper::reference_in_frame(
            canonical,
            &self.frame,
        )?);
        self.required_portal = None;
        self.detail = "Waiting for any portal in Home's canonical record".into();
        Ok(())
    }
    pub fn take_localized_scans(&mut self) -> Vec<crate::collaborative_mapper::LocalizedScan> {
        std::mem::take(&mut self.localized_scans)
    }
    pub fn export_sources(
        &self,
        endpoint: &auki_component_protocol::ComponentProtocolEndpoint,
    ) -> Result<()> {
        endpoint.export_product(&self.history.product())?;
        endpoint.export_product(&self.lidar_capture.product())?;
        endpoint.export_product(&self.camera_source_capture.product())?;
        if let Some((_, capture)) = &self.right_source {
            endpoint.export_product(&capture.product())?;
        }
        endpoint.export_product(&self.map.product())?;
        endpoint.export_product(&self.landmark_capture.product())?;
        Ok(())
    }
    pub fn export_camera_preview(
        &mut self,
        endpoint: &auki_component_protocol::ComponentProtocolEndpoint,
    ) -> Result<bool> {
        for channel in self.cameras.values_mut() {
            if !channel.exported
                && let Some(capture) = &channel.capture
            {
                endpoint.export_product(&capture.product())?;
                channel.exported = true;
            }
        }
        Ok(!self.cameras.is_empty())
    }
    pub fn portals(&self) -> Result<MapSnapshot> {
        let mut snapshot = (*self
            .map
            .product()
            .latest_existing()?
            .ok_or_else(|| anyhow!("map snapshot missing"))?
            .payload)
            .clone();
        // The SDK store retains historical observations. The published scene
        // is the active mapping epoch, excluding invalidated placements until
        // they are measured again in the corrected trajectory.
        if self.correction.is_some() {
            snapshot.scenegraph.anchors.retain(|id, _| self.evidence.contains_key(id));
            snapshot = MapSnapshot::new(snapshot.scenegraph)?;
        }
        Ok(snapshot)
    }
    pub fn accept_home(&mut self, scene: crate::network::CanonicalScene) -> Result<()> {
        self.accept_home_portals(scene.portals, scene.authority_peer_id)
    }
    pub fn accept_home_portals(&mut self, portals: MapSnapshot, peer_id: String) -> Result<()> {
        let observed = self.portals()?;
        let anchor = self
            .latest_shared_portal(&portals)
            .ok_or_else(|| anyhow!("No observed portal is known to Home"))?;
        let home = crate::home_alignment::HomePortals {
            portals,
            peer_id,

            anchor,
        };
        home.portals.validate()?;
        crate::conventions::validate_frame(&home.portals.scenegraph.map.frame)?;
        ensure!((home.portals.scenegraph.anchors[&home.anchor].side_length_m
            - observed.scenegraph.anchors[&home.anchor].side_length_m).abs() < 1e-6,
            "alignment portal sizes differ");
        self.home_portals = Some(home);
        Ok(())
    }
    fn latest_shared_portal(&self, reference: &MapSnapshot) -> Option<String> {
        self.evidence
            .iter()
            .filter(|(id, _)| reference.scenegraph.anchors.contains_key(*id))
            .filter_map(|(id, evidence)| evidence["capture_timestamp_ns"].as_u64().map(|t| (t, id)))
            .max()
            .map(|(_, id)| id.clone())
    }
    pub fn metadata(&self) -> Value {
        json!({"local_mapping":{"status":if self.latest_pose.is_some(){"mapping"}else{"waiting_for_odometry"},"detail":self.detail,"origin":self.origin,
            "scan_progress":self.scan_progress(),
            "map_correction":self.correction.map(|(t,n)|json!({"capture_timestamp_ns":t,"pgo_count":n,"policy":"reproject_calibration_when_history_available",
                "alignment_reprojected":self.home_portals.as_ref().is_some_and(|h|self.evidence[&h.anchor]["trajectory_correction_ns"].as_u64()==Some(t))})),
            "voxel_policy":{"kind":"latest_observations_per_cell","window":3,"free_space":"SDK measured rays only","unseen_cells":"retained"},"alignment_to_home":null,"portal_poses":"observed","portal_color":"blue","portal_axes":"+X printed right; +Y printed top; +Z out of marker",
            "observed_portals":self.evidence,
            "retained_depth_timestamps":self.retained_depth.iter().map(|d|d.observation.timestamp_ns).collect::<std::collections::BTreeSet<_>>(),
            "camera_scans":self.cameras.iter().map(|(frame,c)| (frame.clone(), c.scan_status.clone())).collect::<BTreeMap<_,_>>(),
            "robot_pose":self.latest_pose.as_ref().map(|(t,p)|json!({"capture_timestamp_ns":t,"clock":self.clock,"pose":p,"from_frame":self.contract.from_frame,"frame":self.frame}))},
            "landmark_components":[crate::landmarks::ComponentCapabilities { component: self.processor.reference().clone(), landmark_types: Some(vec![crate::landmarks::PORTAL.into()]) }],
            "landmark_observations":self.landmark_capture.product().reference(),
            "frames":[self.contract.from_frame,self.contract.to_frame],"clocks":[crate::model::clock_entry(&self.sensor_owner,&self.contract.session_id)]})
    }
    pub fn scan_progress(&self) -> Value {
        let age = self.last_integration.map(|t| t.elapsed().as_millis() as u64);
        json!({
            "status": if self.latest_pose.is_none() { "tracking_unavailable" }
                else if age.is_none() { "waiting_for_first_scan" }
                else if age.is_some_and(|ms| ms > 3000) { "stalled" } else { "integrating" },
            "integrated_observations": self.voxels.as_ref().map_or(0, |v| v.integrated_observations),
            "last_integrated_timestamp_ns": self.voxels.as_ref().and_then(|v| v.latest_observation.as_ref()).map(|o| o.timestamp_ns),
            "last_integration_age_ms": age,
            "pending": self.pending.len(), "diagnostics": self.scan_diagnostics
        })
    }
    pub fn landmarks(&self, portals: &MapSnapshot) -> Result<crate::landmarks::Landmarks> {
        crate::landmarks::portal_map(
            portals,
            Some(self.processor.reference().clone()),
            &crate::landmarks::portal_evidence(&self.metadata())?,
        )
    }
    pub fn landmark_product(&self) -> RetainedProduct<crate::landmarks::LandmarkObservations> {
        self.landmark_capture.product()
    }
    pub fn accept(&mut self, event: SensorEvent) -> Result<()> {
        match event {
            SensorEvent::CorrectionPending { pgo_count, .. } => {
                if self.frame.starts_with(&format!("{BRACKETBOT_FAMILY}:")) { return Ok(()); }
                self.pending.clear();self.localized_scans.clear();self.latest_pose=None;
                self.detail=format!("PGO {pgo_count} · waiting for optimized history; geometry retained");
            }
            SensorEvent::TrajectoryCorrection { timestamp_ns, pgo_count, pose, corrected_portals, corrected_depth } => {
                if self.frame.starts_with(&format!("{BRACKETBOT_FAMILY}:")) {
                    return self.accept(SensorEvent::Odometry{timestamp_ns,pose});
                }
                ensure!(corrected_portals.len() <= 256, "Correction landmark limit");
                ensure!(corrected_depth.len() <= 2048, "Correction depth history limit");
                let retained = std::mem::take(&mut self.retained_depth);
                let count = self.voxels.as_ref().map(|v|v.integrated_observations).unwrap_or(0);
                let portals = self.portals()?;
                let evidence = self.evidence.clone();
                let home = self.home_portals.take();
                let origin = self.origin.as_ref().map(|o|o.odometry_to_local.clone());
                self.accept(SensorEvent::MapCorrection { timestamp_ns, pgo_count, pose })?;
                if let Some(origin) = origin {
                    for (id, corrected) in corrected_portals {
                        let Some(old) = evidence.get(&id) else { continue; };
                        if old["capture_timestamp_ns"].as_u64() != Some(corrected.timestamp_ns) { continue; }
                        let Some(anchor) = portals.scenegraph.anchors.get(&id) else { continue; };
                        let Ok(base) = serde_json::from_value::<Pose>(old["base_odometry_pose"].clone()) else { continue; };
                        corrected.pose.validate()?;
                        let delta = compose(&compose(&origin, &corrected.pose)?, &inverse(&compose(&origin, &base)?)?)?;
                        let placement = compose(&delta, &from_rigid(&anchor.pose_in_map))?;
                        let mut corrected_anchor = anchor.clone();
                        corrected_anchor.pose_in_map = rigid(&placement, &anchor.pose_in_map.from_frame_id, &self.frame);
                        InMemoryTransport.invoke(self.map.upsert_qr(), InvocationContext {
                            invocation_id:uuid::Uuid::new_v4().to_string(),caller_peer_id:self.owner.clone(),caller_component_id:"local-mapper".into()
                        }, UpsertQr { expected_snapshot:self.map.snapshot_reference(),anchor:corrected_anchor }).map_err(|e|anyhow!("{e:?}"))?;
                        let mut updated = old.clone();
                        updated["base_odometry_pose"] = json!(corrected.pose);
                        updated["latest_measured_pose"] = json!(placement);
                        updated["trajectory_correction_ns"] = json!(timestamp_ns);
                        self.evidence.insert(id, updated);
                    }
                }
                if let Some(home) = home && self.evidence.contains_key(&home.anchor) {
                    self.home_portals = Some(home);
                    self.detail = format!("SLAM correction {pgo_count} · calibration reprojected from optimized history");
                }
                self.rebuild_depth(retained, &corrected_depth, count)?;
            }
            SensorEvent::MapCorrection { timestamp_ns, pgo_count, pose } => {
                if self.frame.starts_with(&format!("{BRACKETBOT_FAMILY}:")) {
                    return self.accept(SensorEvent::Odometry{timestamp_ns,pose});
                }
                pose.validate()?;
                ensure!(timestamp_ns > 0 && self.correction.is_none_or(|(t,_)| timestamp_ns > t), "out-of-order map correction");
                self.pending.clear();
                self.localized_scans.clear();
                self.evidence.clear();
                self.home_portals = None;
                self.ar_frame = None;
                self.recent_voxels = Default::default();
                self.retained_depth.clear();
                self.last_integration = None;
                self.correction = Some((timestamp_ns, pgo_count));
                for channel in self.cameras.values_mut() { channel.scan_status = json!({"status":"awaiting_corrected_scan"}); }
                if let Some(saved) = &mut self.voxels {
                    saved.checkpoint = auki_datatypes::map::MapUpdate {
                        voxel_chunks: vec![],
                        checkpoint: Some(Default::default()),
                    }.encode_to_vec();
                    saved.integrated_observations = 0;
                    saved.latest_observation = None;
                }
                // Explicit lost bracket prevents interpolation across the jump.
                self.odometry.publish(timestamp_ns-1, Tracking::Lost)?;
                self.accept(SensorEvent::Odometry { timestamp_ns, pose })?;
                self.detail = format!("SLAM correction {pgo_count} · local geometry invalidated; connection retained");
            }
            SensorEvent::TrackingLost { timestamp_ns } => {
                // BBOS recovers into the same SLAM map. A missing pose pauses
                // localization, but does not change the map-to-Home transform.
                // MapCorrection still clears all portal/alignment evidence.
                if !self.frame.starts_with(&format!("{BRACKETBOT_FAMILY}:")) {
                    self.alignment_after_ns = Some(timestamp_ns);
                }
                self.odometry.publish(timestamp_ns, Tracking::Lost)?;
                self.latest_pose = None;
                self.scan_diagnostics.discarded_on_tracking_loss += self.pending.len() as u64;
                self.pending.clear();
                self.detail = "Tracking lost · mapping paused".into();
            }
            SensorEvent::Odometry { timestamp_ns, pose } => {
                if self.correction.is_some_and(|(boundary, _)| timestamp_ns < boundary) {
                    return Ok(());
                }
                pose.validate()?;
                self.odometry
                    .publish(timestamp_ns, Tracking::Valid { pose: pose.clone() })?;
                if self.origin.is_none() && !self.portal_origin {
                    self.origin = Some(establish_origin(timestamp_ns, self.clock.clone(), &pose)?);
                    self.initialize_voxels()?;
                    self.detail = "Robot boot origin · mapping measured geometry".into();
                }
                if let Some(origin) = &self.origin {
                    self.latest_pose =
                        Some((timestamp_ns, compose(&origin.odometry_to_local, &pose)?));
                }
            }
            SensorEvent::Status { detail } => self.detail = detail,
            event => {
                let t = match &event {
                    SensorEvent::Camera { sample } => sample.timestamp_ns,
                    SensorEvent::Lidar { timestamp_ns, .. } => *timestamp_ns,
                    _ => unreachable!(),
                };
                if self.correction.is_some_and(|(boundary, _)| t < boundary) {
                    return Ok(());
                }
                self.scan_diagnostics.received += 1;
                self.scan_diagnostics.latest_received_timestamp_ns = Some(t);
                let right = matches!(&event, SensorEvent::Camera { sample } if sample.calibration.sensor_frame == crate::rgbd::BRACKETBOT_RIGHT_FRAME);
                let output = if right {
                    &self
                        .right_source
                        .as_ref()
                        .ok_or_else(|| anyhow!("right camera not configured"))?
                        .0
                } else if matches!(&event, SensorEvent::Camera { .. }) {
                    &self.camera_source
                } else {
                    &self.lidar_output
                };
                let (o, _) = output.publish(t, Arc::new(event))?;
                ensure!(
                    self.lidar_capture.errors().is_empty()
                        && self.camera_source_capture.errors().is_empty(),
                    "sensor retention failed"
                );
                ensure!(
                    self.right_source
                        .as_ref()
                        .is_none_or(|(_, c)| c.errors().is_empty()),
                    "right camera retention failed"
                );
                if self.pending.len() >= 8 {
                    self.scan_diagnostics.pending_overflow += 1;
                    self.pending.remove(0);
                }
                self.pending.push(o);
            }
        }
        self.drain()
    }
    pub fn accept_remote(&mut self, observation: Observation<SensorEvent>) -> Result<()> {
        let t = match &*observation.payload {
            SensorEvent::Camera { sample } => sample.timestamp_ns,
            SensorEvent::Lidar { timestamp_ns, .. } => *timestamp_ns,
            _ => anyhow::bail!("unexpected remote sensor event"),
        };
        ensure!(
            t == observation.timestamp_ns,
            "sensor envelope timestamp mismatch"
        );
        if self.pending.len() >= 8 {
            self.pending.remove(0);
        }
        self.pending.push(observation);
        self.drain()
    }
    pub fn accept_pose(
        &mut self,
        timestamp_ns: u64,
        update: &auki_odometry::PoseUpdate,
    ) -> Result<()> {
        ensure!(
            update.contract == self.contract,
            "odometry session/frame changed"
        );
        match &update.tracking {
            Tracking::Valid { pose } => self.accept(SensorEvent::Odometry {
                timestamp_ns,
                pose: pose.clone(),
            }),
            Tracking::Lost => {
                self.accept(SensorEvent::TrackingLost { timestamp_ns })
            }
        }
    }
    fn camera_reference(&self) -> ProductReference {
        self.remote_sources
            .as_ref()
            .map(|s| s.0.clone())
            .unwrap_or_else(|| self.camera_source_capture.product().reference())
    }
    fn lidar_reference(&self) -> ProductReference {
        self.remote_sources
            .as_ref()
            .map(|s| s.1.clone())
            .unwrap_or_else(|| self.lidar_capture.product().reference())
    }
    pub fn ar_pose_history(&self) -> crate::ar_preview::PoseHistory {
        crate::ar_preview::PoseHistory {
            product: self.history.product(),
            contract: self.contract.clone(),
        }
    }
    fn at(&self, t: u64) -> Result<Option<Pose>> {
        if self.correction.is_some_and(|(boundary, _)| t < boundary) {
            return Ok(None);
        }
        match pose_at(
            &self.history.product(),
            &PoseQuery {
                contract: self.contract.clone(),
                timestamp_ns: t,
                max_gap_ns: 200_000_000,
            },
        ) {
            Ok(p) => Ok(Some(p.pose)),
            Err(
                auki_odometry::PoseError::OutsideHistory
                | auki_odometry::PoseError::GapTooLarge
                | auki_odometry::PoseError::TrackingLost,
            ) => Ok(None),
            Err(e) => Err(e.into()),
        }
    }
    fn drain(&mut self) -> Result<()> {
        let pending = std::mem::take(&mut self.pending);
        for o in pending {
            let t = match &*o.payload {
                SensorEvent::Camera { sample } => sample.timestamp_ns,
                SensorEvent::Lidar { timestamp_ns, .. } => *timestamp_ns,
                _ => continue,
            };
            let Some(base) = self.at(t)? else {
                if self
                    .history
                    .product()
                    .latest_existing()?
                    .is_none_or(|last| last.timestamp_ns < t + 2_000_000_000)
                {
                    self.pending.push(o);
                } else {
                    self.scan_diagnostics.expired_without_pose += 1;
                }
                continue;
            };
            self.scan_diagnostics.pose_matched += 1;
            match &*o.payload {
                SensorEvent::Camera { sample } => self.camera(sample, &base, o.sequence)?,
                SensorEvent::Lidar {
                    reading,
                    sensor_to_base,
                    ..
                } => {
                    if self.origin.is_none() {
                        continue;
                    }
                    // Keep the existing stationary-base LiDAR admission gate.
                    let (Some(a), Some(b)) = (
                        self.at(t.saturating_sub(150_000_000))?,
                        self.at(t + 150_000_000)?,
                    ) else {
                        self.pending.push(o);
                        continue;
                    };
                    let distance = a
                        .translation
                        .iter()
                        .zip(b.translation)
                        .map(|(a, b)| (a - b).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    let dot = a
                        .rotation_xyzw
                        .iter()
                        .zip(b.rotation_xyzw)
                        .map(|(a, b)| a * b)
                        .sum::<f64>()
                        .abs()
                        .min(1.);
                    if distance >= 0.01 || 2. * dot.acos() >= 0.0174533 {
                        continue;
                    }
                    sensor_to_base.validate()?;
                    if let Some(old) = &self.last_lidar_calibration {
                        ensure!(
                            old.translation
                                .iter()
                                .zip(sensor_to_base.translation)
                                .all(|(a, b)| (a - b).abs() < 1e-8)
                                && old
                                    .rotation_xyzw
                                    .iter()
                                    .zip(sensor_to_base.rotation_xyzw)
                                    .all(|(a, b)| (a - b).abs() < 1e-8),
                            "LiDAR mounting transform changed"
                        );
                    }
                    self.last_lidar_calibration = Some(sensor_to_base.clone());
                    let cloud = crate::runner_voxels::decode_colored(reading, t)?;
                    self.integrate(
                        t,
                        o.sequence,
                        &base,
                        sensor_to_base,
                        FrameRegistryEntry::ros_body(&self.sensor_owner, "lidar_base_link"),
                        cloud.points,
                        cloud.colors,
                    )?;
                }
                _ => {}
            }
        }
        Ok(())
    }
    fn camera(&mut self, s: &CameraSample, base: &Pose, sequence: u64) -> Result<()> {
        ensure!(
            s.calibration.sensor_frame == self.camera_frame
                || (self.camera_frame == crate::rgbd::BRACKETBOT_FRAME
                    && s.calibration.sensor_frame == crate::rgbd::BRACKETBOT_RIGHT_FRAME),
            "unexpected camera frame"
        );
        let frame = s.calibration.sensor_frame.clone();
        let mut channel = self.cameras.remove(&frame).unwrap_or_default();
        let result = self.camera_in_channel(s, base, sequence, &mut channel);
        self.cameras.insert(frame, channel);
        result
    }
    fn camera_in_channel(
        &mut self,
        s: &CameraSample,
        base: &Pose,
        sequence: u64,
        channel: &mut CameraChannel,
    ) -> Result<()> {
        ensure!(
            s.timestamp_ns == s.rgb_timestamp_ns && s.timestamp_ns == s.transform_timestamp_ns,
            "camera timestamp mismatch"
        );
        // A delayed image cannot roll canonical localization back across a
        // later portal correction. The physical map frame itself stays fixed.
        if self.canonical_reference.is_some()
            && self
                .origin
                .as_ref()
                .is_some_and(|o| s.timestamp_ns < o.capture_timestamp_ns)
        {
            return Ok(());
        }
        s.calibration.validate()?;
        ensure!(
            (64..=4096).contains(&s.width) && (64..=4096).contains(&s.height),
            "unexpected camera frame/shape"
        );
        let key = serde_json::to_string(&(
            s.width,
            s.height,
            s.fx,
            s.fy,
            s.cx,
            s.cy,
            &s.calibration.calibration_source,
        ))?;
        if let Some(previous) = &channel.key {
            ensure!(
                previous == &key,
                "camera intrinsics changed; new local session required"
            );
        }
        let bytes = STANDARD.decode(&s.gray_base64)?;
        ensure!(
            bytes.len() == s.width as usize * s.height as usize,
            "camera buffer size mismatch"
        );
        if channel.output.is_none() {
            let name = if s.calibration.sensor_frame == self.camera_frame {
                "local-camera"
            } else {
                "local-camera-right"
            };
            let c = self.runtime.component(ComponentSpec::new(name).observable(
                ObservableContract {
                    name: "frames".into(),
                    datatype: VideoFrame::DATATYPE.into(),
                    schema: "auki.video-frame/v1".into(),
                    access: vec![ObservationAccess::FollowNew],
                    exposure: Exposure::Cluster,
                },
            ))?;
            let output = c.configured_observable(
                ConfiguredObservableSpec::new(
                    "frames",
                    format!("camera-{}", self.contract.session_id),
                    self.clock.clone(),
                    PayloadContract::Camera(CameraPayloadContract {
                        datatype: VideoFrame::DATATYPE.into(),
                        schema: "auki.video-frame/v1".into(),
                        encoding: "luma8".into(),
                        width: s.width,
                        height: s.height,
                        nominal_frame_rate_hz: Some(1),
                        observes: format!("rectified {}", s.calibration.sensor_frame),
                    }),
                )
                .in_registered_frame(FrameRegistryEntry::ros_optical(
                    &self.owner,
                    &s.calibration.sensor_frame,
                )),
            )?;
            c.expose()?;
            let capture = self.runtime.capture_buffer(
                &format!("{name}-history"),
                &output,
                BufferLimits::entries(2),
                |f: &VideoFrame| f.bytes.len(),
            )?;
            channel.calibration = Some(auki_qr_localizer::Calibration::for_product(
                &s.calibration.sensor_frame,
                &capture.product(),
                auki_qr_localizer::Intrinsics {
                    fx: s.fx,
                    fy: s.fy,
                    cx: s.cx,
                    cy: s.cy,
                    distortion: auki_qr_localizer::Distortion::None,
                },
            )?);
            channel.component = Some(c);
            channel.output = Some(output);
            channel.capture = Some(capture);
            channel.key = Some(key);
        }
        let camera_product = channel.capture.as_ref().unwrap().product().reference();
        let (camera_observation, _) = channel.output.as_ref().unwrap().publish(
            s.timestamp_ns,
            Arc::new(VideoFrame {
                width: s.width,
                height: s.height,
                encoding: "luma8".into(),
                bytes: bytes.clone().into(),
            }),
        )?;
        let detections = self.detector.detect_luma(
            &bytes,
            s.width as usize,
            s.height as usize,
            s.width as usize,
        )?;
        let camera_source = crate::landmarks::SourceObservation {
            output: camera_observation.output,
            sequence: camera_observation.sequence,
            timestamp_ns: camera_observation.timestamp_ns,
            clock: self.clock.clone(),
        };
        let image_landmarks = detections
            .codes
            .iter()
            .filter(|code| !code.mirrored)
            .filter_map(|code| {
                portal_code(&code.payload).map(|identity| {
                    (
                        identity,
                        code.refined_corners_px.as_ref().unwrap_or(&code.corners_px),
                    )
                })
            })
            .flat_map(|(identity, corners)| {
                crate::landmarks::CORNERS
                    .into_iter()
                    .zip(corners)
                    .map(move |(feature, pixel)| crate::landmarks::ImageLandmark {
                        recognition: crate::landmarks::Recognition {
                            landmark_type: crate::landmarks::PORTAL.into(),
                            identity: format!("{identity}/{feature}"),
                        },
                        pixel: [pixel.x, pixel.y],
                    })
            })
            .collect();
        let (detection_observation, _) = self.landmark_output.publish(
            s.timestamp_ns,
            Arc::new(crate::landmarks::LandmarkObservations {
                schema: crate::landmarks::OBSERVATIONS_SCHEMA.into(),
                detected_by: self.processor.reference().clone(),
                source_observation: camera_source.clone(),
                landmarks: image_landmarks,
            }),
        )?;
        let provenance = crate::landmarks::Provenance {
            detected_by: Some(self.processor.reference().clone()),
            mapped_by: Some(self.processor.reference().clone()),
            source_observation: Some(crate::landmarks::SourceObservation {
                output: detection_observation.output,
                sequence: detection_observation.sequence,
                timestamp_ns: detection_observation.timestamp_ns,
                clock: self.clock.clone(),
            }),
            camera_observation: Some(camera_source),
        };
        let camera_odom = compose(base, &s.calibration.sensor_to_base)?;
        let mut accepted = 0;
        let decoded = detections.codes.len();
        channel.scan_status = json!({"timestamp_ns":s.timestamp_ns,"decoded_qrs":decoded,
            "mapped_portals":0,"rejections":[]});
        for code in detections.codes {
            if code.mirrored {
                channel.scan_status["rejections"].as_array_mut().unwrap().push(json!("mirrored QR"));
                continue;
            }
            let Some(short) = portal_code(&code.payload) else {
                channel.scan_status["rejections"].as_array_mut().unwrap().push(json!("not a Portal payload"));
                continue;
            };
            let Some(size) = self.sizes.encoded_size_m(short)? else {
                channel.scan_status["rejections"].as_array_mut().unwrap().push(json!({"portal":short,"reason":"printed size unknown"}));
                continue;
            };
            let snapshot = self.portals()?;
            let template = QrAnchor {
                anchor_id: short.into(),
                payload: code.payload.clone(),
                side_length_m: size,
                pose_in_map: rigid(&identity(), &format!("portal-{short}"), &self.frame),
            };
            let corners = code
                .refined_corners_px
                .as_ref()
                .unwrap_or(&code.corners_px)
                .map(|p| [p.x, p.y]);
            let estimate = match auki_qr_localizer::estimate_camera_pose(
                &snapshot.scenegraph.map,
                &template,
                corners,
                channel.calibration.as_ref().unwrap(),
                Default::default(),
            ) {
                Ok(e) => e,
                Err(e) => {
                    channel.scan_status["rejections"].as_array_mut().unwrap().push(json!({"portal":short,"reason":format!("localization: {e}")}));
                    continue;
                },
            };
            let marker_camera = inverse(&from_rigid(&estimate.camera_pose_in_map))?;
            if let Some(reference) = &self.canonical_reference
                && let Some(anchor) = reference.scenegraph.anchors.get(short)
                && self
                    .origin
                    .as_ref()
                    .is_none_or(|o| s.timestamp_ns > o.capture_timestamp_ns)
            {
                ensure!(
                    (anchor.side_length_m - size).abs() < 1e-8,
                    "canonical portal size differs from observation"
                );
                // Reacquire through any known portal, including after a manual
                // push that wheel odometry did not measure. Correct future
                // poses/observations; never rebase existing map geometry.
                self.origin = Some(canonical_origin(
                    s.timestamp_ns,
                    self.clock.clone(),
                    &compose(&camera_odom, &marker_camera)?,
                    anchor,
                )?);
            }
            if self.origin.is_none() && self.portal_origin {
                if self
                    .required_portal
                    .as_ref()
                    .is_some_and(|required| required != short)
                {
                    channel.scan_status["rejections"].as_array_mut().unwrap().push(json!({"portal":short,"reason":"not the required origin Portal"}));
                    continue;
                }
                let measured_marker = compose(&camera_odom, &marker_camera)?;
                self.origin = Some(if let Some(reference) = &self.canonical_reference {
                    let Some(anchor) = reference.scenegraph.anchors.get(short) else {
                        channel.scan_status["rejections"].as_array_mut().unwrap().push(json!({"portal":short,"reason":"not in canonical reference"}));
                        continue;
                    };
                    ensure!(
                        (anchor.side_length_m - size).abs() < 1e-8,
                        "canonical portal size differs from observation"
                    );
                    canonical_origin(s.timestamp_ns, self.clock.clone(), &measured_marker, anchor)?
                } else {
                    Origin {
                        kind: format!("portal:{short}"),
                        capture_timestamp_ns: s.timestamp_ns,
                        capture_clock: self.clock.clone(),
                        odometry_to_local: inverse(&measured_marker)?,
                        calibration: None,
                    }
                });
            }
            let origin = self
                .origin
                .as_ref()
                .ok_or_else(|| anyhow!("Waiting for robot odometry"))?;
            let measured = compose(
                &origin.odometry_to_local,
                &compose(&camera_odom, &marker_camera)?,
            )?;
            let placement = measured.clone();
            let anchor = QrAnchor {
                pose_in_map: rigid(&placement, &format!("portal-{short}"), &self.frame),
                ..template
            };
            InMemoryTransport
                .invoke(
                    self.map.upsert_qr(),
                    InvocationContext {
                        invocation_id: uuid::Uuid::new_v4().to_string(),
                        caller_peer_id: self.owner.clone(),
                        caller_component_id: "local-mapper".into(),
                    },
                    UpsertQr {
                        expected_snapshot: self.map.snapshot_reference(),
                        anchor,
                    },
                )
                .map_err(|e| anyhow!("{e:?}"))?;
            let n = self
                .evidence
                .get(short)
                .and_then(|v| v["observations"].as_u64())
                .unwrap_or(0)
                + 1;
            self.evidence.insert(short.into(),json!({"observations":n,"capture_timestamp_ns":s.timestamp_ns,"clock":self.clock,"camera_product":camera_product,"reprojection_rms_px":estimate.reprojection_rms_px,"latest_measured_pose":measured,"base_odometry_pose":base,"provenance":provenance}));
            accepted += 1;
            channel.scan_status["mapped_portals"] = json!(accepted);
        }
        if let Some(home) = &self.home_portals
            && let Some(anchor) = self.latest_shared_portal(&home.portals)
        {
            self.home_portals.as_mut().unwrap().anchor = anchor;
        }
        self.detail = if let Some(origin) = &self.origin {
            format!(
                "{} · {decoded} QRs decoded; {accepted} portals mapped in latest image",
                origin.kind
            )
        } else {
            "Waiting for a known-size portal to establish Home map origin".into()
        };
        if self.origin.is_some() {
            self.initialize_voxels()?;
            // A delayed image uses its capture-time pose for mapping, but must
            // not rewind the live robot pose supplied by newer odometry.
            if self.latest_pose.as_ref().is_none_or(|(t, _)| s.timestamp_ns >= *t) {
                self.latest_pose = Some((
                    s.timestamp_ns,
                    compose(&self.origin.as_ref().unwrap().odometry_to_local, base)?,
                ));
            }
            if self.emit_scans
                && self
                    .origin
                    .as_ref()
                    .is_some_and(|o| o.kind == "canonical_portals")
            {
                self.ar_frame = Some(crate::ar_preview::Frame {
                    source: camera_product.clone(),
                    clock: self.clock.clone(),
                    timestamp_ns: s.timestamp_ns,
                    sequence,
                    width: s.width,
                    height: s.height,
                    intrinsics: [s.fx, s.fy, s.cx, s.cy],
                    gray_base64: s.gray_base64.clone(),
                    camera_to_map: compose(
                        &self.origin.as_ref().unwrap().odometry_to_local,
                        &camera_odom,
                    )?,
                    origin: self.origin.clone().unwrap(),
                    received: std::time::Instant::now(),
                });
            }
            if !s.hit_points_m.is_empty() {
                let sample = crate::rgbd::Capture {
                    timestamp_ns: s.timestamp_ns,
                    rgb_timestamp_ns: s.rgb_timestamp_ns,
                    transform_timestamp_ns: s.transform_timestamp_ns,
                    hit_points_m: s.hit_points_m.clone(),
                    hit_colors_srgb: s.hit_colors_srgb.clone(),
                    calibration: s.calibration.clone(),
                };
                sample.validate_in_frame(&s.calibration.sensor_frame)?;
                self.integrate(
                    s.timestamp_ns,
                    sequence,
                    base,
                    &s.calibration.sensor_to_base,
                    FrameRegistryEntry::ros_optical(
                        &self.sensor_owner,
                        &s.calibration.sensor_frame,
                    ),
                    s.hit_points_m.clone(),
                    s.hit_colors_srgb.iter().copied().map(Some).collect(),
                )?;
            }
        }
        Ok(())
    }
    fn initialize_voxels(&mut self) -> Result<()> {
        if self.voxels.is_none() {
            let portals = self.portals()?;
            let source = self.camera_reference();
            let frame = FrameRegistryEntry::ros_optical(&self.sensor_owner, &self.camera_frame);
            let mut definition = VoxelMapDefinition::from_scene_map(
                &self.owner,
                "local-voxels",
                self.map.snapshot_reference(),
                &portals,
                crate::conventions::registry(&self.owner, &self.frame)?,
                source,
                frame,
                self.clock.clone(),
                0.1,
            )?;
            crate::voxel_color::enable(&mut definition);
            let component = VoxelMapComponent::new(
                &self.runtime,
                "local-voxel-mapper",
                &format!("local-voxels-{}", self.contract.session_id),
                self.clock.clone(),
                definition,
                now,
            )?;
            self.voxels = Some((*component.product().latest_existing()?.unwrap().payload).clone());
            self._voxel_component = Some(component);
        }
        Ok(())
    }
    #[allow(clippy::too_many_arguments)]
    fn integrate(
        &mut self,
        t: u64,
        sequence: u64,
        base: &Pose,
        sensor_base: &Pose,
        sensor_frame: FrameRegistryEntry,
        points: Vec<[f64; 3]>,
        colors: Vec<Option<[u8; 3]>>,
    ) -> Result<()> {
        let origin = self
            .origin
            .as_ref()
            .ok_or_else(|| anyhow!("no robot boot origin"))?;
        if t < origin.capture_timestamp_ns {
            return Ok(());
        }
        let sensor_local = compose(&origin.odometry_to_local, &compose(base, sensor_base)?)?;
        let source = if sensor_frame.frame_id == crate::rgbd::BRACKETBOT_RIGHT_FRAME
            && self.right_source.is_some()
        {
            self.right_source.as_ref().unwrap().1.product().reference()
        } else if sensor_frame.frame_id == self.camera_frame {
            self.camera_reference()
        } else {
            self.lidar_reference()
        };
        let binding = SensorBinding {
            sensor_product: source.clone(),
            sensor_frame: sensor_frame.clone(),
            observation_clock: self.clock.clone(),
        };
        self.initialize_voxels()?;
        let mut saved = self.voxels.as_ref().unwrap().clone();
        if !saved.definition.sources.contains(&binding) {
            saved.definition.sources.push(binding.clone());
        }
        let observation = DepthObservation {
            source,
            sequence,
            timestamp_ns: t,
            clock: self.clock.clone(),
            sensor_frame_id: sensor_frame.frame_id.clone(),
            hit_points_m: points,
            pose: TimedSensorPose {
                timestamp_ns: t,
                clock: self.clock.clone(),
                sensor_to_map: rigid(&sensor_local, &sensor_frame.frame_id, &self.frame),
                portal_snapshot: saved.definition.portal_snapshot.clone(),
            },
        };
        if self.copy_scans {
            if self.localized_scans.len() >= 32 { self.localized_scans.remove(0); }
            self.localized_scans.push(crate::collaborative_mapper::LocalizedScan {
                observation:observation.clone(),binding:binding.clone(),colors:colors.clone(),origin:self.origin.clone().unwrap(),
            });
        }
        if self.emit_scans {
            ensure!(
                self.localized_scans.len() < 32,
                "localized scan consumer fell behind"
            );
            self.localized_scans
                .push(crate::collaborative_mapper::LocalizedScan {
                    observation,
                    binding,
                    colors,
                    origin: self.origin.clone().unwrap(),
                });

            return Ok(());
        }
        let mut update =
            PortalVoxelMapper::new(saved.definition.clone())?.map_observation(&observation)?;
        crate::voxel_color::add(
            &mut update,
            &observation.hit_points_m,
            &colors,
            &spatial(&sensor_local),
            0.1,
        )?;
        // Multiple rays in one capture count once; an endpoint wins over a
        // crossing ray. Keep the last three captures that actually saw a cell.
        let endpoints = auki_mappers::Voxelizer::new(0.1, 16)?.map_sensor_points(
            observation
                .hit_points_m
                .iter()
                .map(|p| auki_datatypes::pose::Vec3 {
                    x: p[0],
                    y: p[1],
                    z: p[2],
                }),
            &spatial(&sensor_local),
            1.,
        )?;
        let mut recent = self.recent_voxels.clone();
        let checkpoint = recent.observe_at(&update, &endpoints, t)?;
        let mut accumulator = saved.accumulator()?;
        accumulator.apply(&checkpoint)?;
        saved.checkpoint = accumulator.checkpoint_update().encode_to_vec();
        saved.integrated_observations += 1;
        saved.latest_observation = Some((&observation).into());
        saved.accumulator()?;
        self.voxels = Some(saved);
        self.recent_voxels = recent;
        if !self.frame.starts_with(&format!("{BRACKETBOT_FAMILY}:")) {
        self.retained_depth.push(RetainedDepth { observation,colors,base_pose:base.clone(),sensor_to_base:sensor_base.clone() });
        let needed = self.recent_voxels.needed_timestamps();
        self.retained_depth.retain(|d|needed.contains(&d.observation.timestamp_ns));
        if self.retained_depth.len()>2048 { self.retained_depth.drain(..self.retained_depth.len()-2048); }
        }
        self.last_integration = Some(std::time::Instant::now());

        Ok(())
    }
    fn rebuild_depth(&mut self, retained:Vec<RetainedDepth>, corrected:&BTreeMap<u64,Pose>, count:u64) -> Result<()> {
        let Some(mut saved)=self.voxels.clone() else { return Ok(()); };
        let Some(origin)=self.origin.as_ref().map(|o|o.odometry_to_local.clone()) else { return Ok(()); };
        let mapper=PortalVoxelMapper::new(saved.definition.clone())?;
        let voxelizer=auki_mappers::Voxelizer::new(0.1,16)?;
        let mut recent=crate::recent_voxels::RecentVoxels::default();
        let mut replayed=Vec::new();
        for mut depth in retained {
            let Some(base)=corrected.get(&depth.observation.timestamp_ns) else { continue; };
            base.validate()?;
            let sensor=compose(&origin,&compose(base,&depth.sensor_to_base)?)?;
            depth.observation.pose.sensor_to_map=rigid(&sensor,&depth.observation.sensor_frame_id,&self.frame);
            let mut rays=mapper.map_observation(&depth.observation)?;
            crate::voxel_color::add(&mut rays,&depth.observation.hit_points_m,&depth.colors,&spatial(&sensor),0.1)?;
            let endpoints=voxelizer.map_sensor_points(depth.observation.hit_points_m.iter().map(|p|auki_datatypes::pose::Vec3{x:p[0],y:p[1],z:p[2]}),&spatial(&sensor),1.)?;
            recent.record(&rays,&endpoints,depth.observation.timestamp_ns)?;
            depth.base_pose=base.clone();
            replayed.push(depth);
        }
        if !replayed.is_empty() {
            let mut accumulator=saved.accumulator()?;
            accumulator.apply(&recent.checkpoint()?)?;
            saved.checkpoint=accumulator.checkpoint_update().encode_to_vec();
            saved.integrated_observations=count;
            saved.latest_observation=replayed.last().map(|d|(&d.observation).into());
            self.voxels=Some(saved);self.recent_voxels=recent;self.retained_depth=replayed;
            self.last_integration=Some(std::time::Instant::now());
        }
        Ok(())
    }
}
pub fn portal_code(payload: &str) -> Option<&str> {
    let (prefix, code) = payload.rsplit_once('/')?;
    (prefix.eq_ignore_ascii_case("https://r8.hr")
        && code.len() == 11
        && code
            .bytes()
            .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit()))
    .then_some(code)
}
pub fn now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}
