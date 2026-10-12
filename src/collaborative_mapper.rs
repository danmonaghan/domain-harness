//! One Home-owned voxel accumulator. Inputs are localized raw measurements,
//! never a peer's scenegraph or accumulated voxel evidence.
use crate::local_mapper::{Origin, from_rigid};
use anyhow::{Result, ensure};
use auki_components::*;
use auki_odometry::Pose;
use auki_scenegraph::{
    MapDefinition, MapSnapshot, QrAnchor,
    component::{MapComponent, MapComponentConfig, UpsertQr},
};
use auki_voxel_map::{
    DepthObservation, PortalVoxelMapper, SensorBinding, VoxelMapDefinition, VoxelSnapshot,
};
use prost::Message;
use serde_json::{Value, json};
use std::collections::BTreeMap;

pub const ORIGIN_PORTAL: &str = "UCOF1J40N7O";
/// Produced inside Home after capture-time localization in the measured portal
/// basis. Source Product, sequence and capture clock retain the robot identity.
#[derive(Clone)]
pub struct LocalizedScan {
    pub observation: DepthObservation,
    pub binding: SensorBinding,
    pub colors: Vec<Option<[u8; 3]>>,
    pub origin: Origin,
}
pub struct CollaborativeMap {
    pub frame: String,

    pub voxels: Option<VoxelSnapshot>,
    owner: String,
    clock: auki_components::clock::ClockRegistryEntry,
    map: MapComponent,
    reference: MapSnapshot,
    origin_portal: String,
    _runtime: ComponentRuntime,
    recent: crate::recent_voxels::RecentVoxels,
    watermarks: BTreeMap<String, (u64, u64)>,
    contributions: BTreeMap<String, u64>,
    /// Last validated relationship per peer/native frame/clock session. Never
    /// expires merely because a portal leaves view; no past scan is replayed.
    native_alignments: BTreeMap<(String,String,String), crate::scene_contribution::Contribution>,
    peers: BTreeMap<String, Value>,
    input_registries: BTreeMap<String, Value>,
    landmark_provenance: BTreeMap<String, crate::landmarks::Provenance>,
}
impl CollaborativeMap {
    pub fn new(owner: &str, session: &str, canonical: &MapSnapshot) -> Result<Self> {
        let frame = format!("home-live:{session}:zup");
        let definition = MapDefinition {
            map_id: format!("home-collaborative-{session}"),
            name: Some("Home collaborative sensor map".into()),
            domain_reference: None,
            frame: crate::conventions::frame(&frame)?,
        };
        Self::create(
            owner,
            session,
            definition,
            reference_in_frame(canonical, &frame)?,
            ORIGIN_PORTAL,
        )
    }

    /// A saved scene already declares its origin and OpenGL axes. No rebase,
    /// relabeling, or Xmart conversion is involved in this mapper.
    pub fn for_scene(owner: &str, session: &str, reference: &MapSnapshot) -> Result<Self> {
        reference.validate()?;
        let frame = &reference.scenegraph.map.frame.id;
        ensure!(
            crate::local_scenes::frame_scene_id(frame)
                == Some(reference.scenegraph.map.map_id.as_str()),
            "Independent scene identity mismatch"
        );
        let origin = frame.split_once(":portal-").unwrap().1;
        let anchor = reference
            .scenegraph
            .anchors
            .get(origin)
            .ok_or_else(|| anyhow::anyhow!("Scene origin Portal missing"))?;
        ensure!(
            anchor
                .pose_in_map
                .translation
                .iter()
                .all(|v| v.abs() < 1e-8),
            "Scene origin must be at zero"
        );
        let mut map = Self::create(
            owner,
            session,
            reference.scenegraph.map.clone(),
            reference.clone(),
            origin,
        )?;
        for anchor in reference.scenegraph.anchors.values() {
            map.upsert(anchor.clone())?;
        }
        Ok(map)
    }

    fn create(
        owner: &str,
        session: &str,
        definition: MapDefinition,
        reference: MapSnapshot,
        origin_portal: &str,
    ) -> Result<Self> {
        let runtime = ComponentRuntime::new(owner.to_owned());
        let clock = crate::model::clock_entry(owner, session);
        let frame = definition.frame.id.clone();
        let writer = owner.to_owned();
        let map = MapComponent::new(
            &runtime,
            MapComponentConfig {
                component_id: "collaborative-portals".into(),
                publication_id: session.into(),
                clock: crate::model::clock_ref(&clock),
                map: definition,
            },
            crate::local_mapper::now,
            |_| true,
            move |c| c.caller_peer_id == writer && c.caller_component_id == "home-fusion",
        )?;
        Ok(Self {
            frame,

            voxels: None,
            owner: owner.into(),
            clock,
            map,
            reference,
            origin_portal: origin_portal.into(),
            _runtime: runtime,
            recent: Default::default(),
            watermarks: BTreeMap::new(),
            contributions: BTreeMap::new(),
            native_alignments: BTreeMap::new(),
            peers: BTreeMap::new(),
            input_registries: BTreeMap::new(),
            landmark_provenance: BTreeMap::new(),
        })
    }
    fn upsert(&mut self, anchor: QrAnchor) -> Result<()> {
        InMemoryTransport
            .invoke(
                self.map.upsert_qr(),
                InvocationContext {
                    invocation_id: uuid::Uuid::new_v4().to_string(),
                    caller_peer_id: self.owner.clone(),
                    caller_component_id: "home-fusion".into(),
                },
                UpsertQr {
                    expected_snapshot: self.map.snapshot_reference(),
                    anchor,
                },
            )
            .map_err(|e| anyhow::anyhow!("{e:?}"))?;
        Ok(())
    }
    /// Add a newly reported Portal to this map and adopt the resulting snapshot
    /// as the calibration reference for later phone and robot input.
    pub fn add_portal(&mut self, anchor: QrAnchor) -> Result<MapSnapshot> {
        self.upsert(anchor)?;
        let snapshot = self.portals()?;
        self.reference = snapshot.clone();
        Ok(snapshot)
    }
    pub fn portals(&self) -> Result<MapSnapshot> {
        Ok((*self.map.product().latest_existing()?.unwrap().payload).clone())
    }
    /// Robot-local clocks are not comparable. De-duplicate independently per
    /// exact Product and clock; serialize accepted arrivals into one map writer.
    pub fn integrate(&mut self, scan: LocalizedScan) -> Result<bool> {
        let mut o = scan.observation;
        let peer = scan.binding.sensor_product.peer_id.clone();
        ensure!(
            scan.origin.kind == "canonical_portals",
            "runner has not aligned through canonical portal poses"
        );
        let calibration = scan
            .origin
            .calibration
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("canonical calibration missing"))?;
        let anchor = self
            .reference
            .scenegraph
            .anchors
            .get(&calibration.portal_id)
            .ok_or_else(|| anyhow::anyhow!("calibration portal unknown to Home"))?;
        let expected = from_rigid(&anchor.pose_in_map);
        ensure!(
            expected == calibration.portal_pose_in_shared,
            "calibration reference differs from Home canonical record"
        );
        ensure!(
            scan.origin.capture_clock == o.clock
                && scan.origin.capture_timestamp_ns <= o.timestamp_ns,
            "measurement predates this portal alignment or uses another clock"
        );
        ensure!(
            scan.binding.sensor_product == o.source
                && scan.binding.observation_clock == o.clock
                && scan.binding.sensor_frame.peer_id == peer
                && o.clock.peer_id == peer,
            "localized source provenance mismatch"
        );
        ensure!(
            o.pose.clock == o.clock
                && o.pose.timestamp_ns == o.timestamp_ns
                && o.pose.sensor_to_map.from_frame_id == scan.binding.sensor_frame.frame_id,
            "capture-time transform mismatch"
        );
        ensure!(
            if crate::local_scenes::frame_scene_id(&self.frame).is_some() {
                o.pose.sensor_to_map.to_frame_id == self.frame
            } else {
                o.pose.sensor_to_map.to_frame_id.starts_with("home-live:")
                    && o.pose.sensor_to_map.to_frame_id.ends_with(":zup")
            },
            "unexpected localization basis"
        );
        ensure!(
            scan.colors.len() == o.hit_points_m.len(),
            "color/point count differs"
        );
        let key = serde_json::to_string(&(o.source.clone(), o.clock.clone()))?;
        if self
            .watermarks
            .get(&key)
            .is_some_and(|(seq, t)| o.sequence <= *seq || o.timestamp_ns <= *t)
        {
            return Ok(false);
        }
        let mut saved = if let Some(saved) = &self.voxels {
            saved.clone()
        } else {
            let portals = self.portals()?;
            let mut definition = VoxelMapDefinition::from_scene_map(
                &self.owner,
                "home-collaborative-voxels",
                self.map.snapshot_reference(),
                &portals,
                crate::conventions::registry(&self.owner, &self.frame)?,
                o.source.clone(),
                scan.binding.sensor_frame.clone(),
                o.clock.clone(),
                0.1,
            )?;
            crate::voxel_color::enable(&mut definition);
            VoxelSnapshot {
                definition,
                checkpoint: auki_datatypes::map::MapUpdate {
                    voxel_chunks: vec![],
                    checkpoint: Some(Default::default()),
                }
                .encode_to_vec(),
                integrated_observations: 0,
                latest_observation: None,
            }
        };
        if let Some(binding) = saved
            .definition
            .sources
            .iter()
            .find(|b| b.sensor_product == o.source)
        {
            ensure!(
                binding == &scan.binding,
                "source binding changed without a new Product"
            );
        } else {
            saved.definition.sources.push(scan.binding);
        }
        // Both localization frames use canonical portal poses converted by the SDK.
        // Their different measured markers establish the same physical origin and axes.
        o.pose.sensor_to_map.to_frame_id = self.frame.clone();
        o.pose.portal_snapshot = saved.definition.portal_snapshot.clone();
        let p = from_rigid(&o.pose.sensor_to_map);
        let transform = spatial(&p);
        let mut rays = PortalVoxelMapper::new(saved.definition.clone())?.map_observation(&o)?;
        crate::voxel_color::add(&mut rays, &o.hit_points_m, &scan.colors, &transform, 0.1)?;
        let endpoints = auki_mappers::Voxelizer::new(0.1, 16)?.map_sensor_points(
            o.hit_points_m.iter().map(|p| auki_datatypes::pose::Vec3 {
                x: p[0],
                y: p[1],
                z: p[2],
            }),
            &transform,
            1.,
        )?;
        let mut recent = self.recent.clone();
        let checkpoint = recent.observe(&rays, &endpoints)?;
        let mut accumulator = saved.accumulator()?;
        accumulator.apply(&checkpoint)?;
        saved.checkpoint = accumulator.checkpoint_update().encode_to_vec();
        saved.integrated_observations += 1;
        saved.latest_observation = Some((&o).into());
        saved.accumulator()?;
        self.watermarks.insert(key, (o.sequence, o.timestamp_ns));
        *self.contributions.entry(peer.clone()).or_default() += 1;
        self.recent = recent;
        self.voxels = Some(saved);

        Ok(true)
    }
    pub fn integrate_phone(&mut self, envelope: crate::mobile_depth::Envelope) -> Result<bool> {
        let peer = envelope.source.peer_id.clone();
        let sample = envelope.sample.clone();
        let source = serde_json::json!({"capture_timestamp_ns":sample.timestamp_ns,"camera_to_session":sample.camera_to_session,"alignment":sample.alignment,
            "clock":envelope.clock,"sensor_frame":envelope.sensor_frame,"source":envelope.source});
        let scan = crate::mobile_depth::localize(
            envelope,
            &self.reference,
            self.map.snapshot_reference(),
            &self.frame,
        )?;
        let accepted = self.integrate(scan)?;
        if accepted {
            self.input_registries
                .insert(format!("phone:{peer}"), source);
            self.peer_status(
                &peer,
                "aligned",
                "Phone LiDAR · local camera measurements and separate alignment",
            );
        }
        Ok(accepted)
    }
    pub fn integrate_contribution(&mut self, mut contribution: crate::scene_contribution::Contribution) -> Result<bool> {
        contribution.validate(&contribution.source.peer_id,&self.reference)?;
        let key=(contribution.source.peer_id.clone(),contribution.native_frame.frame_id.clone(),contribution.clock.session_id.clone());
        ensure!(self.native_alignments.contains_key(&key)||self.native_alignments.len()<256,"Too many native frame relationships");
        if let Some(saved)=self.native_alignments.get(&key)
            && saved.alignment_timestamp_ns>=contribution.alignment_timestamp_ns {
            contribution.native_to_scene=saved.native_to_scene.clone();
            contribution.portal_to_native=saved.portal_to_native.clone();
            contribution.portal_id=saved.portal_id.clone();
            contribution.alignment_timestamp_ns=saved.alignment_timestamp_ns;
        }
        let peer = contribution.source.peer_id.clone();
        let provenance = serde_json::to_value(&contribution)?;
        let points=std::mem::take(&mut contribution.points);
        let colors=std::mem::take(&mut contribution.colors);
        let relationship=contribution.clone();
        contribution.points=points;contribution.colors=colors;
        let scan = contribution.localize(&self.reference, self.map.snapshot_reference())?;
        let accepted = self.integrate(scan)?;
        if accepted {
            self.native_alignments.insert(key,relationship);
            self.input_registries.insert(format!("peer:{peer}"), provenance);
            self.peer_status(&peer, "aligned", "Scene contribution · native measurements and separate alignment");
        }
        Ok(accepted)
    }
    pub fn peer_status(&mut self, peer: &str, status: &str, detail: &str) {
        let value = self.peers.entry(peer.into()).or_insert_with(|| json!({}));
        value["status"] = json!(status);
        value["detail"] = json!(detail);
    }
    pub fn peer_progress(
        &mut self,
        peer: &str,
        mut metadata: Value,
        portals: &MapSnapshot,
    ) -> Result<()> {
        let evidence = crate::landmarks::portal_evidence(&metadata)?;
        self.input_registries.insert(
            peer.into(),
            json!({"frames":metadata["frames"],"clocks":metadata["clocks"],"landmark_components":metadata["landmark_components"]}),
        );
        let local = &mut metadata["local_mapping"];
        if local["origin"]["kind"] == "canonical_portals" {
            if let Some(robot) = local.get_mut("robot_pose").filter(|r| !r.is_null()) {
                robot["frame"] = json!(self.frame);
            }
            for a in portals.scenegraph.anchors.values() {
                let mut a = a.clone();
                a.pose_in_map.to_frame_id = self.frame.clone();
                let id = a.anchor_id.clone();
                self.upsert(a)?;
                // Keep the evidence that belongs to the pose actually accepted
                // above; capture clocks from different peers aren't comparable.
                if let Some(provenance) = evidence.get(&id) {
                    self.landmark_provenance.insert(id, provenance.clone());
                } else {
                    self.landmark_provenance.remove(&id);
                }
            }
            local["status"] = json!("aligned");
        } else {
            local["status"] = json!("waiting_for_portal");
        }
        self.peers
            .insert(peer.into(), metadata["local_mapping"].clone());

        Ok(())
    }
    pub fn scene(&self) -> Result<Option<crate::network::CanonicalScene>> {
        let Some(voxels) = &self.voxels else {
            return Ok(None);
        };
        let robot = self
            .peers
            .values()
            .filter(|v| v["status"] == "aligned")
            .find_map(|v| v.get("robot_pose").filter(|v| !v.is_null()))
            .cloned();
        let mut scene = crate::network::CanonicalScene {
            schema: crate::network::SCENE_SCHEMA.into(),
            sdk_revision: crate::model::SDK_REV.into(),
            authority_peer_id: self.owner.clone(),

            frame: self.frame.clone(),
            publication_clock: self.clock.clone(),
            portals: self.portals()?,
            voxels: voxels.clone(),
            objects: vec![],
            registries: json!({"local_mapping":{"status":"mapping","detail":"Collaborative raw-observation mapping","origin":{"kind":"canonical_portals","portal":ORIGIN_PORTAL},"robot_pose":robot,"runners":self.peers,"alignment_to_home":null,"voxel_policy":{"kind":"latest_observations_per_cell","window":3,"ordering":"Home arrival order; capture clocks remain per peer"}},"collaborative_mapping":{"origin_portal":ORIGIN_PORTAL,"contributors":self.contributions,"runners":self.peers},"mapping_input":{"kind":"raw_runner_sensors","scenegraph_ingestion":false},"frames":[voxels.definition.frame],"clocks":[self.clock],"input_registries":self.input_registries,"source_bindings":voxels.definition.sources}),
        };
        scene.registries["local_mapping"]["origin"]["portal"] = json!(self.origin_portal);
        scene.registries["collaborative_mapping"]["origin_portal"] = json!(self.origin_portal);
        let mut landmarks = crate::landmarks::portal_map(
            &scene.portals,
            Some(self.map.component().reference().clone()),
            &self.landmark_provenance,
        )?;
        for input in self.input_registries.values() {
            if let Some(values) = input.get("landmark_components").filter(|v| !v.is_null()) {
                let capabilities: Vec<crate::landmarks::ComponentCapabilities> =
                    serde_json::from_value(values.clone())?;
                for capability in capabilities {
                    if !landmarks.components.contains(&capability) {
                        landmarks.components.push(capability);
                    }
                }
            }
        }
        scene.registries["landmarks"] = serde_json::to_value(landmarks)?;
        scene.validate(&self.owner)?;
        Ok(Some(scene))
    }
    pub fn detail(&self) -> String {
        let aligned = self
            .peers
            .values()
            .filter(|s| s["status"] == "aligned")
            .count();
        format!(
            "{aligned}/{} runners calibrated through canonical portals · {} contributing · raw sensor observations",
            self.peers.len(),
            self.contributions.len()
        )
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

/// The canonical record already has UCOF at zero. SDK conversion changes axes,
/// not physical origin. All peers receive this exact reference independently.
pub fn reference_in_frame(canonical: &MapSnapshot, target: &str) -> Result<MapSnapshot> {
    crate::conventions::validate_frame(&canonical.scenegraph.map.frame)?;
    ensure!(
        canonical.scenegraph.map.frame.id == crate::model::FRAME,
        "expected Home canonical portal frame"
    );
    let origin = canonical
        .scenegraph
        .anchors
        .get(ORIGIN_PORTAL)
        .ok_or_else(|| anyhow::anyhow!("canonical origin portal missing"))?;
    ensure!(
        origin
            .pose_in_map
            .translation
            .iter()
            .all(|v| v.abs() < 1e-8),
        "canonical origin portal must be at zero"
    );
    Ok(
        auki_scenegraph::MapConventionConversion::between_named_frames(
            canonical.scenegraph.map.frame.clone(),
            crate::conventions::frame(target)?,
        )?
        .convert_snapshot(canonical, "canonical-calibration-reference")?,
    )
}
