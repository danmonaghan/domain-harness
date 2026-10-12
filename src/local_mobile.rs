//! Scene-scoped phone collaboration through the standard SDK Catalog,
//! Products and operations. The optional voxel layer is absent until a real
//! measurement arrives; no placeholder sensor or synthetic observation.
use crate::{
    local_scenes::LocalScenes,
    network::{CanonicalScene, HomeConnection},
    notes::Notes,
};
use anyhow::{Result, anyhow, bail, ensure};
use auki_component_protocol::{
    CatalogResponse, ComponentProtocolClient, ComponentProtocolEndpoint, ObservationStart,
    RemoteObservationEvent,
};
use auki_components::*;
use auki_scenegraph::MapSnapshot;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

pub const SCHEMA: &str = "spatial-harness.mobile-scene/v1";
const MAX_BYTES: usize = 24 * 1024 * 1024;

#[derive(Clone, Serialize, Deserialize)]
pub struct Scene {
    pub schema: String,
    pub authority_peer_id: String,
    pub publication_clock: clock::ClockRegistryEntry,
    pub portals: MapSnapshot,
    pub geometry: Option<CanonicalScene>,
    pub notes: Notes,
    #[serde(default, skip_serializing_if = "Option::is_none", serialize_with="legacy_volume_layer")]
    pub semantic_volumes: Option<crate::semantic_volumes::Volumes>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub volumes: Vec<crate::semantic_volumes::SemanticVolume>,
    pub presence: Value,
    #[serde(default)]
    pub robot_presence: Value,
}
impl ContractType for Scene {
    const DATATYPE: &'static str = SCHEMA;
}
fn legacy_volume_layer<S:serde::Serializer>(volumes:&Option<crate::semantic_volumes::Volumes>,s:S)->std::result::Result<S::Ok,S::Error> {
    let mut value=serde_json::to_value(volumes).map_err(serde::ser::Error::custom)?;
    if let Some(items)=value["items"].as_array_mut() { for item in items { item.as_object_mut().unwrap().remove("semantic_type"); } }
    value.serialize(s)
}
fn size(s: &Scene) -> usize {
    serde_json::to_vec(s).map_or(usize::MAX, |v| v.len())
}
fn limits() -> BufferLimits {
    BufferLimits {
        max_entries: Some(1),
        max_bytes: Some(MAX_BYTES),
        target_duration: None,
    }
}

impl Scene {
    pub fn validate(&self, peer: &str, frame: &str) -> Result<()> {
        ensure!(
            self.schema == SCHEMA
                && self.authority_peer_id == peer
                && self.publication_clock.peer_id == peer,
            "Mobile scene authority/schema mismatch"
        );
        self.portals.validate()?;
        crate::conventions::validate_frame(&self.portals.scenegraph.map.frame)?;
        ensure!(
            self.portals.scenegraph.map.frame.id == frame
                && crate::local_scenes::frame_scene_id(frame)
                    == Some(self.portals.scenegraph.map.map_id.as_str())
                && self.portals.scenegraph.map.domain_reference.is_none(),
            "Mobile scene/profile mismatch"
        );
        self.notes.validate(frame)?;
        let mut volume_layer=Notes::empty(frame);
        volume_layer.volumes=self.volumes.clone();
        volume_layer.validate(frame)?;
        if let Some(volumes) = &self.semantic_volumes { volumes.validate(frame)?; }
        ensure!(
            self.presence["frame"] == frame,
            "Presence belongs to another scene"
        );
        if let Some(geometry) = &self.geometry {
            geometry.validate(peer)?;
            ensure!(
                geometry.frame == frame
                    && serde_json::to_value(&geometry.portals)?
                        == serde_json::to_value(&self.portals)?,
                "Geometry belongs to another reference scene"
            );
        }
        ensure!(
            size(self) <= MAX_BYTES,
            "Mobile scene exceeds publication limit"
        );
        Ok(())
    }
    pub fn view(&self) -> Result<Value> {
        let map = &self.portals.scenegraph.map;
        let mut view = if let Some(scene) = &self.geometry {
            crate::scene_mirror::render_scene(scene)?
        } else {
            json!({"frame":map.frame.id,"voxel_size":0.1,"voxels":[],"objects":[],"sources":{},"integrated_observations":0,
                "portals":self.portals.scenegraph.anchors.values().collect::<Vec<_>>(),
                "landmarks":crate::landmarks::portal_map(&self.portals,None,&Default::default())?})
        };
        view["notes"] = json!(self.notes.items);
        view["volumes"] = json!(self.volumes);
        let mut collection=self.semantic_volumes.clone().unwrap_or_else(|| {
            let mut collection=crate::semantic_volumes::Volumes::empty(&self.portals.scenegraph.map.frame.id);
            collection.items=self.volumes.iter().map(|v|crate::semantic_volumes::Volume::from_mobile(v,None)).collect();
            collection
        });
        for volume in &mut collection.items {
            if let Some(mobile)=self.volumes.iter().find(|v|v.id==volume.id) { volume.semantic_type=mobile.semantic_type.clone(); }
        }
        view["semantic_volumes"] = serde_json::to_value(collection)?;
        crate::mobile_pose::append_labels_in_frame(&mut view, &self.presence, &map.frame.id);
        crate::robot_presence::append_labels(&mut view, &self.robot_presence, &map.frame.id);
        view["environment_name"] = json!(map.name);
        view["frame_description"] = json!(map.frame.origin_description);
        view["local_reference"] = json!(true);
        view["mobile_enabled"] = json!(true);
        view["convention"] = json!("OpenGL · meters");
        view["authority"] = json!("Home · scene-specific peer mapper");
        view["mode"] = json!("Home-owned scenegraph");
        view["demo"] = json!(false);
        Ok(view)
    }
    fn mobile(&self) -> Result<Value> {
        let mut value = serde_json::to_value(&self.portals)?;
        value["render"] = self.view()?;
        Ok(value)
    }
}

struct State {
    map: crate::collaborative_mapper::CollaborativeMap,
    notes: Notes,
    presence: Arc<Mutex<crate::mobile_pose::Presence>>,
    robot_presence: Arc<Mutex<crate::robot_presence::Presence>>,
}
pub struct Publisher {
    id: String,
    /// Shared with the ingest operations: a phone-reported Portal replaces it.
    reference: Arc<std::sync::RwLock<MapSnapshot>>,
    state: Arc<Mutex<State>>,
    inbox: crate::mobile_depth::SharedInbox,
    contributions: Arc<Mutex<crate::scene_contribution::Inbox>>,
    scenes: Arc<LocalScenes>,
    output: ConfiguredObservable<Scene>,
    capture: BufferProductCapture<Scene>,
    clock: clock::ClockRegistryEntry,
    owner: String,
    started: Instant,
    last_tick: Option<Instant>,
    last_content: Option<Vec<u8>>,
    _components: Vec<Component>,
}
impl Publisher {
    pub fn mount_all(
        runtime: &ComponentRuntime,
        endpoint: &ComponentProtocolEndpoint,
        scenes: Arc<LocalScenes>,
        clock: clock::ClockRegistryEntry,
        started: Instant,
    ) -> Result<Vec<Self>> {
        scenes
            .snapshots
            .iter()
            .filter(|(id, s)| {
                crate::local_scenes::frame_scene_id(&s.scenegraph.map.frame.id) == Some(id.as_str())
            })
            .map(|(id, reference)| {
                Self::mount(
                    runtime,
                    endpoint,
                    scenes.clone(),
                    id,
                    reference.clone(),
                    clock.clone(),
                    started,
                )
            })
            .collect()
    }
    fn mount(
        runtime: &ComponentRuntime,
        endpoint: &ComponentProtocolEndpoint,
        scenes: Arc<LocalScenes>,
        id: &str,
        reference: MapSnapshot,
        clock: clock::ClockRegistryEntry,
        started: Instant,
    ) -> Result<Self> {
        let owner = runtime.peer_id().to_owned();
        let frame = reference.scenegraph.map.frame.id.clone();
        let notes_path = scenes
            .directory
            .join("mobile-notes")
            .join(format!("{id}.json"));
        let notes = if notes_path.exists() {
            ensure!(
                std::fs::metadata(&notes_path)?.len() <= 256 * 1024,
                "Scene notes exceed bounds"
            );
            serde_json::from_slice::<Notes>(&std::fs::read(&notes_path)?)?
        } else {
            Notes::empty(&frame)
        };
        notes.validate(&frame)?;
        scenes.volumes.import_notes(id,&frame,&notes.volumes)?;
        let live_presence = Arc::new(Mutex::new(crate::mobile_pose::Presence::default()));
        let robot_presence = Arc::new(Mutex::new(crate::robot_presence::Presence::default()));
        scenes.phone_presence.lock().map_err(|_| anyhow!("Phone presence unavailable"))?
            .insert(id.into(), live_presence.clone());
        let state = Arc::new(Mutex::new(State {
            map: crate::collaborative_mapper::CollaborativeMap::for_scene(
                &owner,
                &clock.session_id,
                &reference,
            )?,
            notes,
            presence: live_presence.clone(),
            robot_presence: robot_presence.clone(),
        }));
        let inbox = Arc::new(Mutex::new(crate::mobile_depth::Inbox::default()));
        let component = runtime.component(
            ComponentSpec::new(format!("mobile-scene:{id}")).observable(ObservableContract {
                name: "scene".into(),
                datatype: SCHEMA.into(),
                schema: SCHEMA.into(),
                access: vec![ObservationAccess::FollowNew],
                exposure: Exposure::Cluster,
            }),
        )?;
        let output = component.configured_observable(
            ConfiguredObservableSpec::new(
                "scene",
                uuid::Uuid::new_v4().to_string(),
                crate::model::clock_ref(&clock),
                PayloadContract::Structured(StructuredPayloadContract {
                    modality: "scenegraph".into(),
                    datatype: SCHEMA.into(),
                    schema: SCHEMA.into(),
                    observes: "selected_home_scene".into(),
                    unit: Some("meters".into()),
                }),
            )
            .in_registered_frame(crate::conventions::registry(&owner, &frame)?),
        )?;
        component.expose()?;
        let capture = runtime.capture_buffer(
            crate::local_scenes::mobile_product(id),
            &output,
            limits(),
            size,
        )?;
        endpoint.export_product(&capture.product())?;
        let mut components = vec![component];
        let contributions = Arc::new(Mutex::new(crate::scene_contribution::Inbox::default()));
        let receiver = runtime.component(ComponentSpec::new(format!("{}:{id}",crate::scene_contribution::COMPONENT)).operable(
            OperableContract{name:"ingest".into(),instruction:crate::scene_contribution::Contribution::DATATYPE.into(),result:"bool".into(),exposure:Exposure::Cluster}))?;
        let reference = Arc::new(std::sync::RwLock::new(reference));
        let queue=contributions.clone();let reference_copy=reference.clone();
        let operation=receiver.operable("ingest",|ctx|!ctx.caller_peer_id.is_empty(),move |ctx,c:crate::scene_contribution::Contribution|{
            queue.lock().map_err(|_|InvocationError::TargetUnavailable)?.accept(&ctx.caller_peer_id,c,&*reference_copy.read().map_err(|_|InvocationError::TargetUnavailable)?).map_err(rejected)
        })?;
        receiver.expose()?;endpoint.export_operable(&operation)?;components.push(receiver);

        let depth = runtime.component(
            ComponentSpec::new(format!("{}:{id}", crate::mobile_depth::COMPONENT)).operable(
                OperableContract {
                    name: "ingest".into(),
                    instruction: crate::mobile_depth::Envelope::DATATYPE.into(),
                    result: "bool".into(),
                    exposure: Exposure::Cluster,
                },
            ),
        )?;
        let depth_inbox = inbox.clone();
        let calibration = reference.clone();
        let op = depth.operable(
            "ingest",
            |ctx| !ctx.caller_peer_id.is_empty(),
            move |ctx, e: crate::mobile_depth::Envelope| {
                depth_inbox
                    .lock()
                    .map_err(|_| InvocationError::TargetUnavailable)?
                    .accept(
                        &ctx.caller_peer_id,
                        e,
                        &*calibration.read().map_err(|_| InvocationError::TargetUnavailable)?,
                    )
                    .map_err(rejected)
            },
        )?;
        depth.expose()?;
        endpoint.export_operable(&op)?;
        components.push(depth);

        let annotations = runtime.component(
            ComponentSpec::new(format!("{}:{id}", crate::mobile_notes::COMPONENT)).operable(
                OperableContract {
                    name: "place".into(),
                    instruction: crate::mobile_notes::PlaceNote::DATATYPE.into(),
                    result: crate::notes::TextNote::DATATYPE.into(),
                    exposure: Exposure::Cluster,
                },
            ).operable(crate::mobile_notes::change_contract()),
        )?;
        let annotation_state = state.clone();
        let annotation_frame = frame.clone();
        let change_state = state.clone();
        let change_frame = frame.clone();
        let change_path = notes_path.clone();
        let volume_scenes=scenes.clone();
        let volume_key=id.to_owned();
        let memo = std::sync::Mutex::new(crate::mobile_notes::ChangeMemo::default());
        let change = annotations.operable("change", |ctx| !ctx.caller_peer_id.is_empty(),
            move |ctx, request: crate::mobile_notes::ChangeNote| {
                memo.lock().map_err(|_| InvocationError::TargetUnavailable)?.run(&ctx.caller_peer_id, &request, || {
                if matches!(request.action,crate::mobile_notes::NoteAction::PutVolume{..}|crate::mobile_notes::NoteAction::DeleteVolume{..}) {
                    return volume_scenes.volumes.sdk_change(&volume_key,&change_frame,&request);
                }
                let mut state = change_state.lock().map_err(|_| InvocationError::TargetUnavailable)?;
                let (notes, result) = crate::mobile_notes::apply_change(&state.notes, &request, &change_frame)?;
                persist_notes(&notes, &change_path)?;
                state.notes = notes;
                Ok(result)
                }).map_err(rejected)
            })?;
        let op = annotations.operable(
            "place",
            |ctx| !ctx.caller_peer_id.is_empty(),
            move |_, request: crate::mobile_notes::PlaceNote| {
                let mut state = annotation_state
                    .lock()
                    .map_err(|_| InvocationError::TargetUnavailable)?;
                place(&mut state, request, &annotation_frame, &notes_path).map_err(rejected)
            },
        )?;
        annotations.expose()?;
        endpoint.export_operable(&op)?;
        endpoint.export_operable(&change)?;
        components.push(annotations);

        let robot_component = runtime.component(ComponentSpec::new(format!("{}:{id}",crate::robot_presence::COMPONENT))
            .operable(OperableContract { name:"update".into(), instruction:crate::robot_presence::Update::DATATYPE.into(),
                result:"bool".into(), exposure:Exposure::Cluster }))?;
        let robot_home = owner.clone();
        let robot_frame = frame.clone();
        let robot_op = robot_component.operable("update", |ctx| !ctx.caller_peer_id.is_empty(),
            move |ctx, update:crate::robot_presence::Update| {
                robot_presence.lock().map_err(|_|InvocationError::TargetUnavailable)?
                    .accept(&ctx.caller_peer_id,&robot_home,&robot_frame,update,Instant::now()).map_err(rejected)?;
                Ok(true)
            })?;
        robot_component.expose()?;
        endpoint.export_operable(&robot_op)?;
        components.push(robot_component);
        let presence = runtime.component(
            ComponentSpec::new(format!("{}:{id}", crate::mobile_pose::COMPONENT)).operable(
                OperableContract {
                    name: "update".into(),
                    instruction: crate::mobile_pose::Update::DATATYPE.into(),
                    result: "bool".into(),
                    exposure: Exposure::Cluster,
                },
            ).operable(OperableContract {
                name: "snapshot".into(),
                instruction: crate::mobile_pose::ReadPresence::DATATYPE.into(),
                result: crate::mobile_pose::PresenceSnapshot::DATATYPE.into(),
                exposure: Exposure::Cluster,
            }).operable(OperableContract {
                name: "name".into(),
                instruction: crate::mobile_pose::SetName::DATATYPE.into(),
                result: "bool".into(),
                exposure: Exposure::Cluster,
            }),
        )?;
        // Joystick relay: a phone posts stick values for one robot, and that
        // robot reads its own latest command. Home never moves anything.
        let drive = runtime.component(
            ComponentSpec::new(format!("{}:{id}", crate::mobile_drive::COMPONENT))
                .operable(OperableContract {
                    name: "update".into(),
                    instruction: crate::mobile_drive::Command::DATATYPE.into(),
                    result: "bool".into(),
                    exposure: Exposure::Cluster,
                })
                .operable(OperableContract {
                    name: "snapshot".into(),
                    instruction: crate::mobile_drive::ReadDrive::DATATYPE.into(),
                    result: crate::mobile_drive::Snapshot::DATATYPE.into(),
                    exposure: Exposure::Cluster,
                }),
        )?;
        let board = Arc::new(Mutex::new(crate::mobile_drive::Board::default()));
        let drive_board = board.clone();
        let drive_update = drive.operable(
            "update",
            |ctx| !ctx.caller_peer_id.is_empty(),
            move |ctx, command: crate::mobile_drive::Command| {
                drive_board
                    .lock()
                    .map_err(|_| InvocationError::TargetUnavailable)?
                    .accept(&ctx.caller_peer_id, command, Instant::now())
                    .map_err(rejected)
            },
        )?;
        let drive_frame = frame.clone();
        let drive_owner = owner.clone();
        let drive_snapshot = drive.operable(
            "snapshot",
            |ctx| !ctx.caller_peer_id.is_empty(),
            move |ctx, request: crate::mobile_drive::ReadDrive| {
                if request.frame != drive_frame {
                    return Err(InvocationError::Rejected("Drive scene mismatch".into()));
                }
                // A robot can read only the command addressed to itself.
                let command = board
                    .lock()
                    .map_err(|_| InvocationError::TargetUnavailable)?
                    .read(&ctx.caller_peer_id, Instant::now());
                Ok(crate::mobile_drive::Snapshot {
                    authority_peer_id: drive_owner.clone(),
                    frame: drive_frame.clone(),
                    command,
                })
            },
        )?;
        drive.expose()?;
        endpoint.export_operable(&drive_update)?;
        endpoint.export_operable(&drive_snapshot)?;
        components.push(drive);
        // A phone reports a Portal this scene does not know. Home composes its
        // pose from validated alignment evidence, adds it, and saves the scene.
        let portals = runtime.component(
            ComponentSpec::new(format!("{}:{id}", crate::mobile_portal::COMPONENT)).operable(
                OperableContract {
                    name: "add".into(),
                    instruction: crate::mobile_portal::Report::DATATYPE.into(),
                    result: "bool".into(),
                    exposure: Exposure::Cluster,
                },
            ),
        )?;
        let portal_state = state.clone();
        let portal_reference = reference.clone();
        let scene_path = scenes.directory.join(format!("{id}.json"));
        let add = portals.operable(
            "add",
            |ctx| !ctx.caller_peer_id.is_empty(),
            move |_, report: crate::mobile_portal::Report| {
                let mut state = portal_state
                    .lock()
                    .map_err(|_| InvocationError::TargetUnavailable)?;
                let current = portal_reference
                    .read()
                    .map_err(|_| InvocationError::TargetUnavailable)?
                    .clone();
                let anchor = crate::mobile_portal::anchor(&report, &current).map_err(rejected)?;
                let snapshot = state.map.add_portal(anchor).map_err(rejected)?;
                save_scene(&scene_path, &snapshot).map_err(rejected)?;
                *portal_reference
                    .write()
                    .map_err(|_| InvocationError::TargetUnavailable)? = snapshot;
                Ok(true)
            },
        )?;
        portals.expose()?;
        endpoint.export_operable(&add)?;
        components.push(portals);
        // A phone names itself; it applies at once and is saved, with no restart.
        let name_op = presence.operable(
            "name",
            |ctx| !ctx.caller_peer_id.is_empty(),
            |ctx, request: crate::mobile_pose::SetName| {
                crate::mobile_pose::set_name(&ctx.caller_peer_id, &request.name).map_err(rejected)?;
                Ok(true)
            },
        )?;
        let snapshot_state = live_presence.clone();
        let snapshot_frame = frame.clone();
        let snapshot_owner = owner.clone();
        let snapshot_op = presence.operable("snapshot", |ctx| !ctx.caller_peer_id.is_empty(),
            move |_, request: crate::mobile_pose::ReadPresence| {
                if request.frame != snapshot_frame {
                    return Err(InvocationError::Rejected("Presence scene mismatch".into()));
                }
                let presence = snapshot_state.lock().map_err(|_| InvocationError::TargetUnavailable)?
                    .snapshot_in_frame(Instant::now(), &snapshot_frame);
                Ok(crate::mobile_pose::PresenceSnapshot { authority_peer_id: snapshot_owner.clone(), presence })
            })?;
        let presence_state = live_presence;
        let op = presence.operable(
            "update",
            |ctx| !ctx.caller_peer_id.is_empty(),
            move |ctx, update: crate::mobile_pose::Update| {
                presence_state
                    .lock()
                    .map_err(|_| InvocationError::TargetUnavailable)?
                    .accept_in_frame(&ctx.caller_peer_id, update, Instant::now(), &frame)
                    .map_err(rejected)?;
                Ok(true)
            },
        )?;
        presence.expose()?;
        endpoint.export_operable(&op)?;
        endpoint.export_operable(&snapshot_op)?;
        endpoint.export_operable(&name_op)?;
        components.push(presence);
        let mut publisher = Self {
            id: id.into(),
            reference,
            state,
            inbox,
            contributions,
            scenes,
            output,
            capture,
            clock,
            owner,
            started,
            last_tick: None,
            last_content: None,
            _components: components,
        };
        publisher.tick()?;
        Ok(publisher)
    }
    pub fn tick(&mut self) -> Result<()> {
        if self
            .last_tick
            .is_some_and(|t| t.elapsed() < Duration::from_secs(1))
        {
            return Ok(());
        }
        self.last_tick = Some(Instant::now());
        let pending = self
            .inbox
            .lock()
            .map_err(|_| anyhow!("Depth inbox unavailable"))?
            .take();
        let mut state = self
            .state
            .lock()
            .map_err(|_| anyhow!("Scene unavailable"))?;
        for contribution in self.contributions.lock().map_err(|_|anyhow!("Contribution inbox unavailable"))?.take() {
            let peer=contribution.source.peer_id.clone();
            if let Err(e)=state.map.integrate_contribution(contribution){state.map.peer_status(&peer,"rejected",&e.to_string());}
        }
        for envelope in pending {
            let peer = envelope.source.peer_id.clone();
            if let Err(e) = state.map.integrate_phone(envelope) {
                state.map.peer_status(&peer, "rejected", &e.to_string());
            }
        }
        let reference = self
            .reference
            .read()
            .map_err(|_| anyhow!("Scene reference unavailable"))?
            .clone();
        let scene = Scene {
            schema: SCHEMA.into(),
            authority_peer_id: self.owner.clone(),
            publication_clock: self.clock.clone(),
            portals: reference.clone(),
            geometry: state.map.scene()?,
            notes: { let mut notes=state.notes.clone();notes.volumes.clear();notes },
            volumes: self.scenes.volumes.mobile(&self.id,&reference.scenegraph.map.frame.id)?,
            semantic_volumes: Some(self.scenes.volumes.get(&self.id, &reference.scenegraph.map.frame.id)?),
            presence: state
                .presence
                .lock().map_err(|_| anyhow!("Phone presence unavailable"))?
                .snapshot_in_frame(Instant::now(), &reference.scenegraph.map.frame.id),
            robot_presence: state.robot_presence.lock().map_err(|_|anyhow!("Robot presence unavailable"))?
                .snapshot(Instant::now(), &reference.scenegraph.map.frame.id),
        };
        scene.validate(&self.owner, &reference.scenegraph.map.frame.id)?;
        let content = serde_json::to_vec(&scene)?;
        if self.last_content.as_ref() == Some(&content) {
            return Ok(());
        }
        self.scenes
            .mobile_views
            .lock()
            .map_err(|_| anyhow!("Scene view unavailable"))?
            .insert(self.id.clone(), scene.view()?);
        self.output
            .publish(self.started.elapsed().as_nanos() as u64, Arc::new(scene))?;
        ensure!(
            self.capture.errors().is_empty(),
            "Mobile scene retention failed"
        );
        self.last_content = Some(content);
        Ok(())
    }
}
/// Replace the saved scene atomically so a restart keeps reported Portals.
fn save_scene(path: &std::path::Path, snapshot: &MapSnapshot) -> Result<()> {
    use std::io::Write;
    snapshot.validate()?;
    let directory = path.parent().ok_or_else(|| anyhow!("Scene path has no directory"))?;
    let temporary = directory.join(format!(".portal.{}.tmp", uuid::Uuid::new_v4()));
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temporary)?;
    file.write_all(&serde_json::to_vec_pretty(snapshot)?)?;
    file.sync_all()?;
    std::fs::rename(temporary, path)?;
    Ok(())
}
fn rejected(e: anyhow::Error) -> InvocationError {
    InvocationError::Rejected(e.to_string())
}

fn place(
    state: &mut State,
    request: crate::mobile_notes::PlaceNote,
    frame: &str,
    path: &std::path::Path,
) -> Result<crate::notes::TextNote> {
    ensure!(
        request.frame == frame,
        "Annotation belongs to another scene"
    );
    uuid::Uuid::parse_str(&request.id)?;
    if let Some(note) = state.notes.items.iter().find(|n| n.id == request.id) {
        ensure!(
            note.text == request.text.trim(),
            "Annotation ID already used"
        );
        return Ok(note.clone());
    }
    let voxels = state
        .map
        .voxels
        .as_ref()
        .map(crate::model::occupied)
        .transpose()?
        .unwrap_or_default();
    let (hit, floor) =
        crate::mobile_notes::intersection(&voxels, 0.1, request.origin, request.direction)?
            .ok_or_else(|| anyhow!("No floor or voxel within 40 metres"))?;
    let note = crate::notes::TextNote {
        id: request.id,
        text: request.text.trim().into(),
        details: Default::default(),
        position: hit.position,
        anchor_portal: String::new(),
        anchor_robot: None,
        height_m: 0.,
        anchor_surface: Some(crate::notes::SurfaceAnchor {
            frame: frame.into(),
            voxel_center: hit.center,
            beam_diameter_m: 0.,
            floor,
        }),
    };
    let mut notes = state.notes.clone();
    notes.items.push(note.clone());
    notes.validate(frame)?;
    persist_notes(&notes, path)?;
    state.notes = notes;
    Ok(note)
}
fn persist_notes(notes: &Notes, path: &std::path::Path) -> Result<()> {
    std::fs::create_dir_all(path.parent().unwrap())?;
    let temporary = path.with_extension(format!("{}.tmp", uuid::Uuid::new_v4()));
    use std::io::Write;
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temporary)?;
    file.write_all(&serde_json::to_vec_pretty(&notes)?)?;
    file.sync_all()?;
    std::fs::rename(temporary, path)?;
    Ok(())
}

/// Authenticated SDK subscription, pinned to this scene's Product and frame.
pub async fn receive(
    peer: &auki_sdk::AukiPeer,
    home: &HomeConnection,
    stop: &tokio_util::sync::CancellationToken,
    idle: Duration,
    mut accept: impl FnMut(Value),
) -> Result<()> {
    receive_scene(peer, home, stop, idle, |scene| { accept(scene.mobile()?); Ok(()) }).await
}

/// Consume the authenticated scene without constructing its potentially large
/// render representation. Volume consumers need only the semantic extension.
pub async fn receive_scene(
    peer: &auki_sdk::AukiPeer,
    home: &HomeConnection,
    stop: &tokio_util::sync::CancellationToken,
    idle: Duration,
    mut accept: impl FnMut(&Scene) -> Result<()>,
) -> Result<()> {
    let expected = home.validate(peer)?;
    let client = ComponentProtocolClient::new(peer.protocols());
    let mut failures = Vec::new();
    let mut subscription = None;
    for route in &home.routes {
        let route: auki_sdk::Multiaddr = route.parse()?;
        let attempt = async {
            let CatalogResponse::Snapshot { snapshot } =
                client.catalog_exact(expected, route.clone(), None).await?
            else {
                bail!("No catalog snapshot")
            };
            let entry = snapshot
                .products
                .iter()
                .find(|p| p.manifest.product_id == home.product_id)
                .ok_or_else(|| anyhow!("Selected scene not published by Home"))?;
            ensure!(
                entry.manifest.peer_id == home.peer_id,
                "Scene owner mismatch"
            );
            let stream = client
                .subscribe_product_exact(
                    expected,
                    route,
                    ProductReference {
                        peer_id: home.peer_id.clone(),
                        product_id: home.product_id.clone(),
                        manifest_hash: entry.manifest_hash.clone(),
                    },
                    ObservationStart::LatestExisting,
                    limits(),
                    size,
                )
                .await?;
            Ok::<_, anyhow::Error>(stream)
        };
        let result = tokio::select! { _=stop.cancelled()=>return Ok(()), result=tokio::time::timeout(Duration::from_secs(20),attempt)=>result };
        match result {
            Ok(Ok(s)) => {
                subscription = Some(s);
                break;
            }
            Ok(Err(e)) => failures.push(e.to_string()),
            Err(_) => failures.push("Route timed out".into()),
        }
    }
    let mut subscription =
        subscription.ok_or_else(|| anyhow!("Home connection failed: {}", failures.join("; ")))?;
    let work = async {
        ensure!(subscription.product().producer.payload.schema() == SCHEMA,"Unexpected mobile scene schema");
        let clock = subscription.product().producer.clock.clone();
        let mut received = false;
        loop {
            let next = tokio::select! {
                _=stop.cancelled()=>return Ok(()),
                exit=peer.wait_stopped()=>bail!("Phone peer stopped: {exit:?}"),
                event=tokio::time::timeout(if received {idle} else {Duration::from_secs(30)},subscription.next())=>event,
            };
            match next {
                Err(_) if received => return Ok(()),
                Err(_) => bail!("Home did not publish initial scene"),
                Ok(event) => match event? {
                    Some(RemoteObservationEvent::Observation(o))=>{
                        o.payload.validate(&home.peer_id,&home.frame)?;
                        ensure!(crate::model::clock_ref(&o.payload.publication_clock)==clock,"Scene clock mismatch");
                        accept(&o.payload)?; received=true;
                    },
                    Some(RemoteObservationEvent::Gap(_))=>{},
                    _=>bail!("Scene stream ended"),
                }
            }
        }
    }.await;
    let cleanup = subscription.close().await.map_err(anyhow::Error::from);
    work.and(cleanup)
}
