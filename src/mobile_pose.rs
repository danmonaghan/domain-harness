//! Ephemeral, calibrated phone presence. Transport identity owns each pose.
use crate::model::FRAME;
use anyhow::{Result, ensure};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};

pub const COMPONENT: &str = "home-phone-presence";
/// Display names per phone peer. Checked-in `config/phone-names.json` first,
/// then the local file (`SPATIAL_PHONE_NAMES`, default `data/phone-names.json`),
/// both `{"<peer id>": "<name>"}`. A phone can also name itself at runtime with
/// the presence `name` operation; that takes effect at once and is saved to the
/// local file, so no restart is needed. The entry `"*"` names every unlisted
/// phone; otherwise a short peer suffix is used.
static NAMES: std::sync::LazyLock<std::sync::RwLock<BTreeMap<String, String>>> =
    std::sync::LazyLock::new(|| {
        let mut names = read_names("config/phone-names.json");
        names.extend(read_names(&local_names_path()));
        std::sync::RwLock::new(names)
    });
fn local_names_path() -> String {
    std::env::var("SPATIAL_PHONE_NAMES").unwrap_or_else(|_| "data/phone-names.json".into())
}
fn read_names(path: &str) -> BTreeMap<String, String> {
    std::fs::read(path)
        .ok()
        .and_then(|bytes| serde_json::from_slice(&bytes).ok())
        .unwrap_or_default()
}
pub fn display_name(peer: &str) -> String {
    let names = NAMES.read().unwrap_or_else(|e| e.into_inner());
    names
        .get(peer)
        .or_else(|| names.get("*"))
        .cloned()
        .unwrap_or_else(|| {
            let tail = &peer[peer.len().saturating_sub(4)..];
            format!("phone {tail}")
        })
}
/// A phone names itself. The transport identity is the key, so a phone can
/// never rename another device.
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SetName {
    pub name: String,
}
impl auki_components::ContractType for SetName {
    const DATATYPE: &'static str = "spatial.phone-name/v1";
}
pub fn set_name(peer: &str, name: &str) -> Result<()> {
    let name = name.trim();
    ensure!(!peer.is_empty() && peer != "*", "Missing phone identity");
    ensure!(
        (1..=24).contains(&name.chars().count()) && !name.chars().any(char::is_control),
        "A name is 1 to 24 printable characters"
    );
    let mut names = NAMES.write().unwrap_or_else(|e| e.into_inner());
    if names.get(peer).map(String::as_str) == Some(name) {
        return Ok(());
    }
    ensure!(
        names.contains_key(peer) || names.len() < 256,
        "Phone name capacity reached"
    );
    // Save only to the local file; the checked-in file is never rewritten.
    let path = local_names_path();
    let mut saved = read_names(&path);
    saved.insert(peer.into(), name.into());
    let target = std::path::Path::new(&path);
    if let Some(parent) = target.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    let temporary = target.with_extension(format!("{}.tmp", uuid::Uuid::new_v4()));
    std::fs::write(&temporary, serde_json::to_vec_pretty(&saved)?)?;
    std::fs::rename(temporary, target)?;
    names.insert(peer.into(), name.into());
    Ok(())
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReadPresence { pub frame: String }
impl auki_components::ContractType for ReadPresence {
    const DATATYPE: &'static str = "spatial.phone-presence-read/v1";
}
#[derive(Clone, Serialize, Deserialize)]
pub struct PresenceSnapshot { pub authority_peer_id: String, pub presence: Value }
impl auki_components::ContractType for PresenceSnapshot {
    const DATATYPE: &'static str = "spatial.phone-presence-snapshot/v1";
}

/// Live, scene-scoped authenticated read. Unlike retained scene geometry, this
/// samples the presence cache at invocation time; request RTT bounds transit age.
pub async fn fetch(peer: &auki_sdk::AukiPeer, home: &crate::network::HomeConnection) -> Result<PresenceSnapshot> {
    Ok(PresenceReader::default().read(peer, home).await?.0)
}

/// Cache only discovery, never poses. A changed profile or failed invocation
/// discards the pinned reference before the next read.
#[derive(Default)]
pub struct PresenceReader {
    target: Option<(String, auki_sdk::Multiaddr, auki_components::ComponentReference)>,
}
impl PresenceReader {
pub async fn read(&mut self, peer: &auki_sdk::AukiPeer, home: &crate::network::HomeConnection) -> Result<(PresenceSnapshot, Duration)> {
    use auki_component_protocol::{CatalogResponse, ComponentProtocolClient};
    let expected = home.validate(peer)?;
    let client = ComponentProtocolClient::new(peer.protocols());
    let key = serde_json::to_string(home)?;
    if self.target.as_ref().is_some_and(|(old,_,_)| old != &key) { self.target = None; }
    if self.target.is_none() {
    for route in &home.routes {
        let route: auki_sdk::Multiaddr = route.parse()?;
        let Ok(CatalogResponse::Snapshot { snapshot }) = client.catalog_exact(expected, route.clone(), None).await else {continue};
        let entry = snapshot.components.iter().find(|c| c.manifest.component_id == home.scoped_component(COMPONENT))
            .ok_or_else(|| anyhow::anyhow!("Home phone presence unavailable"))?;
        ensure!(entry.manifest.peer_id == home.peer_id, "Presence authority mismatch");
        self.target = Some((key.clone(), route, entry.manifest.reference()));
        break;
    }
    }
    let (_,route,reference) = self.target.as_ref().ok_or_else(|| anyhow::anyhow!("Home phone presence unavailable"))?;
    let began = Instant::now();
    let result = client.invoke_exact::<_, PresenceSnapshot>(expected, route.clone(), reference.clone(),
            "snapshot", "robot-phone-target", uuid::Uuid::new_v4().to_string(), Some(Duration::from_millis(700)),
            &ReadPresence {frame: home.frame.clone()}).await;
    let age = began.elapsed();
    let value = match result {
        Ok(reply) => reply.result,
        Err(error) => { self.target = None; return Err(error.into()); }
    };
    if value.authority_peer_id != home.peer_id || value.presence["frame"] != home.frame { self.target = None; }
        ensure!(value.authority_peer_id == home.peer_id && value.presence["frame"] == home.frame,
                "Phone presence belongs to another Home/scene");
    Ok((value, age))
}
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sample {
    pub session: String,
    pub timestamp_ns: u64,
    pub pose: auki_scenegraph::RigidTransform,
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Update {
    pub sample: Option<Sample>,
    /// `Some("asleep")` with no sample: the phone paused its camera and keeps
    /// its last pose. Omitted otherwise, so live updates are unchanged on the wire.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub status: Option<String>,
}
/// How long a sleeping phone stays listed without a fresh heartbeat.
pub const ASLEEP_TTL: Duration = Duration::from_secs(15 * 60);
#[derive(Clone, Copy, PartialEq)]
enum State {
    Live,
    Asleep,
    Withdrawn,
}
impl auki_components::ContractType for Update {
    const DATATYPE: &'static str = "spatial.phone-presence/v1";
}
#[derive(Default)]
pub struct Presence {
    entries: BTreeMap<String, (Sample, Instant, State)>,
}
impl Presence {
    pub fn accept(&mut self, peer: &str, update: Update, now: Instant) -> Result<()> {
        self.accept_in_frame(peer, update, now, FRAME)
    }
    pub fn accept_in_frame(
        &mut self,
        peer: &str,
        update: Update,
        now: Instant,
        frame: &str,
    ) -> Result<()> {
        ensure!(!peer.is_empty(), "Missing phone identity");
        ensure!(
            matches!(update.status.as_deref(), None | Some("asleep")),
            "Unknown phone status"
        );
        let Some(sample) = update.sample else {
            if let Some(entry) = self.entries.get_mut(peer) {
                if update.status.is_some() {
                    // Sleep keeps the last pose; each heartbeat refreshes it.
                    entry.1 = now;
                    entry.2 = State::Asleep;
                } else {
                    entry.2 = State::Withdrawn;
                }
            }
            return Ok(());
        };
        ensure!(update.status.is_none(), "A posed phone cannot be asleep");
        uuid::Uuid::parse_str(&sample.session)?;
        ensure!(
            sample.pose.to_frame_id == frame
                && sample.pose.from_frame_id == format!("phone-camera:{}", sample.session),
            "Phone pose frame mismatch"
        );
        ensure!(
            sample
                .pose
                .translation
                .iter()
                .chain(sample.pose.rotation_wxyz.iter())
                .all(|n| n.is_finite())
                && (sample.pose.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs()
                    < 0.001,
            "Invalid phone pose"
        );
        if let Some((old, _, _)) = self.entries.get(peer)
            && old.session == sample.session
            && sample.timestamp_ns <= old.timestamp_ns
        {
            return Ok(());
        }
        ensure!(
            self.entries.contains_key(peer) || self.entries.len() < 64,
            "Phone presence capacity reached"
        );
        self.entries.insert(peer.into(), (sample, now, State::Live));
        Ok(())
    }
    pub fn snapshot(&self, now: Instant) -> Value {
        self.snapshot_in_frame(now, FRAME)
    }
    pub fn snapshot_in_frame(&self, now: Instant, frame: &str) -> Value {
        let poses: Vec<_> = self.entries.iter().filter(|(_, (_,received,state))| now.duration_since(*received) < match state { State::Live => Duration::from_secs(3), State::Asleep => ASLEEP_TTL, State::Withdrawn => Duration::ZERO }).map(|(peer,(s,received,state))| {
            let mut clock = crate::model::clock_entry(peer, &s.session);
            clock.clock_id = "arkit-capture".into();
            json!({"peer_id":peer,"name":display_name(peer),"state":if *state == State::Asleep { "asleep" } else { "live" },"pose":s.pose,"capture_timestamp_ns":s.timestamp_ns,"clock":clock,"received_age_ms":now.duration_since(*received).as_millis()})
        }).collect();
        json!({"frame":frame,"poses":poses})
    }
}
pub fn sync(store: &mut crate::model::Store, presence: &Presence, now: Instant) {
    let value = presence.snapshot(now);
    if store.built.registries["mobile_presence"] != value {
        store.built.registries["mobile_presence"] = value;
    }
}
pub fn append_labels(view: &mut Value, registry: &Value) {
    append_labels_in_frame(view, registry, FRAME)
}
pub fn append_labels_in_frame(view: &mut Value, registry: &Value, frame: &str) {
    if registry["frame"] != frame {
        return;
    }
    let Some(poses) = registry["poses"].as_array() else {
        return;
    };
    if !view["notes"].is_array() {
        view["notes"] = json!([]);
    }
    for pose in poses {
        view["notes"].as_array_mut().unwrap().push(json!({
            "id":format!("phone-name:{}",pose["peer_id"].as_str().unwrap_or("")),
            "text":format!("{}{}", pose["name"].as_str().unwrap_or("phone"), if pose["state"] == "asleep" { " · asleep" } else { "" }), "position":pose["pose"]["translation"], "height_m":0,
            "phone_pose":pose
        }));
    }
}

pub async fn send(
    peer: &auki_sdk::AukiPeer,
    home: &crate::network::HomeConnection,
    update: &Update,
) -> Result<()> {
    send_component(peer, home, COMPONENT, update).await
}
pub async fn send_component<T: auki_components::ContractType + Serialize>(
    peer: &auki_sdk::AukiPeer,
    home: &crate::network::HomeConnection,
    component: &str,
    update: &T,
) -> Result<()> {
    invoke_pinned(peer, home, component, "update", "phone-pose", update, "Home phone presence unavailable").await?;
    Ok(())
}
type Pinned = (auki_sdk::Multiaddr, auki_components::ComponentReference);
/// Home's component references change only when Home restarts, so one catalog
/// read serves every later call. A failed call forgets it and looks it up again.
static PINNED: std::sync::LazyLock<std::sync::Mutex<std::collections::HashMap<String, Pinned>>> =
    std::sync::LazyLock::new(Default::default);
pub(crate) async fn invoke_pinned<T: auki_components::ContractType + Serialize>(
    peer: &auki_sdk::AukiPeer,
    home: &crate::network::HomeConnection,
    component: &str,
    operable: &str,
    kind: &str,
    payload: &T,
    unavailable: &'static str,
) -> Result<bool> {
    use auki_component_protocol::{CatalogResponse, ComponentProtocolClient};
    let expected = home.validate(peer)?;
    let client = ComponentProtocolClient::new(peer.protocols());
    let component_id = home.scoped_component(component);
    let key = format!("{}|{component_id}", home.peer_id);
    for fresh in [false, true] {
        let pinned = PINNED.lock().unwrap().get(&key).cloned().filter(|_| !fresh);
        let (route, reference) = match pinned {
            Some(pinned) => pinned,
            None => {
                let mut found = None;
                for route in &home.routes {
                    let route: auki_sdk::Multiaddr = route.parse()?;
                    let Ok(Ok(CatalogResponse::Snapshot { snapshot })) = tokio::time::timeout(
                        Duration::from_secs(1),
                        client.catalog_exact(expected, route.clone(), None),
                    )
                    .await
                    else {
                        continue;
                    };
                    let entry = snapshot
                        .components
                        .iter()
                        .find(|c| c.manifest.component_id == component_id)
                        .ok_or_else(|| anyhow::anyhow!(unavailable))?;
                    found = Some((route, entry.manifest.reference()));
                    break;
                }
                let found = found.ok_or_else(|| anyhow::anyhow!(unavailable))?;
                PINNED.lock().unwrap().insert(key.clone(), found.clone());
                found
            }
        };
        match client
            .invoke_exact::<_, bool>(
                expected,
                route,
                reference,
                operable,
                kind,
                uuid::Uuid::new_v4().to_string(),
                Some(Duration::from_secs(1)),
                payload,
            )
            .await
        {
            Ok(done) => return Ok(done.result),
            Err(error) => {
                PINNED.lock().unwrap().remove(&key);
                if fresh {
                    return Err(error.into());
                }
            }
        }
    }
    unreachable!("the second attempt returns")
}
