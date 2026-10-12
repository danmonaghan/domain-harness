//! A separately discoverable relationship between physical frames. Never rebases a map.
use crate::local_mapper::{LocalMap, compose, from_rigid, inverse, rigid};
use anyhow::{Result, anyhow, ensure};
use auki_component_protocol::*;
use auki_components::*;
use auki_scenegraph::{MapFrame, QrAnchor, RigidTransform};
use serde::{Deserialize, Serialize};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

pub const PRODUCT: &str = "robot-to-scene-transform";
pub const SCHEMA: &str = "spatial.frame-transform/v1";
pub const MAX_AGE: Duration = Duration::from_secs(3);
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Validity {
    Unavailable,
    Valid,
    Stale,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Evidence {
    pub portal: String,
    pub capture_timestamp_ns: u64,
    pub clock: ClockReference,
    pub provenance: crate::landmarks::Provenance,
    pub reference_peer_id: String,
    pub reference_anchor: QrAnchor,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FrameTransform {
    pub schema: String,
    pub authority_peer_id: String,
    pub source_frame: MapFrame,
    pub target_frame: Option<MapFrame>,
    pub validity: Validity,
    pub reason: String,
    pub transform: Option<RigidTransform>,
    pub evidence: Option<Evidence>,
}
impl ContractType for FrameTransform {
    const DATATYPE: &'static str = SCHEMA;
}
impl FrameTransform {
    pub fn from_local(local: &LocalMap, owner: &str) -> Result<Self> {
        let observed = local.portals()?;
        let mut value = Self {
            schema: SCHEMA.into(),
            authority_peer_id: owner.into(),
            source_frame: observed.scenegraph.map.frame.clone(),
            target_frame: None,
            validity: Validity::Unavailable,
            reason: "No shared portal measurement".into(),
            transform: None,
            evidence: None,
        };
        let Some(home) = &local.home_portals else {
            return Ok(value);
        };
        let measured = observed
            .scenegraph
            .anchors
            .get(&home.anchor)
            .ok_or_else(|| anyhow!("Measured portal missing"))?;
        let reference = home
            .portals
            .scenegraph
            .anchors
            .get(&home.anchor)
            .ok_or_else(|| anyhow!("Reference portal missing"))?;
        ensure!(
            (reference.side_length_m - measured.side_length_m).abs() < 1e-6,
            "Portal sizes differ"
        );
        // Both poses use the same physical printed-right/up/out marker axes.
        // T_scene_bracket = T_scene_marker * inverse(T_bracket_marker).
        // No extra convention rotation: these full poses already encode it.
        let pose = compose(
            &from_rigid(&reference.pose_in_map),
            &inverse(&from_rigid(&measured.pose_in_map))?,
        )?;
        let metadata = local.metadata();
        let evidence = &metadata["local_mapping"]["observed_portals"][&home.anchor];
        let timestamp = evidence["capture_timestamp_ns"]
            .as_u64()
            .ok_or_else(|| anyhow!("Missing portal capture time"))?;
        value.target_frame = Some(home.portals.scenegraph.map.frame.clone());
        value.transform = Some(rigid(
            &pose,
            &local.frame,
            &home.portals.scenegraph.map.frame.id,
        ));
        value.evidence = Some(Evidence {
            portal: home.anchor.clone(),
            capture_timestamp_ns: timestamp,
            clock: serde_json::from_value(evidence["clock"].clone())?,
            provenance: serde_json::from_value(evidence["provenance"].clone())?,
            reference_peer_id: home.peer_id.clone(),
            reference_anchor: reference.clone(),
        });
        let valid =
            (local.latest_pose.is_some() || local.frame.starts_with(&format!("{}:",crate::local_mapper::BRACKETBOT_FAMILY)))
                && local.alignment_after_ns.is_none_or(|t| timestamp > t);
        value.validity = if valid {
            Validity::Valid
        } else {
            Validity::Stale
        };
        value.reason = if valid && evidence["restored_from_memory"] == true {
            "Remembered shared portal in the saved SLAM map; robot frame unchanged"
        } else if valid {
            "Measured shared portal; robot frame unchanged"
        } else if local.frame.starts_with(&format!("{}:", crate::local_mapper::BRACKETBOT_FAMILY)) {
            "Tracking unavailable; waiting for a valid pose in the same map frame"
        } else {
            "Tracking lost; a new tracked portal observation is required"
        }
        .into();
        value.validate(owner)?;
        Ok(value)
    }
    pub fn validate(&self, owner: &str) -> Result<()> {
        ensure!(
            self.schema == SCHEMA && self.authority_peer_id == owner,
            "Transform authority/schema mismatch"
        );
        crate::conventions::validate_frame(&self.source_frame)?;
        ensure!(
            crate::local_mapper::is_local_frame(&self.source_frame.id),
            "Transform must originate in robot space"
        );
        match (&self.transform, &self.target_frame, &self.evidence) {
            (None, None, None) => ensure!(
                self.validity == Validity::Unavailable,
                "Missing transform evidence"
            ),
            (Some(t), Some(target), Some(e)) => {
                crate::conventions::validate_frame(target)?;
                ensure!(
                    t.from_frame_id == self.source_frame.id && t.to_frame_id == target.id,
                    "Transform endpoints mismatch"
                );
                from_rigid(t).validate()?;
                ensure!(
                    self.validity != Validity::Unavailable
                        && e.capture_timestamp_ns > 0
                        && e.portal == e.reference_anchor.anchor_id
                        && !e.reference_peer_id.is_empty(),
                    "Invalid transform evidence"
                );
                e.reference_anchor.validate()?;
                crate::landmarks::provenance(&e.provenance)?;
                let mapper = e
                    .provenance
                    .mapped_by
                    .as_ref()
                    .ok_or_else(|| anyhow!("Missing mapper"))?;
                ensure!(mapper.peer_id == owner, "Mapper provenance mismatch");
                ensure!(
                    e.reference_anchor.pose_in_map.to_frame_id == target.id,
                    "Reference frame mismatch"
                );
                let observation = e
                    .provenance
                    .source_observation
                    .as_ref()
                    .ok_or_else(|| anyhow!("Missing source observation"))?;
                let detector = e
                    .provenance
                    .detected_by
                    .as_ref()
                    .ok_or_else(|| anyhow!("Missing detector"))?;
                ensure!(
                    e.clock.peer_id == owner
                        && observation.output.peer_id == owner
                        && detector.peer_id == owner
                        && observation.clock == e.clock
                        && observation.timestamp_ns == e.capture_timestamp_ns,
                    "Observation provenance mismatch"
                );
            }
            _ => anyhow::bail!("Incomplete transform"),
        }
        Ok(())
    }
}
fn size(v: &FrameTransform) -> usize {
    serde_json::to_vec(v).map_or(usize::MAX, |v| v.len())
}
fn limits() -> BufferLimits {
    BufferLimits {
        max_entries: Some(2),
        max_bytes: Some(128 * 1024),
        target_duration: None,
    }
}
pub struct Publisher {
    _component: Component,
    output: ConfiguredObservable<FrameTransform>,
    capture: BufferProductCapture<FrameTransform>,
}
impl Publisher {
    pub fn mount(
        runtime: &ComponentRuntime,
        endpoint: &ComponentProtocolEndpoint,
        clock: ClockReference,
    ) -> Result<Self> {
        let component = runtime.component(
            ComponentSpec::new("portal-frame-transform").observable(ObservableContract {
                name: "transform".into(),
                datatype: SCHEMA.into(),
                schema: SCHEMA.into(),
                access: vec![ObservationAccess::FollowNew],
                exposure: Exposure::Cluster,
            }),
        )?;
        let output = component.configured_observable(ConfiguredObservableSpec::new(
            "transform",
            uuid::Uuid::new_v4().to_string(),
            clock,
            PayloadContract::Structured(StructuredPayloadContract {
                modality: "frame_transform".into(),
                datatype: SCHEMA.into(),
                schema: SCHEMA.into(),
                observes: "robot frame to referenced scene frame".into(),
                unit: Some("meters".into()),
            }),
        ))?;
        component.expose()?;
        let capture = runtime.capture_buffer(PRODUCT, &output, limits(), size)?;
        endpoint.export_product(&capture.product())?;
        Ok(Self {
            _component: component,
            output,
            capture,
        })
    }
    pub fn reference(&self) -> ProductReference {
        self.capture.product().reference()
    }
    pub fn publish(&self, local: &LocalMap, owner: &str, timestamp: u64) -> Result<FrameTransform> {
        let value = FrameTransform::from_local(local, owner)?;
        self.output.publish(
            timestamp,
            Arc::new(value.clone()),
        )?;
        ensure!(
            self.capture.errors().is_empty(),
            "Transform retention failed"
        );
        Ok(value)
    }
}

#[derive(Clone)]
pub struct Received {
    pub value: FrameTransform,
    pub product: ProductReference,
    pub received: Instant,
}
impl Received {
    pub fn usable(&self, scene: &crate::network::CanonicalScene) -> bool {
        self.value.validate(&scene.authority_peer_id).is_ok()
            && self.received.elapsed()<=MAX_AGE && self.value.validity==Validity::Valid
            && self.value.source_frame==scene.portals.scenegraph.map.frame
            && self.value.authority_peer_id==scene.authority_peer_id
            && !scene.registries["local_mapping"]["robot_pose"].is_null()
            && self.value.evidence.as_ref().is_some_and(|e|
                scene.registries["local_mapping"]["observed_portals"][&e.portal]["capture_timestamp_ns"].as_u64()==Some(e.capture_timestamp_ns))
            && serde_json::to_value(&self.product).ok().as_ref()==Some(&scene.registries["frame_transform_product"])
    }
}
pub async fn subscribe(
    peer: &auki_sdk::AukiPeer,
    home: &crate::network::HomeConnection,
) -> Result<RemoteProductSubscription<FrameTransform>> {
    let expected = home.validate(peer)?;
    let client = ComponentProtocolClient::new(peer.protocols());
    for route in &home.routes {
        let route: auki_sdk::Multiaddr = route.parse()?;
        let attempt = async {
            let CatalogResponse::Snapshot { snapshot } =
                client.catalog_exact(expected, route.clone(), None).await?
            else {
                anyhow::bail!("Missing catalog")
            };
            let p = snapshot
                .products
                .iter()
                .find(|p| p.manifest.product_id == PRODUCT)
                .ok_or_else(|| anyhow!("Transform Product unavailable"))?;
            ensure!(
                p.manifest.peer_id == home.peer_id,
                "Transform Product owner mismatch"
            );
            let subscription = client
                .subscribe_product_exact(
                    expected,
                    route,
                    ProductReference {
                        peer_id: home.peer_id.clone(),
                        product_id: PRODUCT.into(),
                        manifest_hash: p.manifest_hash.clone(),
                    },
                    ObservationStart::LatestExisting,
                    limits(),
                    size,
                )
                .await?;
            ensure!(
                subscription.product().producer.payload.schema() == SCHEMA,
                "Transform Product schema mismatch"
            );
            Ok::<_, anyhow::Error>(subscription)
        };
        if let Ok(Ok(subscription)) = tokio::time::timeout(Duration::from_secs(5), attempt).await {
            return Ok(subscription);
        }
    }
    anyhow::bail!("Transform Product unavailable")
}
pub async fn follow(
    peer: &auki_sdk::AukiPeer,
    home: &crate::network::HomeConnection,
    state: &tokio::sync::watch::Sender<crate::scene_mirror::RemoteScene>,
    stop: &tokio_util::sync::CancellationToken,
) {
    loop {
        let work = async {
            let mut subscription = subscribe(peer, home).await?;
            let result=async {
                loop {
                    let event=tokio::select!{_=stop.cancelled()=>return Ok(()),e=tokio::time::timeout(MAX_AGE,subscription.next())=>e??};
                    match event {
                        Some(RemoteObservationEvent::Observation(o))=>{
                            o.payload.validate(&home.peer_id)?;
                            ensure!(o.output==subscription.product().manifest.producer,"Transform publication producer mismatch");
                            state.send_modify(|s|s.frame_transform=Some(Received{value:(*o.payload).clone(),product:subscription.product().reference(),received:Instant::now()}));
                        },
                        Some(RemoteObservationEvent::Gap(_))=>{state.send_modify(|s|s.frame_transform=None);},
                        _=>anyhow::bail!("Transform stream ended"),
                    }
                }
            }.await;
            let _ = subscription.close().await;
            result
        };
        let result: Result<()> = tokio::select! {_=stop.cancelled()=>break,result=work=>result};
        state.send_modify(|s| s.frame_transform = None);
        if stop.is_cancelled() {
            break;
        }
        let _ = result; // Optional Product: older peers retain their native view.
        tokio::select! {_=stop.cancelled()=>break,_=tokio::time::sleep(Duration::from_secs(2))=>{}}
    }
    state.send_modify(|s| s.frame_transform = None);
}
