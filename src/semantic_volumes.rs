//! Home-authored semantic regions, in meters in one explicit scene frame.
use anyhow::{Result, ensure};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::PathBuf, sync::Mutex};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Volume {
    pub id: String,
    pub title: String,
    pub description: String,
    #[serde(default = "default_semantic_type")]
    pub semantic_type: String,
    pub center: [f64; 3],
    pub size: [f64; 3],
    /// Rotation around the OpenGL Y axis, in radians.
    pub yaw: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Volumes {
    pub schema: String,
    pub frame: String,
    pub items: Vec<Volume>,
}
impl Volumes {
    pub fn empty(frame: &str) -> Self {
        Self {
            schema: "spatial-harness.semantic-volumes/v1".into(),
            frame: frame.into(),
            items: vec![],
        }
    }
    pub fn validate(&self, frame: &str) -> Result<()> {
        ensure!(
            self.schema == "spatial-harness.semantic-volumes/v1" && self.frame == frame,
            "Semantic volume frame or schema mismatch"
        );
        ensure!(self.items.len() <= 256, "Scene has too many volumes");
        ensure!(serde_json::to_vec(self)?.len() <= 4 * 1024 * 1024, "Semantic volume collection is too large");
        let mut ids = std::collections::BTreeSet::new();
        for v in &self.items {
            ensure!(
                uuid::Uuid::parse_str(&v.id).is_ok() && ids.insert(&v.id),
                "Invalid or duplicate volume identity"
            );
            ensure!(
                !v.title.trim().is_empty()
                    && v.title.chars().count() <= 120
                    && v.description.chars().count() <= 10000,
                "Provide a title (up to 120 characters) and a description up to 10000 characters"
            );
            ensure!(!v.semantic_type.trim().is_empty() && v.semantic_type.chars().count()<=80
                && !v.semantic_type.chars().any(char::is_control),"Invalid semantic type");
            ensure!(
                v.center.iter().all(|n| n.is_finite() && n.abs() <= 100000.)
                    && v.size
                        .iter()
                        .all(|n| n.is_finite() && *n >= 0.01 && *n <= 10000.)
                    && v.yaw.is_finite()
                    && v.yaw.abs() <= std::f64::consts::TAU,
                "Invalid volume position, dimensions or rotation"
            );
        }
        Ok(())
    }
}

pub struct Storage {
    directory: PathBuf,
    cache: Mutex<BTreeMap<String, Volumes>>,
}
fn default_semantic_type() -> String { "region".into() }

/// SDK/mobile representation: position is bottom-centre, not box centre.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticVolume {
    pub id: String,
    pub name: String,
    pub semantic_type: String,
    pub position: [f64; 3],
    pub dimensions: [f64; 3],
    pub yaw: f64,
    // Older iPhone clients omit this. Preserve the saved description on edits.
    #[serde(default, skip_serializing)]
    pub description: Option<String>,
}
impl SemanticVolume {
    pub fn validate(&self) -> Result<()> {
        let v = Volume::from_mobile(self, None);
        let mut collection = Volumes::empty("validation");
        collection.items.push(v);
        collection.validate("validation")
    }
}
impl Volume {
    pub fn mobile(&self) -> SemanticVolume {
        let mut position = self.center;
        position[1] -= self.size[1] / 2.;
        SemanticVolume { id:self.id.clone(), name:self.title.clone(), semantic_type:self.semantic_type.clone(),
            position, dimensions:self.size, yaw:(self.yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI,
            description:Some(self.description.clone()) }
    }
    pub fn from_mobile(v:&SemanticVolume, old:Option<&Volume>) -> Self {
        let mut center=v.position; center[1]+=v.dimensions[1]/2.;
        Self { id:v.id.clone(),title:v.name.clone(),semantic_type:v.semantic_type.clone(),center,size:v.dimensions,yaw:v.yaw,
            description:v.description.clone().or_else(||old.map(|v|v.description.clone())).unwrap_or_default() }
    }
}
impl Storage {
    pub fn new(directory: PathBuf) -> Self {
        Self {
            directory,
            cache: Default::default(),
        }
    }
    pub fn mobile(&self,key:&str,frame:&str) -> Result<Vec<SemanticVolume>> {
        Ok(self.get(key,frame)?.items.iter().map(Volume::mobile).collect())
    }
    /// Both authenticated SDK changes and browser edits use this same store.
    pub fn sdk_change(&self,key:&str,frame:&str,request:&crate::mobile_notes::ChangeNote) -> Result<crate::mobile_notes::NoteChange> {
        ensure!(request.frame==frame,"Volume belongs to another scene");
        uuid::Uuid::parse_str(&request.request_id)?;
        match &request.action {
            crate::mobile_notes::NoteAction::PutVolume{volume} => {
                volume.validate()?;
                self.put_mobile(key,frame,volume)?;
            }
            crate::mobile_notes::NoteAction::DeleteVolume{id} => {
                uuid::Uuid::parse_str(id)?;
                if self.get(key,frame)?.items.iter().any(|v|v.id==*id) { self.change(key,frame,Some(id),None,frame)?; }
            }
            _ => anyhow::bail!("Not a volume operation"),
        }
        Ok(crate::mobile_notes::NoteChange{request_id:request.request_id.clone(),note:None,
            deleted:matches!(request.action,crate::mobile_notes::NoteAction::DeleteVolume{..})})
    }
    fn put_mobile(&self,key:&str,frame:&str,v:&SemanticVolume) -> Result<()> {
        let existing=self.get(key,frame)?;
        let old=existing.items.iter().find(|old|old.id==v.id);
        self.change(key,frame,old.map(|v|v.id.as_str()),Some(Volume::from_mobile(v,old)),frame)?;
        Ok(())
    }
    /// Import PR17's saved Notes.volumes without deleting its source file.
    /// A receipt prevents deleted migrated volumes being resurrected on restart.
    pub fn import_notes(&self,key:&str,frame:&str,values:&[SemanticVolume]) -> Result<()> {
        self.get(key,frame)?; // Validate the scene key before constructing a path.
        let receipt=self.directory.join(format!("{key}.notes-import.json"));
        if receipt.exists() || values.is_empty() { return Ok(()); }
        let existing=self.get(key,frame)?;
        for v in values {
            v.validate()?;
            if let Some(old)=existing.items.iter().find(|old|old.id==v.id) {
                let migrated=Volume::from_mobile(v,Some(old));
                ensure!(old==&migrated,"Conflicting saved volume {} requires reconciliation",v.id);
            }
        }
        for v in values { self.put_mobile(key,frame,v)?; }
        crate::model::persist(&receipt,&serde_json::json!({"frame":frame,"ids":values.iter().map(|v|&v.id).collect::<Vec<_>>()}))?;
        Ok(())
    }
    fn read(&self, key: &str, frame: &str) -> Result<Volumes> {
        ensure!(
            !key.is_empty()
                && key.len() <= 64
                && key
                    .bytes()
                    .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-'),
            "Invalid scene identity"
        );
        let path = self.directory.join(format!("{key}.json"));
        let value = if path.exists() {
            ensure!(
                std::fs::metadata(&path)?.len() <= 4 * 1024 * 1024,
                "Volume file too large"
            );
            serde_json::from_slice::<Volumes>(&std::fs::read(path)?)?
        } else {
            Volumes::empty(frame)
        };
        value.validate(frame)?;
        Ok(value)
    }
    pub fn get(&self, key: &str, frame: &str) -> Result<Volumes> {
        let mut cache = self
            .cache
            .lock()
            .map_err(|_| anyhow::anyhow!("Volume storage unavailable"))?;
        if !cache.contains_key(key) {
            cache.insert(key.into(), self.read(key, frame)?);
        }
        let value = cache.get(key).unwrap();
        value.validate(frame)?;
        Ok(value.clone())
    }
    /// Persist before exposing the change. IDs cannot be silently upserted.
    pub fn change(
        &self,
        key: &str,
        frame: &str,
        id: Option<&str>,
        value: Option<Volume>,
        supplied_frame: &str,
    ) -> Result<Volumes> {
        ensure!(
            frame == supplied_frame,
            "Scene frame changed; reload before editing"
        );
        let mut cache = self
            .cache
            .lock()
            .map_err(|_| anyhow::anyhow!("Volume storage unavailable"))?;
        let mut next = match cache.get(key) {
            Some(v) => v.clone(),
            None => self.read(key, frame)?,
        };
        next.validate(frame)?;
        match id {
            Some(id) => {
                let index = next
                    .items
                    .iter()
                    .position(|v| v.id == id)
                    .ok_or_else(|| anyhow::anyhow!("Volume not found"))?;
                if let Some(v) = value {
                    ensure!(v.id == id, "Volume identity mismatch");
                    next.items[index] = v;
                } else {
                    next.items.remove(index);
                }
            }
            None => {
                let v = value.ok_or_else(|| anyhow::anyhow!("Missing volume"))?;
                next.items.push(v);
            }
        }
        next.validate(frame)?;
        crate::model::persist(&self.directory.join(format!("{key}.json")), &next)?;
        cache.insert(key.into(), next.clone());
        Ok(next)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn volume() -> Volume {
        Volume {
            id: uuid::Uuid::new_v4().to_string(),
            title: "Kitchen".into(),
            description: "Preparation area".into(),
            semantic_type: "region".into(),
            center: [1., 1., 2.],
            size: [2., 2., 3.],
            yaw: 0.,
        }
    }
    #[test]
    fn mobile_migration_preserves_sources_and_does_not_resurrect_deleted_volumes() -> Result<()> {
        let dir=tempfile::tempdir()?;
        let storage=Storage::new(dir.path().into());
        let legacy=volume();
        storage.change("room","frame",None,Some(legacy.clone()),"frame")?;
        let mut imported=legacy.mobile();imported.id=uuid::Uuid::new_v4().to_string();imported.name="Phone room".into();
        storage.import_notes("room","frame",&[imported.clone()])?;
        assert_eq!(storage.get("room","frame")?.items.len(),2);
        assert_eq!(storage.mobile("room","frame")?[0].position,[1.,0.,2.]);
        let request=crate::mobile_notes::ChangeNote{request_id:uuid::Uuid::new_v4().to_string(),frame:"frame".into(),action:crate::mobile_notes::NoteAction::DeleteVolume{id:imported.id.clone()}};
        storage.sdk_change("room","frame",&request)?;
        let reopened=Storage::new(dir.path().into());
        reopened.import_notes("room","frame",&[imported])?;
        assert_eq!(reopened.get("room","frame")?.items,vec![legacy]);
        Ok(())
    }
    #[test]
    fn old_home_json_keeps_title_description_and_default_type() -> Result<()> {
        let v=volume();let mut json=serde_json::to_value(&v)?;
        json.as_object_mut().unwrap().remove("semantic_type");
        let restored:Volume=serde_json::from_value(json)?;
        assert_eq!(restored,v);assert_eq!(restored.mobile().description,Some(v.description));
        Ok(())
    }
    #[test]
    fn persistence_isolation_and_crud() -> Result<()> {
        let dir = std::env::temp_dir().join(format!("semantic-volumes-{}", uuid::Uuid::new_v4()));
        let s = Storage::new(dir.clone());
        let mut v = volume();
        s.change("room", "room-frame", None, Some(v.clone()), "room-frame")?;
        assert!(s.get("other", "other-frame")?.items.is_empty());
        assert!(
            s.change("room", "room-frame", Some(&v.id), None, "wrong")
                .is_err()
        );
        v.title = "Dining".into();
        s.change(
            "room",
            "room-frame",
            Some(&v.id),
            Some(v.clone()),
            "room-frame",
        )?;
        assert_eq!(
            Storage::new(dir.clone()).get("room", "room-frame")?.items,
            vec![v.clone()]
        );
        s.change("room", "room-frame", Some(&v.id), None, "room-frame")?;
        assert!(
            Storage::new(dir.clone())
                .get("room", "room-frame")?
                .items
                .is_empty()
        );
        std::fs::remove_dir_all(dir)?;
        Ok(())
    }
    #[test]
    fn rejects_invalid_geometry_and_identity() {
        let mut values = Volumes::empty("frame");
        let mut v = volume();
        v.size[0] = 0.;
        values.items.push(v);
        assert!(values.validate("frame").is_err());
        values.items[0].size[0] = 1.;
        assert!(values.validate("different").is_err());
        values.items.push(values.items[0].clone());
        assert!(values.validate("frame").is_err());
    }
}
