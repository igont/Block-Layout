//! A cut belongs to the physical end. Later planning reads it without recreating it.
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde::ser::SerializeStruct;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "id", rename_all = "snake_case")]
pub enum CutSource {
    Opening(String),
    Beam(String),
    Wall(String),
    Stock,
    Imported,
}

impl CutSource {
    pub fn from_obstacle(source: &str) -> Self {
        if let Some(id) = source.strip_prefix("opening:") { Self::Opening(id.into()) }
        else if let Some(id) = source.strip_prefix("beam:") { Self::Beam(id.into()) }
        else if let Some(id) = source.strip_prefix("wall:") { Self::Wall(id.into()) }
        else { Self::Stock }
    }
    pub fn obstacle_id(&self) -> Option<String> {
        match self {
            Self::Opening(id) => Some(format!("opening:{id}")),
            Self::Beam(id) => Some(format!("beam:{id}")),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CutFace {
    /// Exclusion/saw reference plane along the local stock axis, in 0.01 mm.
    /// Factory profile removal is an inset from this plane, never a second cut.
    pub plane_centimm: i64,
    pub sources: Vec<CutSource>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "state", content = "cut", rename_all = "snake_case")]
pub enum EndState {
    FactoryUncut,
    FactoryCut(CutFace),
    SawCut(CutFace),
}

impl EndState {
    pub fn natural(&self) -> bool { !matches!(self, Self::SawCut(_)) }
    pub fn cut(&self) -> bool { !matches!(self, Self::FactoryUncut) }
    pub fn face(&self) -> Option<&CutFace> {
        match self { Self::FactoryUncut => None, Self::FactoryCut(c) | Self::SawCut(c) => Some(c) }
    }
    fn shifted(&self, offset: i64) -> Self {
        let mut end = self.clone();
        if let Self::FactoryCut(c) | Self::SawCut(c) = &mut end { c.plane_centimm -= offset; }
        end
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EndStates { left: EndState, right: EndState }

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CutPlaneMismatch {
    pub recorded_centimm: i64,
    pub requested_centimm: i64,
}

impl Default for EndStates { fn default() -> Self { Self::factory() } }

impl EndStates {
    pub fn factory() -> Self { Self { left: EndState::FactoryUncut, right: EndState::FactoryUncut } }
    pub fn left(&self) -> &EndState { &self.left }
    pub fn right(&self) -> &EndState { &self.right }
    pub fn factory_cuts(left: bool, right: bool, length: i64, sources: &[String]) -> Self {
        let mut ends = Self::factory();
        let cut_sources: Vec<_> = sources.iter().filter(|s| s.starts_with("opening:") || s.starts_with("beam:"))
            .map(|s| CutSource::from_obstacle(s)).collect();
        let fallback = sources.iter().find(|s| s.starts_with("wall:"))
            .map_or(CutSource::Stock, |s| CutSource::from_obstacle(s));
        for (is_left, hidden, plane) in [(true, left, 0), (false, right, length)] {
            if hidden {
                let sources = if cut_sources.is_empty() { vec![fallback.clone()] } else { cut_sources.clone() };
                let end = EndState::FactoryCut(CutFace { plane_centimm: plane, sources });
                if is_left { ends.left = end; } else { ends.right = end; }
            }
        }
        ends
    }
    /// Compatibility input is converted once; booleans are not retained internally.
    pub fn from_flags(hide_left: bool, hide_right: bool, natural_left: bool, natural_right: bool) -> Self {
        let end = |hidden: bool, natural: bool| {
            let cut = CutFace { plane_centimm: 0, sources: vec![CutSource::Imported] };
            if !natural { EndState::SawCut(cut) }
            else if hidden { EndState::FactoryCut(cut) }
            else { EndState::FactoryUncut }
        };
        Self { left: end(hide_left, natural_left), right: end(hide_right, natural_right) }
    }
    pub fn stock(natural_left: bool, natural_right: bool, length: i64) -> Self {
        let end = |natural: bool, plane: i64| if natural { EndState::FactoryUncut }
            else { EndState::SawCut(CutFace { plane_centimm: plane, sources: vec![CutSource::Stock] }) };
        Self { left: end(natural_left, 0), right: end(natural_right, length) }
    }
    /// Existing cuts cannot be undone. Coincident causes enrich the same face.
    pub fn record_cut(&mut self, left: bool, plane_centimm: i64, source: CutSource) -> Result<(), CutPlaneMismatch> {
        let end = if left { &mut self.left } else { &mut self.right };
        match end {
            EndState::FactoryUncut => *end = EndState::FactoryCut(CutFace { plane_centimm, sources: vec![source] }),
            EndState::FactoryCut(cut) | EndState::SawCut(cut) => {
                // Old boolean input has no plane. The first real operation binds it once.
                if cut.sources == [CutSource::Imported] && source != CutSource::Imported {
                    cut.plane_centimm = plane_centimm;
                    cut.sources.clear();
                } else if cut.plane_centimm != plane_centimm {
                    return Err(CutPlaneMismatch { recorded_centimm: cut.plane_centimm, requested_centimm: plane_centimm });
                }
                if !cut.sources.contains(&source) { cut.sources.push(source); }
            }
        }
        Ok(())
    }
    pub fn slice(&self, from: i64, to: i64, length: i64, left_source: CutSource, right_source: CutSource) -> Self {
        Self {
            left: if from == 0 { self.left.clone() } else {
                EndState::SawCut(CutFace { plane_centimm: 0, sources: vec![left_source] }) },
            right: if to == length { self.right.shifted(from) } else {
                EndState::SawCut(CutFace { plane_centimm: to - from, sources: vec![right_source] }) },
        }
    }
    pub fn reversed(&self, length: i64) -> Self {
        let reverse = |end: &EndState| {
            let mut end = end.clone();
            if let EndState::FactoryCut(c) | EndState::SawCut(c) = &mut end {
                c.plane_centimm = length - c.plane_centimm;
            }
            end
        };
        Self { left: reverse(&self.right), right: reverse(&self.left) }
    }
    pub fn outer(left: &Self, right: &Self, right_offset: i64) -> Self {
        Self { left: left.left.clone(), right: right.right.shifted(-right_offset) }
    }
}

impl Serialize for EndStates {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut state = serializer.serialize_struct("EndStates", 6)?;
        state.serialize_field("left_end", &self.left)?;
        state.serialize_field("right_end", &self.right)?;
        state.serialize_field("hide_spikes_left", &self.left.cut())?;
        state.serialize_field("hide_spikes_right", &self.right.cut())?;
        state.serialize_field("natural_end_left", &self.left.natural())?;
        state.serialize_field("natural_end_right", &self.right.natural())?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for EndStates {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        struct Input {
            left_end: Option<EndState>, right_end: Option<EndState>,
            #[serde(default)] hide_spikes_left: bool, #[serde(default)] hide_spikes_right: bool,
            #[serde(default = "yes")] natural_end_left: bool,
            #[serde(default = "yes")] natural_end_right: bool,
        }
        fn yes() -> bool { true }
        let input = Input::deserialize(deserializer)?;
        if (!input.natural_end_left && !input.hide_spikes_left && input.left_end.is_none())
            || (!input.natural_end_right && !input.hide_spikes_right && input.right_end.is_none()) {
            return Err(serde::de::Error::custom("Artificial ends cannot carry spikes"));
        }
        let legacy = Self::from_flags(input.hide_spikes_left, input.hide_spikes_right,
            input.natural_end_left, input.natural_end_right);
        Ok(Self { left: input.left_end.unwrap_or(legacy.left), right: input.right_end.unwrap_or(legacy.right) })
    }
}
