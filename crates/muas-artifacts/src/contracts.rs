//! The artifact lens manifests: one `mission-dataset` kind (and one
//! `run-set` kind for comparisons), four render contracts over it —
//! authored in the same raw-model style as `uas-fleet-data`'s L1 manifests
//! and `uas-console`'s lens vocabulary.
//!
//! This is the flotilla dogfood: `report.html`, `deck.html`, `demo.html`,
//! and `compare.html` are only reachable through the console `Binder`'s
//! **match → authorize → instantiate** pipeline, against these documents.
//! Four intents, four `Via::Native` renderer ids, ONE subject kind — the
//! artifacts differ only in *lens*, never in *data*.

use manifest::model::{Clause, Contract, Document, EdgeForm, Intent, Manifest, Subject, Term, Via, Vocabulary};
use manifest::{document_hash, encode_document, term_hash, EncodeError, FrozenDag, Hash};
use uas_console::TrustFrontier;
use uas_fleet_data::manifests::BuiltManifest;

/// The artifact intents.
pub mod intent {
    /// Operator/engineer mission report.
    pub const REPORT: &str = "artifact.report";
    /// Stakeholder slide deck.
    pub const DECK: &str = "artifact.deck";
    /// Self-contained replayable mini-map demo.
    pub const DEMO: &str = "artifact.demo";
    /// Inter-run comparison (settings deltas ↔ outcome deltas).
    pub const COMPARE: &str = "artifact.compare";
}

/// Native renderer ids the contracts dispatch through. App-neutral on
/// purpose (the uas-console invariant: a renderer id never names an app).
pub mod renderer {
    /// Renders the report lens.
    pub const REPORT: &str = "artifact.render.report";
    /// Renders the deck lens.
    pub const DECK: &str = "artifact.render.deck";
    /// Renders the demo lens.
    pub const DEMO: &str = "artifact.render.demo";
    /// Renders the comparison lens.
    pub const COMPARE: &str = "artifact.render.compare";
}

fn build(document: Document) -> BuiltManifest {
    let canonical_bytes = encode_document(&document).expect("statically authored document encodes");
    let hash = document_hash(&canonical_bytes);
    BuiltManifest { document, canonical_bytes, hash }
}

fn marker(label: &str, doc: &str) -> Term {
    Term { label: label.into(), doc: Some(doc.into()), ty: None, attrs: Vec::new() }
}

/// Identity hashes of the artifact lens terms.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ArtifactTerms {
    /// The subject kind: one run's mission dataset.
    pub mission_dataset: Hash,
    /// The subject kind of a comparison: an ordered set of run datasets.
    pub run_set: Hash,
    /// Target of `artifact.report`.
    pub mission_report: Hash,
    /// Target of `artifact.deck`.
    pub mission_deck: Hash,
    /// Target of `artifact.demo`.
    pub mission_replay: Hash,
    /// Target of `artifact.compare`.
    pub mission_comparison: Hash,
}

fn term_defs() -> Vec<Term> {
    vec![
        marker(
            "mission-dataset",
            "One run's mission data as resolved, content-addressed Blocks: \
             the run configuration (the inputs) plus every journaled event, \
             telemetry sample, coordination decision, service call, and \
             link-health datum (the outcomes), each carrying the hash, \
             chain, and seq of the Block it came from. There is exactly one \
             of these per run — every artifact is a lens over it, never a \
             copy of it.",
        ),
        marker(
            "run-set",
            "An ordered set of mission-datasets from different runs, for \
             input→output association across runs: which settings changed, \
             and how the outcomes moved with them (associated settings, \
             never claimed causality).",
        ),
        marker(
            "mission-report",
            "The operator/engineer lens: run configuration first, then \
             timeline, per-vehicle flight summaries, and network health, \
             each outcome correlated to the settings that shaped it, with \
             Block-hash provenance behind progressive disclosure.",
        ),
        marker(
            "mission-deck",
            "The stakeholder lens: few slides, high level — outcome, one \
             coordination chart, a link table — same data, same hashes, \
             less detail.",
        ),
        marker(
            "mission-replay",
            "The demo lens: the mission animated on a miniature map from \
             the same telemetry the report summarizes.",
        ),
        marker(
            "mission-comparison",
            "The cross-run lens: a sortable settings×outcomes table with \
             setting deltas highlighted, and an overlay map of every run's \
             tracks color-coded by run.",
        ),
    ]
}

/// Compute the artifact term hashes.
pub fn artifact_terms() -> ArtifactTerms {
    let defs = term_defs();
    let h = |i: usize| term_hash(&defs[i]).expect("artifact term hashes");
    ArtifactTerms {
        mission_dataset: h(0),
        run_set: h(1),
        mission_report: h(2),
        mission_deck: h(3),
        mission_replay: h(4),
        mission_comparison: h(5),
    }
}

/// The artifact lens vocabulary: the dataset/run-set kinds, the four lens
/// targets, and the edges that let ONE dataset express three lenses (and a
/// run-set the fourth) — the sharing is in the graph, not in copies.
pub fn artifact_vocabulary() -> BuiltManifest {
    let t = artifact_terms();
    build(Document::Vocabulary(Vocabulary {
        label: "artifact-lenses".into(),
        doc: Some(
            "Lenses over one named mission dataset. The same \
             mission-dataset term reaches report, deck, and replay — three \
             audiences, zero data copies; provenance is the Block hash."
                .into(),
        ),
        imports: Vec::new(),
        terms: term_defs(),
        edges: vec![
            EdgeForm::NarrowerThan { narrower: t.mission_dataset, broader: t.mission_report },
            EdgeForm::NarrowerThan { narrower: t.mission_dataset, broader: t.mission_deck },
            EdgeForm::NarrowerThan { narrower: t.mission_dataset, broader: t.mission_replay },
            EdgeForm::NarrowerThan { narrower: t.run_set, broader: t.mission_comparison },
        ],
        supersedes: None,
    }))
}

fn express(name: &str, target: Hash, via: &str) -> Clause {
    Clause::Express {
        intent: Intent { name: name.into(), attrs: Vec::new() },
        target,
        via: Some(Via::Native(via.into())),
        attrs: Vec::new(),
    }
}

/// The artifact render contract: four Express clauses, one per lens.
pub fn artifact_contract() -> BuiltManifest {
    let t = artifact_terms();
    build(Document::Contract(Contract {
        label: "mission-artifacts".into(),
        doc: Some(
            "Render contracts for the mission artifact lenses. Every clause \
             targets a lens term reachable from the ONE dataset kind — an \
             artifact is a matched, authorized rendering, never a private \
             export."
                .into(),
        ),
        imports: vec![artifact_vocabulary().hash],
        binds: Vec::new(),
        clauses: vec![
            express(intent::REPORT, t.mission_report, renderer::REPORT),
            express(intent::DECK, t.mission_deck, renderer::DECK),
            express(intent::DEMO, t.mission_replay, renderer::DEMO),
            express(intent::COMPARE, t.mission_comparison, renderer::COMPARE),
        ],
    }))
}

/// The frozen DAG + hashes the binder needs.
pub struct ArtifactPack {
    /// The DAG (vocabulary + contract + published instance manifests).
    pub dag: FrozenDag,
    /// Vocabulary document hash.
    pub vocabulary: Hash,
    /// Contract document hash.
    pub contract: Hash,
    /// The term hashes.
    pub terms: ArtifactTerms,
}

/// Assemble the artifact pack.
pub fn artifact_pack() -> ArtifactPack {
    let mut dag = FrozenDag::new();
    let vocabulary =
        dag.insert_bytes(&artifact_vocabulary().canonical_bytes).expect("vocabulary decodes");
    let contract =
        dag.insert_bytes(&artifact_contract().canonical_bytes).expect("contract decodes");
    ArtifactPack { dag, vocabulary, contract, terms: artifact_terms() }
}

impl ArtifactPack {
    /// Contracts to offer in match calls.
    pub fn contracts(&self) -> Vec<Hash> {
        vec![self.contract]
    }

    /// The trust frontier: our lens vocabulary.
    pub fn frontier(&self) -> TrustFrontier {
        TrustFrontier::from_vocabularies([self.vocabulary])
    }

    /// Publish an instance manifest of `ty` describing `subject` — the
    /// shape a dataset publisher emits alongside its chains.
    pub fn publish_instance(&mut self, ty: Hash, subject: &str) -> Result<Hash, EncodeError> {
        self.dag.insert_document(&Document::Manifest(Manifest {
            ty,
            label: None,
            describes: Subject::Name(subject.into()),
            entries: Vec::new(),
            edges: Vec::new(),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use manifest::dag::Resolution;
    use manifest::decode_document;

    #[test]
    fn documents_round_trip_canonically() {
        for built in [artifact_vocabulary(), artifact_contract()] {
            let decoded = decode_document(&built.canonical_bytes)
                .unwrap_or_else(|r| panic!("{r:?} decoding own canonical bytes"));
            assert_eq!(decoded.doc, built.document, "decode is lossless");
            assert_eq!(built.hash, document_hash(&built.canonical_bytes));
        }
        // Deterministic authoring.
        assert_eq!(artifact_contract().hash, artifact_contract().hash);
    }

    #[test]
    fn contract_covers_all_four_lenses_via_native() {
        let built = artifact_contract();
        let Document::Contract(c) = &built.document else { panic!("contract") };
        let mut expressed = Vec::new();
        for clause in &c.clauses {
            let Clause::Express { intent, via, .. } = clause else {
                panic!("artifact contract declares Express clauses only");
            };
            let Some(Via::Native(id)) = via else { panic!("native via") };
            expressed.push((intent.name.as_str(), id.as_str()));
        }
        assert_eq!(
            expressed,
            [
                (intent::REPORT, renderer::REPORT),
                (intent::DECK, renderer::DECK),
                (intent::DEMO, renderer::DEMO),
                (intent::COMPARE, renderer::COMPARE),
            ]
        );
    }

    #[test]
    fn one_dataset_term_reaches_three_lenses() {
        let built = artifact_vocabulary();
        let Document::Vocabulary(v) = &built.document else { panic!("vocabulary") };
        let t = artifact_terms();
        for target in [t.mission_report, t.mission_deck, t.mission_replay] {
            assert!(
                v.edges.contains(&EdgeForm::NarrowerThan {
                    narrower: t.mission_dataset,
                    broader: target
                }),
                "the ONE dataset kind reaches every single-run lens"
            );
        }
        assert!(v
            .edges
            .contains(&EdgeForm::NarrowerThan { narrower: t.run_set, broader: t.mission_comparison }));
    }

    #[test]
    fn renderer_ids_never_name_an_app() {
        for id in [renderer::REPORT, renderer::DECK, renderer::DEMO, renderer::COMPARE] {
            assert!(!id.to_ascii_lowercase().contains("muas"), "renderer id `{id}` names an app");
        }
    }

    #[test]
    fn pack_import_closures_complete() {
        let pack = artifact_pack();
        for h in [pack.contract, pack.vocabulary] {
            match pack.dag.import_closure(&h) {
                Resolution::Complete(_) => {}
                Resolution::Unresolved { missing, .. } => {
                    panic!("import closure incomplete, missing {missing:?}")
                }
            }
        }
    }
}
