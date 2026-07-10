//! `muas-artifacts` CLI — resolve a run's chains (live NDN or journal
//! fallback), render the artifact lenses through the console Binder, and
//! optionally print the provenance audit.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use muas_artifacts::audit::{all_verified, audit};
use muas_artifacts::chains::{from_journal_dir, resolve_live, Bootstrap, Mission};
use muas_artifacts::contracts::intent;
use muas_artifacts::render::{produce, RunSet};

const HELP: &str = "\
muas-artifacts — NDF artifact lenses over one mission dataset

USAGE:
    muas-artifacts --from-journal <DIR> [--out <DIR>] [--audit]
    muas-artifacts --bootstrap <FILE>   [--out <DIR>] [--audit]
    muas-artifacts --runs <DIR> --runs <DIR> [...] [--out <DIR>] [--audit]

SOURCES (exactly one form):
    --bootstrap <FILE>     live mode: JSON with engine links, chain addresses
                           (root/writer/writer key), and this reader's identity.
                           Mission data is fetched by name over NDN — no files.
    --from-journal <DIR>   offline mode: agent journals + dashboard recordings
                           (*.jsonl) republished through the identical Block
                           publish path — same hashing, different transport.
    --runs <DIR>           repeatable (>= 2): one journal dir per run; renders
                           the inter-run comparison (compare.html).

OUTPUT:
    --out <DIR>            output directory (default ./artifacts)
    --audit                print the provenance audit JSON (artifact ->
                           citations -> re-fetch + re-hash verification) and
                           exit non-zero if any hash fails to verify

Single-run sources render report.html + deck.html + demo.html;
--runs renders compare.html.
";

struct Args {
    bootstrap: Option<PathBuf>,
    from_journal: Option<PathBuf>,
    runs: Vec<PathBuf>,
    out: PathBuf,
    audit: bool,
}

fn parse_args(argv: &[String]) -> Result<Option<Args>, String> {
    let mut args = Args {
        bootstrap: None,
        from_journal: None,
        runs: Vec::new(),
        out: PathBuf::from("./artifacts"),
        audit: false,
    };
    let mut it = argv.iter();
    let next = |flag: &str, it: &mut std::slice::Iter<'_, String>| {
        it.next().cloned().ok_or_else(|| format!("{flag}: missing value"))
    };
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--help" | "-h" => return Ok(None),
            "--bootstrap" => args.bootstrap = Some(PathBuf::from(next(arg, &mut it)?)),
            "--from-journal" => args.from_journal = Some(PathBuf::from(next(arg, &mut it)?)),
            "--runs" => args.runs.push(PathBuf::from(next(arg, &mut it)?)),
            "--out" => args.out = PathBuf::from(next(arg, &mut it)?),
            "--audit" => args.audit = true,
            other => return Err(format!("unknown flag '{other}' (see --help)")),
        }
    }
    let sources = usize::from(args.bootstrap.is_some())
        + usize::from(args.from_journal.is_some())
        + usize::from(!args.runs.is_empty());
    if sources != 1 {
        return Err("exactly one of --bootstrap, --from-journal, --runs required (see --help)".into());
    }
    if !args.runs.is_empty() && args.runs.len() < 2 {
        return Err("--runs needs at least two run directories (one run? use --from-journal)".into());
    }
    Ok(Some(args))
}

async fn run(args: Args) -> Result<i32, String> {
    // 1 — resolve the run(s) into missions.
    let (missions, intents): (Vec<Mission>, Vec<&str>) = if let Some(path) = &args.bootstrap {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let bootstrap = Bootstrap::from_json(&text)?;
        (vec![resolve_live(&bootstrap).await?], vec![intent::REPORT, intent::DECK, intent::DEMO])
    } else if let Some(dir) = &args.from_journal {
        (vec![from_journal_dir(dir).await?], vec![intent::REPORT, intent::DECK, intent::DEMO])
    } else {
        let mut missions = Vec::new();
        for dir in &args.runs {
            missions.push(from_journal_dir(dir).await?);
        }
        (missions, vec![intent::COMPARE])
    };

    // 2 — one shared run set; render every lens through the Binder.
    let set = Arc::new(RunSet { runs: missions.iter().map(|m| Arc::clone(&m.dataset)).collect() });
    let artifacts = produce(&set, &intents)?;

    // 3 — write the HTML files.
    std::fs::create_dir_all(&args.out).map_err(|e| format!("{}: {e}", args.out.display()))?;
    for (name, art) in &artifacts {
        let path = args.out.join(name);
        std::fs::write(&path, art.html.as_bytes()).map_err(|e| format!("{}: {e}", path.display()))?;
        eprintln!(
            "wrote {} ({} bytes, {} block citations)",
            path.display(),
            art.html.len(),
            art.citations.len()
        );
    }

    // 4 — audit: the provenance proof.
    let mut code = 0;
    if args.audit {
        let mission_refs: Vec<&Mission> = missions.iter().collect();
        let artifact_map: BTreeMap<_, _> =
            artifacts.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        let report = audit(&mission_refs, &artifact_map)?;
        println!("{}", serde_json::to_string_pretty(&report).expect("audit serializes"));
        if !all_verified(&report) {
            eprintln!("AUDIT FAILED: at least one citation did not verify");
            code = 2;
        }
    }

    for mission in missions {
        mission.shutdown().await;
    }
    Ok(code)
}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("muas_artifacts=info")),
        )
        .with_writer(std::io::stderr)
        .init();
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let code = match parse_args(&argv) {
        Ok(None) => {
            print!("{HELP}");
            0
        }
        Ok(Some(args)) => {
            let rt = tokio::runtime::Runtime::new().expect("tokio runtime");
            match rt.block_on(run(args)) {
                Ok(code) => code,
                Err(err) => {
                    eprintln!("error: {err}");
                    1
                }
            }
        }
        Err(err) => {
            eprintln!("error: {err}");
            2
        }
    };
    std::process::exit(code);
}
