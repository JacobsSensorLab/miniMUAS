//! Deploying pinned code to the fleet (PROTOCOL.md I4, I5, I8).
//!
//! Two phases. `plan` only reads — the local checkouts, the config repo, GitHub (hash prefetch) —
//! and writes `deploys/<id>.plan.json`. `execute` re-verifies the plan against the config repo,
//! then: edit pins → commit → push → build every closure → canary → other airframes → GCS →
//! verify the fleet runs what was built → restart roles in the I5 order → health gate, recording
//! each step into `deploys/<id>.json`.

use std::collections::BTreeMap;
use std::path::Path;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::cells::{self, out_json, tail};
use crate::config::{Config, Node};
use crate::jobs::JobCtx;
use crate::remote::{self, Output, sh_quote};
use crate::state::{Recorder, now_ms};
use crate::status;

const GIT: Duration = Duration::from_secs(120);
const PREFETCH: Duration = Duration::from_secs(900);
/// ndn-fwd is a full Rust workspace build for aarch64 on nixbuild.net.
const BUILD: Duration = Duration::from_secs(4 * 3600);
const COPY: Duration = Duration::from_secs(3600);
const SWITCH: Duration = Duration::from_secs(900);
const QUICK: Duration = Duration::from_secs(60);
const CANARY_HEALTH: Duration = Duration::from_secs(180);
const HEALTH_GATE: Duration = Duration::from_secs(300);
/// After an ssh drop during activation, how long `/run/current-system` may take to flip.
const SETTLE_AFTER_DROP: Duration = Duration::from_secs(300);
/// The config flake input carrying the miniMUAS sources.
pub const MINIMUAS_INPUT: &str = "minimuas-src";
const MAX_COMMITS_IN_MESSAGE: usize = 40;
const SSH_CONNECT_FAILURE: i32 = 255;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RepoChange {
    pub name: String,
    pub old_rev: String,
    pub new_rev: String,
    pub new_hash: String,
    pub commits: Vec<String>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Plan {
    pub id: String,
    pub created_unix_ms: u64,
    /// Only the repos whose pin changes.
    pub repos: Vec<RepoChange>,
    pub minimuas: Option<RepoChange>,
    pub config_head: String,
    /// Rollout order: canary, other airframes, GCS.
    pub nodes: Vec<String>,
    pub warnings: Vec<String>,
    /// Every pinned repo's rev once this plan is in force, changed or not, plus
    /// `minimuas-src` — the "pins of the deploy in force" a run manifest carries (I8).
    #[serde(default)]
    pub pins: BTreeMap<String, String>,
}

// ---------------------------------------------------------------------------------------------
// Pins file: `srcs.<name> = fetchFromGitHub { owner = ".."; repo = ".."; rev = ".."; hash = ".."; };`

/// One `fetchFromGitHub` block of the pins file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pin {
    pub owner: Option<String>,
    pub repo: Option<String>,
    pub rev: String,
    pub hash: String,
}

fn is_comment(line: &str) -> bool {
    line.trim_start().starts_with('#')
}

fn is_ident(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'-' || b == b'_' || b == b'.'
}

/// Byte range of the quoted value of `key = "…"` in `line` (several attributes may share a line).
fn attr_span(line: &str, key: &str) -> Option<std::ops::Range<usize>> {
    let bytes = line.as_bytes();
    let mut from = 0;
    while let Some(off) = line[from..].find(key) {
        let start = from + off;
        from = start + key.len();
        if start > 0 && is_ident(bytes[start - 1]) {
            continue;
        }
        let Some(after_eq) = line[from..].trim_start().strip_prefix('=') else {
            continue;
        };
        let Some(value) = after_eq.trim_start().strip_prefix('"') else {
            continue;
        };
        let value_start = line.len() - value.len();
        return Some(value_start..value_start + value.find('"')?);
    }
    None
}

fn attr_value<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    attr_span(line, key).map(|r| &line[r])
}

/// `<name> = fetchFromGitHub {` — the name must be followed by `=`, so `ndn-radio` does not
/// match `ndn-radio-drivers`.
fn block_start_name(line: &str) -> Option<&str> {
    if is_comment(line) {
        return None;
    }
    let (name, rest) = line.trim_start().split_once('=')?;
    let name = name.trim_end();
    (!name.is_empty()
        && name.bytes().all(is_ident)
        && rest.trim_start().starts_with("fetchFromGitHub"))
    .then_some(name)
}

/// Every `fetchFromGitHub` block name in the pins file.
pub fn pin_names(text: &str) -> Vec<String> {
    text.lines()
        .filter_map(block_start_name)
        .map(str::to_string)
        .collect()
}

struct Block {
    rev_line: usize,
    hash_line: usize,
    pin: Pin,
}

fn find_block(lines: &[&str], name: &str) -> Result<Block> {
    let starts: Vec<usize> = lines
        .iter()
        .enumerate()
        .filter(|(_, l)| block_start_name(l) == Some(name))
        .map(|(i, _)| i)
        .collect();
    let start = match starts.as_slice() {
        [s] => *s,
        [] => bail!("pins file has no `{name} = fetchFromGitHub {{` block"),
        many => bail!("pins file has {} `{name}` blocks", many.len()),
    };
    let (mut rev, mut hash, mut owner, mut repo) = (None, None, None, None);
    let mut closed = false;
    for (i, line) in lines.iter().enumerate().skip(start + 1) {
        if is_comment(line) {
            continue;
        }
        if line.trim_start().starts_with('}') {
            closed = true;
            break;
        }
        // Comment lines are skipped above: the ndn-rs block documents its history in them.
        for (key, slot) in [("rev", &mut rev), ("hash", &mut hash)] {
            if let Some(v) = attr_value(line, key)
                && slot.replace((i, v.to_string())).is_some()
            {
                bail!("`{name}` block has more than one `{key}`");
            }
        }
        owner = owner.or_else(|| attr_value(line, "owner").map(str::to_string));
        repo = repo.or_else(|| attr_value(line, "repo").map(str::to_string));
    }
    if !closed {
        bail!("`{name}` block is not closed");
    }
    let (rev_line, rev) = rev.ok_or_else(|| anyhow!("`{name}` block has no rev"))?;
    let (hash_line, hash) = hash.ok_or_else(|| anyhow!("`{name}` block has no hash"))?;
    Ok(Block {
        rev_line,
        hash_line,
        pin: Pin {
            owner,
            repo,
            rev,
            hash,
        },
    })
}

pub fn read_pin(text: &str, name: &str) -> Result<Pin> {
    let lines: Vec<&str> = text.split_inclusive('\n').collect();
    Ok(find_block(&lines, name)?.pin)
}

fn replace_attr(line: &str, key: &str, value: &str) -> String {
    let r = attr_span(line, key).expect("span found by find_block");
    format!("{}{value}{}", &line[..r.start], &line[r.end..])
}

fn check_rev(rev: &str) -> Result<()> {
    if rev.len() != 40 || !rev.bytes().all(|b| b.is_ascii_hexdigit()) {
        bail!("'{rev}' is not a full 40-hex commit id");
    }
    Ok(())
}

/// Set the rev AND hash of block `name` — always together: bumping a rev without its hash
/// silently rebuilt the old source (PROTOCOL.md I4). Nothing else in the file changes.
pub fn edit_pin(text: &str, name: &str, rev: &str, hash: &str) -> Result<String> {
    check_rev(rev)?;
    if !hash.starts_with("sha256-") {
        bail!("'{hash}' is not an SRI sha256 hash");
    }
    let lines: Vec<&str> = text.split_inclusive('\n').collect();
    let block = find_block(&lines, name)?;
    Ok(lines
        .iter()
        .enumerate()
        .map(|(i, line)| match i {
            _ if i == block.rev_line => replace_attr(line, "rev", rev),
            _ if i == block.hash_line => replace_attr(line, "hash", hash),
            _ => (*line).to_string(),
        })
        .collect())
}

// ---------------------------------------------------------------------------------------------
// flake.nix: `inputs.minimuas-src.url = "git+ssh://…?ref=refs/heads/<branch>&rev=<rev>";`

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlakeInputPin {
    pub url: String,
    pub rev: String,
    /// The branch named by `ref=refs/heads/…`, which nix fetches the rev through.
    pub branch: Option<String>,
}

fn input_url_key(input: &str) -> String {
    format!("inputs.{input}.url")
}

fn input_line(lines: &[&str], input: &str) -> Result<usize> {
    let key = input_url_key(input);
    let found: Vec<usize> = lines
        .iter()
        .enumerate()
        .filter(|(_, l)| !is_comment(l) && attr_span(l, &key).is_some())
        .map(|(i, _)| i)
        .collect();
    match found.as_slice() {
        [i] => Ok(*i),
        [] => bail!("flake.nix has no `{key} = \"…\";` line (other pin forms are not supported)"),
        _ => bail!("flake.nix sets `{key}` more than once"),
    }
}

/// Byte range of the `rev=` query value inside `url`.
fn url_rev_span(url: &str) -> Option<std::ops::Range<usize>> {
    let query_at = url.find('?')? + 1;
    let mut offset = query_at;
    for param in url[query_at..].split('&') {
        if let Some(v) = param.strip_prefix("rev=") {
            let start = offset + "rev=".len();
            return Some(start..start + v.len());
        }
        offset += param.len() + 1;
    }
    None
}

pub fn read_flake_input(text: &str, input: &str) -> Result<FlakeInputPin> {
    let lines: Vec<&str> = text.lines().collect();
    let line = lines[input_line(&lines, input)?];
    let url = attr_value(line, &input_url_key(input)).expect("matched by input_line");
    let rev = url_rev_span(url)
        .map(|r| url[r].to_string())
        .ok_or_else(|| anyhow!("`{input}` url has no rev= pin: {url}"))?;
    let branch = url
        .split_once('?')
        .and_then(|(_, q)| q.split('&').find_map(|p| p.strip_prefix("ref=")))
        .map(|r| r.strip_prefix("refs/heads/").unwrap_or(r).to_string());
    Ok(FlakeInputPin {
        url: url.to_string(),
        rev,
        branch,
    })
}

/// `url` with its `rev=` query value replaced.
fn url_with_rev(url: &str, rev: &str) -> Result<String> {
    let r = url_rev_span(url).ok_or_else(|| anyhow!("url has no rev= pin: {url}"))?;
    Ok(format!("{}{rev}{}", &url[..r.start], &url[r.end..]))
}

/// Set the `rev=` of input `input`'s url; `nix flake lock --update-input` then relocks it.
pub fn edit_flake_input_rev(text: &str, input: &str, rev: &str) -> Result<String> {
    check_rev(rev)?;
    let lines: Vec<&str> = text.split_inclusive('\n').collect();
    let at = input_line(&lines, input)?;
    let line = lines[at];
    let span = attr_span(line, &input_url_key(input)).expect("matched by input_line");
    let url = url_with_rev(&line[span.clone()], rev)?;
    let edited = format!("{}{url}{}", &line[..span.start], &line[span.end..]);
    Ok(lines
        .iter()
        .enumerate()
        .map(|(i, l)| {
            if i == at {
                edited.clone()
            } else {
                (*l).to_string()
            }
        })
        .collect())
}

// ---------------------------------------------------------------------------------------------
// Local checkouts

async fn git(dir: &Path, args: &[&str]) -> Result<Output> {
    remote::local("git", args, Some(dir), &[], GIT).await
}

async fn git_out(dir: &Path, args: &[&str]) -> Result<String> {
    let out = git(dir, args).await?;
    Ok(out
        .stdout_ok()
        .with_context(|| format!("in {}", dir.display()))?
        .trim()
        .to_string())
}

fn lines_of(s: &str) -> Vec<String> {
    s.lines()
        .map(str::trim)
        .filter(|l| !l.is_empty())
        .map(str::to_string)
        .collect()
}

fn short(rev: &str) -> &str {
    &rev[..rev.len().min(8)]
}

/// A local checkout after `git fetch`: what it is on and whether it is pushed.
struct Checkout {
    head: String,
    branch: Option<String>,
    upstream: String,
    upstream_tip: String,
    /// Tracked files with changes (`git status --porcelain --untracked-files=no`).
    dirty: Vec<String>,
}

/// `upstream`: the ref to compare against; default `@{u}`, else `origin/<current branch>`.
async fn inspect(dir: &Path, upstream: Option<&str>) -> Result<Checkout> {
    git(dir, &["fetch", "-q"])
        .await?
        .stdout_ok()
        .with_context(|| format!("fetching {} (needed to know what is pushed)", dir.display()))?;
    let head = git_out(dir, &["rev-parse", "HEAD"]).await?;
    let branch = git_out(dir, &["symbolic-ref", "-q", "--short", "HEAD"])
        .await
        .ok();
    let upstream = match upstream {
        Some(u) => u.to_string(),
        None => match git_out(
            dir,
            &["rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"],
        )
        .await
        {
            Ok(u) => u,
            Err(_) => format!(
                "origin/{}",
                branch
                    .as_deref()
                    .ok_or_else(|| anyhow!("{}: detached HEAD with no upstream", dir.display()))?
            ),
        },
    };
    let upstream_tip = git_out(
        dir,
        &["rev-parse", "--verify", &format!("{upstream}^{{commit}}")],
    )
    .await
    .with_context(|| format!("{}: no upstream '{upstream}'", dir.display()))?;
    let dirty = lines_of(&git_out(dir, &["status", "--porcelain", "--untracked-files=no"]).await?);
    Ok(Checkout {
        head,
        branch,
        upstream,
        upstream_tip,
        dirty,
    })
}

async fn is_ancestor(dir: &Path, rev: &str, of: &str) -> Result<bool> {
    let out = git(dir, &["merge-base", "--is-ancestor", rev, of]).await?;
    match out.status {
        Some(0) => Ok(true),
        Some(1) => Ok(false),
        _ => Err(out.stdout_ok().unwrap_err()),
    }
}

async fn has_commit(dir: &Path, rev: &str) -> bool {
    git(dir, &["cat-file", "-e", &format!("{rev}^{{commit}}")])
        .await
        .is_ok_and(|o| o.ok())
}

/// Where a repo's new hash comes from.
enum Source<'a> {
    /// `fetchFromGitHub` (the ndn-fwd pins): `nix flake prefetch github:<owner>/<repo>/<rev>`.
    GitHub { owner: &'a str, repo: &'a str },
    /// A flake `git+…` input url carrying `rev=`.
    FlakeUrl(&'a str),
}

impl Source<'_> {
    fn prefetch_ref(&self, rev: &str) -> Result<String> {
        match self {
            Source::GitHub { owner, repo } => Ok(format!("github:{owner}/{repo}/{rev}")),
            Source::FlakeUrl(url) => url_with_rev(url, rev),
        }
    }
}

/// SRI hash of the unpacked source, exactly what `fetchFromGitHub`/`flake.lock` pin.
async fn prefetch(flake_ref: &str) -> Result<String> {
    let out = remote::local(
        "nix",
        &["flake", "prefetch", flake_ref, "--json"],
        None,
        &[],
        PREFETCH,
    )
    .await?;
    let v: Value = serde_json::from_str(out.stdout_ok()?)
        .with_context(|| format!("parsing `nix flake prefetch {flake_ref}` output"))?;
    v["hash"]
        .as_str()
        .filter(|h| h.starts_with("sha256-"))
        .map(str::to_string)
        .ok_or_else(|| anyhow!("`nix flake prefetch {flake_ref}` returned no sha256 hash"))
}

/// Resolve and vet one repo's requested rev against its current pin. Returns the rev in force
/// after the deploy and the change, if any. Refuses (I4) a rev that is not on the upstream — the
/// fleet builds from GitHub — and a pin change out of a dirty checkout, whose deployer would
/// believe their local edits ship.
async fn plan_repo(
    name: &str,
    dir: &Path,
    upstream: Option<&str>,
    old_rev: &str,
    requested: Option<&str>,
    source: Source<'_>,
    warnings: &mut Vec<String>,
) -> Result<(String, Option<RepoChange>)> {
    let co = inspect(dir, upstream).await?;
    let new_rev = match requested {
        Some(r) => git_out(dir, &["rev-parse", "--verify", &format!("{r}^{{commit}}")])
            .await
            .with_context(|| format!("{name}: unknown rev '{r}'"))?,
        None => co.head.clone(),
    };
    if !is_ancestor(dir, &new_rev, &co.upstream).await? {
        bail!(
            "{name}: {} is not on {} — push it first (the fleet builds from GitHub)",
            short(&new_rev),
            co.upstream
        );
    }
    if requested.is_none() && co.head != co.upstream_tip {
        warnings.push(format!(
            "{name}: local {} is behind {} ({}); deploying the local HEAD",
            co.branch.as_deref().unwrap_or("HEAD"),
            co.upstream,
            short(&co.upstream_tip)
        ));
    }
    if new_rev == old_rev {
        if !co.dirty.is_empty() {
            warnings.push(format!(
                "{name}: unchanged, but the checkout has uncommitted tracked changes that are NOT deployed: {}",
                co.dirty.join(", ")
            ));
        }
        return Ok((new_rev, None));
    }
    if !co.dirty.is_empty() {
        bail!(
            "{name}: checkout {} has uncommitted tracked changes ({}) — commit and push, or stash, first",
            dir.display(),
            co.dirty.join(", ")
        );
    }
    let new_hash = prefetch(&source.prefetch_ref(&new_rev)?).await?;
    let commits = if has_commit(dir, old_rev).await {
        let range = format!("{old_rev}..{new_rev}");
        let dropped_range = format!("{new_rev}..{old_rev}");
        let log = |r: String| async move {
            git_out(dir, &["log", "--no-decorate", "--format=%h %s", &r])
                .await
                .map(|s| lines_of(&s))
        };
        let dropped = log(dropped_range).await?;
        if !dropped.is_empty() {
            warnings.push(format!(
                "{name}: {} does not contain {} commit(s) of the current pin {}: {}",
                short(&new_rev),
                dropped.len(),
                short(old_rev),
                dropped
                    .iter()
                    .take(10)
                    .cloned()
                    .collect::<Vec<_>>()
                    .join("; ")
            ));
        }
        log(range).await?
    } else {
        warnings.push(format!(
            "{name}: current pin {} is not in the local checkout; commit list unavailable",
            short(old_rev)
        ));
        Vec::new()
    };
    Ok((
        new_rev.clone(),
        Some(RepoChange {
            name: name.to_string(),
            old_rev: old_rev.to_string(),
            new_rev,
            new_hash,
            commits,
        }),
    ))
}

/// The config repo must be clean, on the deploy branch, and equal to `origin/<branch>` —
/// the pins commit is pushed there before anything builds (I4).
async fn verify_config_repo(cfg: &Config) -> Result<String> {
    let dir = &cfg.deploy.config_repo;
    let branch = &cfg.deploy.config_branch;
    let upstream = format!("origin/{branch}");
    let co = inspect(dir, Some(&upstream)).await?;
    if co.branch.as_deref() != Some(branch) {
        bail!(
            "config repo {} is on '{}', not '{branch}'",
            dir.display(),
            co.branch.as_deref().unwrap_or("detached HEAD")
        );
    }
    if !co.dirty.is_empty() {
        bail!(
            "config repo {} has uncommitted tracked changes: {}",
            dir.display(),
            co.dirty.join(", ")
        );
    }
    if co.head != co.upstream_tip {
        bail!(
            "config repo {branch} is at {} but {upstream} is at {} — pull/push first",
            short(&co.head),
            short(&co.upstream_tip)
        );
    }
    Ok(co.head)
}

/// Resolve revs (default: every pinned repo's local HEAD), verify they are clean and pushed,
/// hash and describe each change, and record the plan. Reads only: the fleet and the config
/// repo's files are untouched (`git fetch` updates remote-tracking refs, nothing else).
pub async fn plan(
    cfg: &Config,
    rec: &Recorder,
    revs: BTreeMap<String, String>,
    minimuas_rev: Option<String>,
) -> Result<Plan> {
    for name in revs.keys() {
        if !cfg.deploy.repos.iter().any(|r| &r.name == name) {
            let known: Vec<&str> = cfg.deploy.repos.iter().map(|r| r.name.as_str()).collect();
            bail!("unknown repo '{name}' (pinned: {})", known.join(", "));
        }
    }
    let config_head = verify_config_repo(cfg).await?;
    let pins_path = cfg.deploy.config_repo.join(&cfg.deploy.pins_file);
    let pins_text = std::fs::read_to_string(&pins_path)
        .with_context(|| format!("reading {}", pins_path.display()))?;

    let mut warnings = Vec::new();
    let mut repos = Vec::new();
    let mut pins = BTreeMap::new();
    for name in pin_names(&pins_text) {
        if !cfg.deploy.repos.iter().any(|r| r.name == name) {
            warnings.push(format!(
                "pins file block '{name}' is not in fleet.toml [[deploy.repo]]; it is never bumped"
            ));
        }
    }
    for repo in &cfg.deploy.repos {
        let pin = read_pin(&pins_text, &repo.name)?;
        if pin.owner.as_deref().is_some_and(|o| o != repo.owner)
            || pin.repo.as_deref().is_some_and(|r| r != repo.name)
        {
            bail!(
                "pins block '{}' fetches {:?}/{:?}, but fleet.toml says {}/{}",
                repo.name,
                pin.owner,
                pin.repo,
                repo.owner,
                repo.name
            );
        }
        let source = Source::GitHub {
            owner: &repo.owner,
            repo: &repo.name,
        };
        let (rev, change) = plan_repo(
            &repo.name,
            &repo.local,
            None,
            &pin.rev,
            revs.get(&repo.name).map(String::as_str),
            source,
            &mut warnings,
        )
        .await?;
        pins.insert(repo.name.clone(), rev);
        repos.extend(change);
    }

    let flake_path = cfg.deploy.config_repo.join("flake.nix");
    let flake_text = std::fs::read_to_string(&flake_path)
        .with_context(|| format!("reading {}", flake_path.display()))?;
    let mm = read_flake_input(&flake_text, MINIMUAS_INPUT)?;
    let minimuas = match minimuas_rev {
        None => {
            pins.insert(MINIMUAS_INPUT.to_string(), mm.rev.clone());
            None
        }
        Some(requested) => {
            let dir = cfg.deploy.minimuas_local.as_deref().ok_or_else(|| {
                anyhow!(
                    "fleet.toml [deploy] minimuas_local is not set; cannot plan a miniMUAS bump"
                )
            })?;
            // nix fetches the rev through the url's ref, so the rev must be on that branch.
            let upstream = mm.branch.as_ref().map(|b| format!("origin/{b}"));
            let (rev, change) = plan_repo(
                MINIMUAS_INPUT,
                dir,
                upstream.as_deref(),
                &mm.rev,
                Some(&requested),
                Source::FlakeUrl(&mm.url),
                &mut warnings,
            )
            .await?;
            pins.insert(MINIMUAS_INPUT.to_string(), rev);
            change
        }
    };
    if repos.is_empty() && minimuas.is_none() {
        warnings.push(format!(
            "no pin changes: executing this plan rebuilds and rolls out config {} as it is",
            short(&config_head)
        ));
    }

    let plan = Plan {
        id: rec.new_id("deploy"),
        created_unix_ms: now_ms(),
        repos,
        minimuas,
        config_head,
        nodes: cfg.rollout_order().iter().map(|n| n.name.clone()).collect(),
        warnings,
        pins,
    };
    rec.write_json(&format!("deploys/{}.plan.json", plan.id), &plan)?;
    rec.ledger(
        "deploy_plan",
        json!({
            "plan": plan.id,
            "changes": plan.repos.iter().chain(&plan.minimuas)
                .map(|c| format!("{} {} -> {}", c.name, short(&c.old_rev), short(&c.new_rev)))
                .collect::<Vec<_>>(),
            "config_head": plan.config_head,
            "warnings": plan.warnings,
        }),
    );
    Ok(plan)
}

// ---------------------------------------------------------------------------------------------
// Execute

#[derive(Debug, Serialize)]
struct Step {
    name: String,
    ok: bool,
    elapsed_ms: u64,
    detail: Value,
}

#[derive(Debug, Serialize)]
struct NodeRollout {
    node: String,
    system: String,
    previous_system: Option<String>,
    /// The switch command was issued: the node may run the new system.
    switched: bool,
    copy: Vec<Value>,
    switch: Option<Value>,
    current_system: Option<String>,
    ok: bool,
    elapsed_ms: u64,
    note: String,
}

#[derive(Debug, Serialize)]
struct DeployRecord {
    id: String,
    plan: Plan,
    job: String,
    started_unix_ms: u64,
    finished_unix_ms: Option<u64>,
    /// running | succeeded | failed
    outcome: String,
    error: Option<String>,
    /// The cell every node was on before the deploy; the final health gate expects it back.
    expected_cell: Option<String>,
    pins_commit: Option<String>,
    build_attempts: Vec<Value>,
    /// node → built toplevel.
    built: BTreeMap<String, String>,
    steps: Vec<Step>,
    nodes: Vec<NodeRollout>,
    verification: Value,
}

impl DeployRecord {
    fn step<T>(&mut self, job: &JobCtx, name: &str, t: Instant, r: &Result<T>, detail: Value) {
        let ok = r.is_ok();
        let detail = match r {
            Ok(_) => detail,
            Err(e) => json!({ "error": format!("{e:#}"), "detail": detail }),
        };
        job.log(format!(
            "step {name}: {} in {} ms",
            if ok { "ok" } else { "FAILED" },
            t.elapsed().as_millis()
        ));
        self.steps.push(Step {
            name: name.to_string(),
            ok,
            elapsed_ms: t.elapsed().as_millis() as u64,
            detail,
        });
    }
}

/// Execute a plan as a job. Caller holds the fleet lock (I1) and has checked armed (I2).
/// The deploy record is written whatever the outcome.
pub async fn execute(cfg: &Config, rec: &Recorder, job: &JobCtx, plan_id: &str) -> Result<Value> {
    let plan: Plan = rec
        .read_json(&format!("deploys/{plan_id}.plan.json"))
        .with_context(|| {
            format!("no plan '{plan_id}' (call fleet_deploy without plan_id first)")
        })?;
    if rec
        .read_json::<Value>(&format!("deploys/{plan_id}.json"))
        .is_ok()
    {
        bail!("plan {plan_id} was already executed (deploys/{plan_id}.json exists); plan again");
    }
    let mut record = DeployRecord {
        id: plan.id.clone(),
        plan,
        job: job.id.clone(),
        started_unix_ms: now_ms(),
        finished_unix_ms: None,
        outcome: "running".into(),
        error: None,
        expected_cell: None,
        pins_commit: None,
        build_attempts: Vec::new(),
        built: BTreeMap::new(),
        steps: Vec::new(),
        nodes: Vec::new(),
        verification: Value::Null,
    };
    rec.ledger("deploy_start", json!({ "plan": plan_id, "job": job.id }));
    let result = run(cfg, rec, job, &mut record).await;
    record.finished_unix_ms = Some(now_ms());
    record.outcome = if result.is_ok() {
        "succeeded"
    } else {
        "failed"
    }
    .into();
    record.error = result.as_ref().err().map(|e| format!("{e:#}"));

    let touched = record.nodes.iter().any(|n| n.switched);
    if touched {
        rec.disturb("deploy", &record.id);
    }
    // A partial rollout is still the deploy in force on the nodes it reached; run manifests
    // carry per-node system paths, so the record plus those paths stay exact.
    if touched || result.is_ok() {
        rec.set_last_deploy(&record.id);
    }
    if let Err(e) = rec.write_json(&format!("deploys/{}.json", record.id), &record) {
        job.log(format!("writing deploy record failed: {e:#}"));
    }
    rec.ledger(
        "deploy_finished",
        json!({ "plan": record.id, "outcome": record.outcome, "error": record.error }),
    );
    result?;
    Ok(json!({
        "deploy_id": record.id,
        "outcome": record.outcome,
        "record": format!("deploys/{}.json", record.id),
        "pins_commit": record.pins_commit,
        "built": record.built,
        "nodes": record.nodes.iter().map(|n| json!({
            "node": n.node, "ok": n.ok, "elapsed_ms": n.elapsed_ms, "note": n.note,
        })).collect::<Vec<_>>(),
        "verification": record.verification,
    }))
}

async fn run(cfg: &Config, rec: &Recorder, job: &JobCtx, record: &mut DeployRecord) -> Result<()> {
    let order = cfg.rollout_order();
    let names: Vec<String> = order.iter().map(|n| n.name.clone()).collect();

    // Stale-plan check: the pins it was computed against must still be HEAD.
    let t = Instant::now();
    let r = async {
        if names != record.plan.nodes {
            bail!(
                "stale plan: fleet.toml rollout is {names:?}, plan has {:?}",
                record.plan.nodes
            );
        }
        let head = verify_config_repo(cfg).await?;
        if head != record.plan.config_head {
            bail!(
                "stale plan: config repo is at {} but the plan was made at {}; plan again",
                short(&head),
                short(&record.plan.config_head)
            );
        }
        Ok(head)
    }
    .await;
    record.step(
        job,
        "verify-plan",
        t,
        &r,
        json!({ "config_head": record.plan.config_head }),
    );
    r?;

    // Preflight before anything is pushed: every node reachable and on one cell.
    let t = Instant::now();
    let r = preflight(cfg, &order).await;
    let detail = match &r {
        Ok((systems, cell)) => json!({ "systems": systems, "cell": cell }),
        Err(_) => Value::Null,
    };
    record.step(job, "preflight", t, &r, detail);
    let (previous, expected_cell) = r?;
    record.expected_cell = Some(expected_cell.clone());

    if !record.plan.repos.is_empty() || record.plan.minimuas.is_some() {
        let t = Instant::now();
        let r = commit_and_push_pins(cfg, job, &record.plan).await;
        record.step(job, "pins", t, &r, json!({ "commit": r.as_ref().ok() }));
        record.pins_commit = Some(r?);
    } else {
        job.log("no pin changes: building config HEAD as it is");
    }

    let t = Instant::now();
    let mut attempts = Vec::new();
    let r = build_all(cfg, job, &order, &mut attempts).await;
    record.build_attempts = attempts;
    record.step(job, "build", t, &r, json!({ "built": r.as_ref().ok() }));
    record.built = r?;

    let mut stamped = false;
    for (i, node) in order.iter().enumerate() {
        let sys = record.built[&node.name].clone();
        let prev = previous.get(&node.name).cloned();
        if !stamped && prev.as_deref() != Some(sys.as_str()) {
            // Stamp before the first activation so the settle clock (I3) covers a rollout that
            // dies midway, not only one that finishes.
            rec.disturb("deploy", &format!("{}: rollout started", record.id));
            stamped = true;
        }
        let rollout = switch_node(cfg, job, node, &sys, prev).await;
        let failed = (!rollout.ok).then(|| rollout.note.clone());
        record.nodes.push(rollout);
        if let Some(note) = failed {
            bail!(
                "{} failed ({note}); rollout stopped — nodes after it in {:?} are untouched",
                node.name,
                names
            );
        }
        if i == 0 {
            let t = Instant::now();
            let r = cells::wait_forwarder_healthy(cfg, node, CANARY_HEALTH, &|l| job.log(l)).await;
            let detail = r.as_ref().map(|h| json!(h)).unwrap_or(Value::Null);
            record.step(job, "canary-health", t, &r, detail);
            r.with_context(|| {
                format!(
                    "canary {} unhealthy after switch; rollout stopped, other nodes untouched",
                    node.name
                )
            })?;
        }
    }

    let t = Instant::now();
    let r = verify_fleet(cfg, &order, &record.built).await;
    record.step(
        job,
        "verify",
        t,
        &r,
        r.as_ref().cloned().unwrap_or(Value::Null),
    );
    record.verification = json!({ "fleet": r? });

    if record.nodes.iter().any(|n| n.switched) {
        let t = Instant::now();
        let r = cells::restart_roles(cfg, rec, job).await;
        record.step(
            job,
            "restart-roles",
            t,
            &r,
            r.as_ref().cloned().unwrap_or(Value::Null),
        );
        r?;
    } else {
        job.log("no node switched: role restart skipped");
    }

    let t = Instant::now();
    let r = status::health_gate(cfg, rec, &expected_cell, HEALTH_GATE, &|l| job.log(l)).await;
    let detail = r.as_ref().map(|s| json!(s)).unwrap_or(Value::Null);
    record.step(job, "health-gate", t, &r, detail.clone());
    r?;
    record.verification["health"] = detail;
    Ok(())
}

type Systems = BTreeMap<String, String>;

/// Each node's current system, and the one cell they all share (the final health gate needs
/// it; a mixed fleet cannot pass one and is refused before anything is pushed).
async fn preflight(cfg: &Config, order: &[&Node]) -> Result<(Systems, String)> {
    let cmd = "readlink /run/current-system; cat /var/lib/minimuas/fabric/active";
    let outs = status::join_all(order.iter().map(|n| remote::ssh(cfg, n, cmd, QUICK))).await;
    let mut systems = BTreeMap::new();
    let mut cells: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (node, out) in order.iter().zip(outs) {
        let out = out?;
        let text = out
            .stdout_ok()
            .with_context(|| format!("{}: preflight", node.name))?;
        let mut lines = text.lines().map(str::trim);
        let sys = lines.next().unwrap_or_default();
        let cell = lines.next().unwrap_or_default();
        if !sys.starts_with("/nix/store/") || cell.is_empty() {
            bail!("{}: unexpected preflight output: {text:?}", node.name);
        }
        systems.insert(node.name.clone(), sys.to_string());
        cells
            .entry(cell.to_string())
            .or_default()
            .push(node.name.clone());
    }
    if cells.len() != 1 {
        bail!("fleet is on mixed cells {cells:?}; set one cell (fleet_set_cell) before deploying");
    }
    let cell = cells.into_keys().next().expect("one cell");
    Ok((systems, cell))
}

fn commit_message(plan: &Plan) -> String {
    let mut subject = Vec::new();
    if !plan.repos.is_empty() {
        let list: Vec<String> = plan
            .repos
            .iter()
            .map(|c| format!("{} {}", c.name, short(&c.new_rev)))
            .collect();
        subject.push(format!("ndn-fwd: bump pins ({})", list.join(", ")));
    }
    if let Some(m) = &plan.minimuas {
        subject.push(format!("{MINIMUAS_INPUT}: bump to {}", short(&m.new_rev)));
    }
    let mut msg = subject.join("; ");
    msg.push_str("\n\n");
    for c in plan.repos.iter().chain(&plan.minimuas) {
        msg.push_str(&format!(
            "{} {} -> {}\n",
            c.name,
            short(&c.old_rev),
            short(&c.new_rev)
        ));
        for line in c.commits.iter().take(MAX_COMMITS_IN_MESSAGE) {
            msg.push_str(&format!("  {line}\n"));
        }
        if c.commits.len() > MAX_COMMITS_IN_MESSAGE {
            msg.push_str(&format!(
                "  … {} more\n",
                c.commits.len() - MAX_COMMITS_IN_MESSAGE
            ));
        }
        if c.commits.is_empty() {
            msg.push_str("  (commit list unavailable)\n");
        }
        msg.push('\n');
    }
    msg.push_str(&format!("Deployed-by: ndn-fleet {}\n", plan.id));
    msg
}

/// Edit pins (rev + hash together) and the miniMUAS input, commit, push. Any failure before
/// the commit restores the touched files, leaving the config repo as found.
async fn commit_and_push_pins(cfg: &Config, job: &JobCtx, plan: &Plan) -> Result<String> {
    let repo = &cfg.deploy.config_repo;
    let pins_rel = cfg.deploy.pins_file.to_string_lossy().into_owned();
    let mut touched: Vec<String> = Vec::new();
    let edited = edit_config_files(cfg, job, plan, &mut touched).await;
    let committed = match edited {
        Ok(()) => {
            let mut add = vec!["add", "--"];
            add.extend(touched.iter().map(String::as_str));
            let message = commit_message(plan);
            async {
                git_out(repo, &add).await?;
                git_out(repo, &["commit", "-q", "-m", &message]).await?;
                git_out(repo, &["rev-parse", "HEAD"]).await
            }
            .await
        }
        Err(e) => Err(e),
    };
    let sha = match committed {
        Ok(sha) => sha,
        Err(e) => {
            if !touched.is_empty() {
                let mut restore = vec!["checkout", "HEAD", "--"];
                restore.extend(touched.iter().map(String::as_str));
                if let Err(r) = git_out(repo, &restore).await {
                    job.log(format!("restoring {touched:?} failed: {r:#}"));
                }
            }
            return Err(e.context(format!("editing/committing pins ({pins_rel})")));
        }
    };
    job.log(format!("pins commit {sha}"));
    let target = format!("HEAD:refs/heads/{}", cfg.deploy.config_branch);
    git(repo, &["push", "-q", "origin", &target])
        .await?
        .stdout_ok()
        .with_context(|| {
            format!(
                "pins commit {} exists locally but the push failed; nothing was built or deployed",
                short(&sha)
            )
        })?;
    job.log(format!(
        "pushed {} to origin/{}",
        short(&sha),
        cfg.deploy.config_branch
    ));
    Ok(sha)
}

async fn edit_config_files(
    cfg: &Config,
    job: &JobCtx,
    plan: &Plan,
    touched: &mut Vec<String>,
) -> Result<()> {
    let repo = &cfg.deploy.config_repo;
    if !plan.repos.is_empty() {
        let path = repo.join(&cfg.deploy.pins_file);
        let mut text = std::fs::read_to_string(&path)?;
        for c in &plan.repos {
            let current = read_pin(&text, &c.name)?;
            if current.rev != c.old_rev {
                bail!(
                    "stale plan: {} is pinned at {}, plan expected {}",
                    c.name,
                    short(&current.rev),
                    short(&c.old_rev)
                );
            }
            text = edit_pin(&text, &c.name, &c.new_rev, &c.new_hash)?;
            job.log(format!(
                "pin {}: {} -> {} ({})",
                c.name,
                short(&c.old_rev),
                short(&c.new_rev),
                c.new_hash
            ));
        }
        touched.push(cfg.deploy.pins_file.to_string_lossy().into_owned());
        std::fs::write(&path, text)?;
    }
    if let Some(m) = &plan.minimuas {
        let path = repo.join("flake.nix");
        let text = std::fs::read_to_string(&path)?;
        let current = read_flake_input(&text, MINIMUAS_INPUT)?;
        if current.rev != m.old_rev {
            bail!(
                "stale plan: {MINIMUAS_INPUT} is at {}, plan expected {}",
                short(&current.rev),
                short(&m.old_rev)
            );
        }
        touched.push("flake.nix".into());
        touched.push("flake.lock".into());
        std::fs::write(
            &path,
            edit_flake_input_rev(&text, MINIMUAS_INPUT, &m.new_rev)?,
        )?;
        remote::local(
            "nix",
            &["flake", "lock", "--update-input", MINIMUAS_INPUT],
            Some(repo),
            &[],
            PREFETCH,
        )
        .await?
        .stdout_ok()?;
        let lock: Value = serde_json::from_str(&std::fs::read_to_string(repo.join("flake.lock"))?)?;
        let locked = &lock["nodes"][MINIMUAS_INPUT]["locked"];
        if locked["rev"].as_str() != Some(&m.new_rev)
            || locked["narHash"].as_str() != Some(&m.new_hash)
        {
            bail!(
                "flake.lock {MINIMUAS_INPUT} locked {} / {}, plan expected {} / {}",
                locked["rev"],
                locked["narHash"],
                m.new_rev,
                m.new_hash
            );
        }
        job.log(format!(
            "{MINIMUAS_INPUT}: {} -> {} ({})",
            short(&m.old_rev),
            short(&m.new_rev),
            m.new_hash
        ));
    }
    Ok(())
}

/// nixbuild.net copy-back failures that a retry clears (fleet record).
fn transient_build_failure(stderr: &str) -> bool {
    ["unexpected end-of-file", "Connection reset", "EOF"]
        .iter()
        .any(|p| stderr.contains(p))
}

/// Out-paths of one `nix build` of every node, in argument order.
fn map_out_paths(order: &[&Node], stdout: &str) -> Result<Systems> {
    let paths: Vec<&str> = stdout
        .lines()
        .map(str::trim)
        .filter(|l| l.starts_with("/nix/store/"))
        .collect();
    if paths.len() != order.len() {
        bail!(
            "nix build printed {} out-paths for {} nodes: {paths:?}",
            paths.len(),
            order.len()
        );
    }
    let mut built = BTreeMap::new();
    for (node, path) in order.iter().zip(paths) {
        // A NixOS toplevel is named nixos-system-<hostname>-<version>; a path naming another
        // node means the order assumption broke, and deploying it would swap identities.
        if let Some(other) = order
            .iter()
            .find(|o| o.name != node.name && path.contains(&format!("-nixos-system-{}-", o.name)))
        {
            bail!(
                "out-path {path} for {} names {}; refusing to map builds by order",
                node.name,
                other.name
            );
        }
        built.insert(node.name.clone(), path.to_string());
    }
    Ok(built)
}

/// Build every node's toplevel in ONE `nix build`, retrying transient copy-back failures.
async fn build_all(
    cfg: &Config,
    job: &JobCtx,
    order: &[&Node],
    attempts: &mut Vec<Value>,
) -> Result<Systems> {
    let flake = cfg.deploy.config_repo.display().to_string();
    let installables: Vec<String> = order
        .iter()
        .map(|n| {
            format!(
                "{flake}#{}",
                cfg.deploy.flake_attr.replace("{node}", &n.name)
            )
        })
        .collect();
    let mut args = vec!["build", "--no-link", "--print-out-paths"];
    args.extend(installables.iter().map(String::as_str));
    let total = cfg.deploy.build_retries + 1;
    for attempt in 1..=total {
        job.log(format!(
            "build attempt {attempt}/{total}: nix {}",
            args.join(" ")
        ));
        let out = remote::local("nix", &args, Some(&cfg.deploy.config_repo), &[], BUILD).await?;
        attempts.push(json!({
            "attempt": attempt,
            "status": out.status,
            "elapsed_ms": out.elapsed_ms,
            "stderr_tail": tail(&out.stderr, 8000),
        }));
        if out.ok() {
            let built = map_out_paths(order, &out.stdout)?;
            for (node, path) in &built {
                job.log(format!("built {node}: {path}"));
            }
            return Ok(built);
        }
        let retry = attempt < total && transient_build_failure(&out.stderr);
        job.log(format!(
            "build failed ({}){}",
            out.status
                .map_or_else(|| "timed out".into(), |s| format!("exit {s}")),
            if retry { "; transient, retrying" } else { "" }
        ));
        if !retry {
            return Err(out.stdout_ok().unwrap_err().context("building the fleet"));
        }
    }
    unreachable!("the last attempt returns")
}

async fn current_system(cfg: &Config, node: &Node) -> Option<String> {
    let out = remote::ssh(cfg, node, "readlink /run/current-system", QUICK)
        .await
        .ok()?;
    out.ok().then(|| out.stdout.trim().to_string())
}

/// Copy the closure, falling back through the jump host (node 01's campus DNS has been dead).
async fn copy_closure(cfg: &Config, job: &JobCtx, node: &Node, sys: &str) -> Result<Vec<Value>> {
    let user = &cfg.fleet.ssh_user;
    let direct_to = format!("ssh://{user}@{}", node.host);
    let direct = remote::local("nix", &["copy", "--to", &direct_to, sys], None, &[], COPY).await?;
    let mut outs = vec![out_json(&direct)];
    if direct.ok() {
        return Ok(outs);
    }
    job.log(format!(
        "{}: direct copy failed ({}); retrying through {}",
        node.name,
        direct.stderr.trim().lines().last().unwrap_or(""),
        cfg.fleet.jump_host
    ));
    let opts = format!("-J {user}@{} -o ConnectTimeout=10", cfg.fleet.jump_host);
    let jump_to = format!("ssh://{user}@{}", node.addr);
    let jumped = remote::local(
        "nix",
        &["copy", "--to", &jump_to, sys],
        None,
        &[("NIX_SSHOPTS", &opts)],
        COPY,
    )
    .await?;
    outs.push(out_json(&jumped));
    jumped.stdout_ok()?;
    Ok(outs)
}

async fn switch_node(
    cfg: &Config,
    job: &JobCtx,
    node: &Node,
    sys: &str,
    previous: Option<String>,
) -> NodeRollout {
    let t = Instant::now();
    let mut r = NodeRollout {
        node: node.name.clone(),
        system: sys.to_string(),
        previous_system: previous.clone(),
        switched: false,
        copy: Vec::new(),
        switch: None,
        current_system: None,
        ok: false,
        elapsed_ms: 0,
        note: String::new(),
    };
    if previous.as_deref() == Some(sys) {
        job.log(format!("{}: already runs {sys}; not switched", node.name));
        r.current_system = previous;
        r.ok = true;
        r.note = "already running this system".into();
        return r;
    }
    match switch_node_inner(cfg, job, node, sys, &mut r).await {
        Ok(note) => {
            r.ok = true;
            r.note = note;
        }
        Err(e) => r.note = format!("{e:#}"),
    }
    r.elapsed_ms = t.elapsed().as_millis() as u64;
    job.log(format!(
        "{}: {} in {} ms — {}",
        node.name,
        if r.ok { "switched" } else { "FAILED" },
        r.elapsed_ms,
        r.note
    ));
    r
}

async fn switch_node_inner(
    cfg: &Config,
    job: &JobCtx,
    node: &Node,
    sys: &str,
    r: &mut NodeRollout,
) -> Result<String> {
    job.log(format!("{}: copying {sys}", node.name));
    let copied = copy_closure(cfg, job, node, sys).await;
    match copied {
        Ok(outs) => r.copy = outs,
        Err(e) => return Err(e.context("nix copy")),
    }

    // Activation runs under systemd-run (as nixos-rebuild does) so an ssh drop mid-activation
    // cannot kill it halfway; the fleet record also has switches hanging the ssh session after
    // activation succeeded, so the verdict comes from /run/current-system, not the session.
    let q = sh_quote(sys);
    let unit = format!("ndn-fleet-switch-{}", now_ms());
    let cmd = format!(
        "sudo -n nix-env -p /nix/var/nix/profiles/system --set {q} && \
         sudo -n systemd-run --unit={unit} --collect --no-ask-password --pipe --quiet \
           --service-type=exec --wait {q}/bin/switch-to-configuration switch"
    );
    job.log(format!("{}: switching", node.name));
    r.switched = true;
    let out = remote::ssh(cfg, node, &cmd, SWITCH).await?;
    r.switch = Some(out_json(&out));
    let session_lost = matches!(out.status, None | Some(SSH_CONNECT_FAILURE));

    let deadline = Instant::now()
        + if session_lost {
            SETTLE_AFTER_DROP
        } else {
            Duration::ZERO
        };
    let current = loop {
        let current = current_system(cfg, node).await;
        if current.as_deref() == Some(sys) || Instant::now() >= deadline {
            break current;
        }
        tokio::time::sleep(Duration::from_secs(10)).await;
    };
    r.current_system = current.clone();
    if current.as_deref() != Some(sys) {
        bail!(
            "/run/current-system is {} after switch ({}), expected {sys}",
            current.as_deref().unwrap_or("unreadable"),
            out.status
                .map_or_else(|| "timed out".into(), |s| format!("exit {s}"))
        );
    }
    if out.ok() {
        Ok("switched; /run/current-system verified".into())
    } else if session_lost {
        Ok("ssh session ended before switch returned; /run/current-system verified".into())
    } else {
        // switch-to-configuration exits non-zero when units failed to (re)start.
        bail!(
            "switch-to-configuration exited {:?} (units failed?): {}",
            out.status,
            tail(out.stderr.trim(), 1000)
        )
    }
}

/// I4 after rollout: every node runs the closure built for it, and one ndn-fwd package path.
async fn verify_fleet(cfg: &Config, order: &[&Node], built: &Systems) -> Result<Value> {
    let cmd = "readlink /run/current-system; \
               systemctl show -p ExecStart --value muas-fabric-ndn-fwd.service \
               | grep -oE '/nix/store/[a-z0-9]+-ndn-fwd[^ /]*' | head -n1";
    let outs = status::join_all(order.iter().map(|n| remote::ssh(cfg, n, cmd, QUICK))).await;
    let mut per_node = BTreeMap::new();
    let mut problems = Vec::new();
    for (node, out) in order.iter().zip(outs) {
        let out = out?;
        let text = out
            .stdout_ok()
            .with_context(|| format!("{}: verifying", node.name))?;
        let mut lines = text.lines().map(str::trim);
        let sys = lines.next().unwrap_or_default().to_string();
        let pkg = lines.next().unwrap_or_default().to_string();
        if Some(&sys) != built.get(&node.name) {
            problems.push(format!(
                "{} runs {sys}, built {:?}",
                node.name,
                built.get(&node.name)
            ));
        }
        if pkg.is_empty() {
            problems.push(format!(
                "{}: no ndn-fwd package in muas-fabric-ndn-fwd",
                node.name
            ));
        }
        per_node.insert(node.name.clone(), json!({ "system": sys, "ndn_fwd": pkg }));
    }
    let pkgs: std::collections::BTreeSet<&str> = per_node
        .values()
        .filter_map(|v| v["ndn_fwd"].as_str())
        .collect();
    if pkgs.len() > 1 {
        problems.push(format!("nodes run different ndn-fwd packages: {pkgs:?}"));
    }
    if !problems.is_empty() {
        bail!("fleet verification failed: {}", problems.join("; "));
    }
    Ok(json!({ "nodes": per_node, "ndn_fwd": pkgs.into_iter().next() }))
}

#[cfg(test)]
mod tests {
    use super::*;

    const REAL_PINS: &str = include_str!("../testdata/ndn-fwd-pins.nix");
    const REV: &str = "2a5840be79098017965517e6c975874602d7f22a";
    const HASH: &str = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=";

    #[test]
    fn edit_pin_changes_only_the_named_blocks_rev_and_hash() {
        let before = read_pin(REAL_PINS, "ndn-radio").unwrap();
        let drivers = read_pin(REAL_PINS, "ndn-radio-drivers").unwrap();
        let edited = edit_pin(REAL_PINS, "ndn-radio", REV, HASH).unwrap();

        let changed: Vec<(&str, &str)> = REAL_PINS
            .lines()
            .zip(edited.lines())
            .filter(|(a, b)| a != b)
            .collect();
        assert_eq!(
            changed.len(),
            2,
            "exactly the rev and hash lines: {changed:?}"
        );
        assert_eq!(REAL_PINS.lines().count(), edited.lines().count());
        assert!(changed[0].0.contains(&before.rev) && changed[0].1.contains(REV));
        assert!(changed[1].0.contains(&before.hash) && changed[1].1.contains(HASH));

        let after = read_pin(&edited, "ndn-radio").unwrap();
        assert_eq!((after.rev.as_str(), after.hash.as_str()), (REV, HASH));
        // The prefix-sharing sibling is untouched.
        assert_eq!(read_pin(&edited, "ndn-radio-drivers").unwrap(), drivers);
    }

    #[test]
    fn edit_pin_skips_comment_lines_inside_a_block() {
        // The ndn-rs block carries a long comment history above its rev line.
        let before = read_pin(REAL_PINS, "ndn-rs").unwrap();
        assert_eq!(before.rev.len(), 40);
        let edited = edit_pin(REAL_PINS, "ndn-rs", REV, HASH).unwrap();
        let comments = |t: &str| {
            t.lines()
                .filter(|l| is_comment(l))
                .collect::<Vec<_>>()
                .join("\n")
        };
        assert_eq!(comments(REAL_PINS), comments(&edited));
        assert_eq!(read_pin(&edited, "ndn-rs").unwrap().rev, REV);
        for other in pin_names(REAL_PINS).iter().filter(|n| *n != "ndn-rs") {
            assert_eq!(
                read_pin(&edited, other).unwrap(),
                read_pin(REAL_PINS, other).unwrap()
            );
        }
    }

    #[test]
    fn edit_pin_refuses_unknown_blocks_and_partial_pins() {
        assert!(edit_pin(REAL_PINS, "ndn-sim", REV, HASH).is_err());
        assert!(
            edit_pin(REAL_PINS, "ndn-rs", "2a5840be", HASH).is_err(),
            "short rev"
        );
        assert!(
            edit_pin(REAL_PINS, "ndn-rs", REV, "").is_err(),
            "rev without hash"
        );
    }

    #[test]
    fn minimuas_input_rev_is_edited_inside_the_url_only() {
        let flake = "  # To bump: change the rev\n  inputs.minimuas-src.url = \"git+ssh://git@github.com/JacobsSensorLab/miniMUAS?ref=refs/heads/video-stream-prototype&rev=aba87c6ad25a81a96803c171e007c02b91b18cd9\";\n  inputs.minimuas-src.flake = false;\n";
        let pin = read_flake_input(flake, MINIMUAS_INPUT).unwrap();
        assert_eq!(pin.rev, "aba87c6ad25a81a96803c171e007c02b91b18cd9");
        assert_eq!(pin.branch.as_deref(), Some("video-stream-prototype"));
        let edited = edit_flake_input_rev(flake, MINIMUAS_INPUT, REV).unwrap();
        assert_eq!(
            edited,
            flake.replace("aba87c6ad25a81a96803c171e007c02b91b18cd9", REV)
        );
    }

    /// A throwaway repo with a bare `origin`, one pushed commit, upstream set.
    fn scratch_repo(tag: &str) -> std::path::PathBuf {
        let root = std::env::temp_dir().join(format!(
            "ndn-fleet-deploy-{tag}-{}-{}",
            std::process::id(),
            now_ms()
        ));
        let origin = root.join("origin.git");
        let work = root.join("work");
        std::fs::create_dir_all(&root).unwrap();
        let run = |dir: &Path, args: &[&str]| {
            let out = std::process::Command::new("git")
                .args([
                    "-c",
                    "user.name=t",
                    "-c",
                    "user.email=t@t",
                    "-c",
                    "commit.gpgsign=false",
                ])
                .args(args)
                .current_dir(dir)
                .output()
                .unwrap();
            assert!(
                out.status.success(),
                "git {args:?}: {}",
                String::from_utf8_lossy(&out.stderr)
            );
        };
        run(&root, &["init", "-q", "--bare", "-b", "main", "origin.git"]);
        run(&root, &["clone", "-q", origin.to_str().unwrap(), "work"]);
        std::fs::write(work.join("a.txt"), "one\n").unwrap();
        run(&work, &["add", "a.txt"]);
        run(&work, &["commit", "-q", "-m", "one"]);
        run(&work, &["push", "-q", "-u", "origin", "HEAD:main"]);
        work
    }

    fn git_sync(dir: &Path, args: &[&str]) -> String {
        let out = std::process::Command::new("git")
            .args([
                "-c",
                "user.name=t",
                "-c",
                "user.email=t@t",
                "-c",
                "commit.gpgsign=false",
            ])
            .args(args)
            .current_dir(dir)
            .output()
            .unwrap();
        assert!(out.status.success());
        String::from_utf8_lossy(&out.stdout).trim().to_string()
    }

    #[tokio::test]
    async fn plan_refuses_dirty_or_unpushed_changes_and_only_warns_for_unchanged_dirt() {
        let work = scratch_repo("dirty");
        let head = git_sync(&work, &["rev-parse", "HEAD"]);
        let old = "1111111111111111111111111111111111111111";
        let src = || Source::GitHub {
            owner: "o",
            repo: "r",
        };
        std::fs::write(work.join("a.txt"), "edited\n").unwrap();

        // HEAD == pin: nothing ships, so dirt is a warning.
        let mut warnings = Vec::new();
        let (rev, change) = plan_repo("r", &work, None, &head, None, src(), &mut warnings)
            .await
            .unwrap();
        assert_eq!(rev, head);
        assert!(change.is_none());
        assert!(
            warnings.iter().any(|w| w.contains("NOT deployed")),
            "{warnings:?}"
        );

        // HEAD != pin with dirt: refused before any prefetch.
        let err = plan_repo("r", &work, None, old, None, src(), &mut Vec::new())
            .await
            .unwrap_err();
        assert!(format!("{err:#}").contains("uncommitted"), "{err:#}");

        // Committed but not pushed: refused.
        git_sync(&work, &["commit", "-q", "-am", "two"]);
        let err = plan_repo("r", &work, None, old, None, src(), &mut Vec::new())
            .await
            .unwrap_err();
        assert!(format!("{err:#}").contains("not on origin/main"), "{err:#}");

        std::fs::remove_dir_all(work.parent().unwrap()).ok();
    }
}
