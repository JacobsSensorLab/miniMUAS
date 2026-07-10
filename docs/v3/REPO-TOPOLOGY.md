# UAS stack repo topology

Decided 2026-07-09: multi-repo with decoupled layers, from the deployment
configs down to the libraries, so new use cases and new big capabilities
(e.g. autonomous navigation) land without growing pains. miniMUAS becomes
*one app* on a use-case-agnostic stack instead of the stack itself.

## The layers

```
deployment   minidronesys-configurations (branch per system)
                │  composes per-repo flakes; owns host config ONLY
apps         miniMUAS (this repo)         [future: other use-case apps]
                │  mission choreography, app contracts, composed binaries
operator     uas-console                  manifest-driven console framework
                │
fleet        uas-fleet                    NDN fleet plane: data kinds, node
                │                         framework, coordination, RC
vehicle      uas-flight                   flight primitives + MAVLink adapter
                │
frameworks   ndn-workspace · ndf-rs · flotilla   (upstream, separate owners)
```

## Repos and their contracts

| Repo | Owns | Must NOT contain |
|---|---|---|
| **uas-flight** (`~/Documents/Dev/uas-flight`) | `uas-flight` (primitives, deconflict, patterns, placement, constraints — the UAS-IPBRC extraction), `uas-mavlink` (vehicle adapter) | NDN/NDF anything; mission logic; app names |
| **uas-fleet** | `uas-fleet-data` (generic kinds + L1 manifests: telemetry, coord, capability, journal chains, artifacts), `uas-fleet-node` (agent framework: service hosting over pluggable carriers, flight-backend seam, PeerGuard loop, bearers, journals, shutdown), `uas-rc` (1-to-many RC over Sparks) | Use-case choreography (raster→detect→investigate is miniMUAS's); UI |
| **uas-console** | `uas-console` (render-contract-matched view framework, generic fleet intents: track/video/event-log/replay; waterline instrument presentation) | App panels; vehicle-side code |
| **miniMUAS** | `muas-contracts` (mission names/kinds/services), `muas-agent` + `muas-dashboard` (thin binaries composing the stack), `muas-sim` (mission scenarios), docs, field doctrine | Anything a second use case would want to reuse — if it's generic, it moves down a layer |
| **minidronesys-configurations** | Host/system config, fleet composition, image building | Application packaging knowledge (see below) |

## Dependency rules

1. **Downward only**, no skips prohibited (an app may use uas-flight
   directly), no cycles ever.
2. **Generic-by-default test**: before adding code to miniMUAS, ask "would a
   relay-chain app or a survey app want this?" If yes, it goes in the layer
   repo. This is the growing-pain firewall.
3. App names (`/muas/...`) appear only in app repos. Layer repos take the
   app prefix as a parameter.
4. New capability (autonomous nav, trajectory planning) = new crate/module in
   **uas-flight** behind the `FlightPrimitive`/planner seam. New use case =
   new **app repo**. Neither touches the other layers.
5. Layer repos version independently (semver once published; rev pins until
   then). Breaking a layer's API requires bumping every consumer *by rev*,
   which is visible in the deployment lockfile — no silent lockstep.

## Dev linkage vs deploy linkage

- **Dev**: relative path deps between sibling checkouts under
  `~/Documents/Dev/` (`uas-flight = { path = "../uas-flight/..." }`) — the
  same convention ndn-workspace and the ndf refounding already use. One
  clone layout, everything editable at once.
- **Deploy**: the config repo pins each repo **by rev** as a flake input
  (exactly today's `minimuas-src`/`relay` pattern, one input per repo).
  ndn-workspace ships `convert_git_deps.py` / `git_to_pathdeps.py` for
  flipping path↔git deps; we adopt the same approach when publishing.

## Deployment decoupling (the fix for today's inversion)

Today `minidronesys-configurations-copy` owns miniMUAS's packaging
(`nix/ndn-packages/minimuas/*.nix` + patches like `config_dir.patch` against
minimuas internals) — the deployment layer knows application internals.
v3 inverts this:

- **Each source repo ships its own flake** exposing `packages.<system>.*`
  (and a devShell). The Rust repos share one packaging idiom
  (`rustPlatform.buildRustPackage` + vendored deps, or crane) defined once
  and copied per repo.
- **The config repo (v3 branch, when created) composes**: takes each repo's
  flake as an input pinned by rev, wires binaries into systemd units/host
  config, and owns *only* system concerns (hostnames, interfaces, radios,
  autostart). No application patches — if a package needs a knob, the knob
  becomes a proper config option in the app repo.
- Bump flow stays the memorized one, per repo: push repo → bump rev in
  config repo → `nix flake lock --update-input <repo>`.

Flakes land in each repo at the deployment milestone (M-deploy, after M3);
until then dev is pure cargo + path deps.

## Naming

`uas-flight` / `uas-fleet` / `uas-console` are functional working names,
consistent with UAS-IPBRC. Nothing is published or remoted yet, so renaming
is a `mv` + sed over path deps — cheap until first push. Veto window closes
when the config repo starts pinning them.

## Migration status (2026-07-09)

- Done: crates `muas-flight`→`uas-flight/crates/uas-flight`,
  `muas-mavlink`→`uas-flight/crates/uas-mavlink`,
  `muas-rc`→`uas-fleet/crates/uas-rc`; new `uas-fleet-data`,
  `uas-fleet-node`, `uas-console` skeletons; miniMUAS workspace rewired to
  sibling path deps. All four workspaces build and test green.
- Pending: git remotes for the three new repos (JacobsSensorLab org);
  per-repo flakes (M-deploy); config-repo v3 branch.
