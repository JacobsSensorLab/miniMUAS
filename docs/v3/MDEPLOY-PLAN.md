# M-deploy: v3 config-repo composition plan (minidronesys-configurations)

Status: repo-side flakes exist in uas-flight, uas-fleet, uas-console (M-deploy).
This is the exact composition the config repo (branch `mini-muas-v2` → v3) needs.
v3 inversion: repos own packaging and export packages; the config repo only
pins each repo's flake by rev and wires systemd units around the exported bins
(vs. today's source-only `flake = false` inputs + in-config derivations).

## 1. Inputs table

The stack repos take their *sibling sources* as `flake = false` inputs
(dev default: `git+file:///Users/pmle/Documents/Dev/...` — tracked files of the
local checkout). The config repo owns each pin ONCE and fans it out with
`follows`, so uas-fleet and uas-console are guaranteed to build against the
same sibling revs.

```nix
inputs = {
  nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";          # host OS pin (companions)
  home-manager.url = "github:nix-community/home-manager/release-25.05";
  home-manager.inputs.nixpkgs.follows = "nixpkgs";

  # ── package-exporting stack flakes (v3), pinned by rev ──
  uas-flight.url  = "git+ssh://git@github.com/JacobsSensorLab/uas-flight?rev=<REV>";
  uas-fleet.url   = "git+ssh://git@github.com/JacobsSensorLab/uas-fleet?rev=<REV>";
  uas-console.url = "git+ssh://git@github.com/JacobsSensorLab/uas-console?rev=<REV>";

  # ── source-only pins for the cross-repo path deps (flake = false) ──
  # (hosting locations for flotilla / ndn-rs / ndn-ext / ndf-rs: whatever the
  #  org publishes them as; they are separate git repos nested under the
  #  ~/Documents/Dev/ndn-workspace umbrella on dev machines)
  uas-flight-src = { url = "git+ssh://git@github.com/JacobsSensorLab/uas-flight?rev=<REV>"; flake = false; };
  uas-fleet-src  = { url = "git+ssh://git@github.com/JacobsSensorLab/uas-fleet?rev=<REV>";  flake = false; };
  flotilla-src   = { url = "git+ssh://git@github.com/<org>/flotilla?rev=<REV>";  flake = false; };
  ndn-rs-src     = { url = "git+ssh://git@github.com/<org>/ndn-rs?rev=<REV>";    flake = false; };
  ndn-ext-src    = { url = "git+ssh://git@github.com/<org>/ndn-ext?rev=<REV>";   flake = false; };
  ndf-rs-src     = { url = "git+ssh://git@github.com/<org>/ndf-rs?rev=<REV>";    flake = false; };

  # ── follows-overrides: point every repo-flake sibling input at the pins ──
  # uas-flight has no sibling inputs (self-contained).
  uas-fleet.inputs.uas-flight-src.follows = "uas-flight-src";
  uas-fleet.inputs.flotilla-src.follows   = "flotilla-src";
  uas-fleet.inputs.ndn-rs-src.follows     = "ndn-rs-src";
  uas-fleet.inputs.ndn-ext-src.follows    = "ndn-ext-src";
  uas-fleet.inputs.ndf-rs-src.follows     = "ndf-rs-src";

  uas-console.inputs.uas-fleet-src.follows  = "uas-fleet-src";
  uas-console.inputs.uas-flight-src.follows = "uas-flight-src";
  uas-console.inputs.flotilla-src.follows   = "flotilla-src";
  uas-console.inputs.ndn-rs-src.follows     = "ndn-rs-src";
  uas-console.inputs.ndn-ext-src.follows    = "ndn-ext-src";
  uas-console.inputs.ndf-rs-src.follows     = "ndf-rs-src";

  # Optional dedup: also pin the stack flakes' nixpkgs, e.g.
  #   uas-fleet.inputs.nixpkgs.follows = "nixpkgs-unstable";
  # CAUTION: the stack flakes track nixos-unstable because flotilla's MSRV is
  # rust 1.90 (nixos-25.05 rustc is too old). Do NOT follow them onto the
  # 25.05 host pin; either leave their own locks alone (fine — packages are
  # closures, host and package nixpkgs need not match) or add a separate
  # `nixpkgs-unstable` input and follow onto that.
};
```

Notes:
- IMPORTANT: the repo flakes' committed dev defaults are `git+file:///Users/pmle/...`
  URLs. The config repo MUST override every `*-src` input (table above) or
  evaluation fails on any other machine. `nix flake lock` in the config repo
  re-locks the whole tree, so the repo-side dev locks never leak in.
- `follows` (option a) is preferred over direct nested URL overrides
  (`inputs.uas-fleet.inputs.flotilla-src.url = "...?rev=..."`, option b)
  because fleet and console share five of the six sources — one pin each.
- uas-flight and uas-fleet appear twice (as a flake for its packages, and as
  `-src` for the dependents' stitched build trees). Keep both at the SAME rev
  when bumping.

## 2. What each repo flake exports (v3 contract)

| repo        | packages                                    | devShells | checks |
|-------------|---------------------------------------------|-----------|--------|
| uas-flight  | `checkride` (= `sitl-checkride` bin), `default` | `default` | — |
| uas-fleet   | `rc-bench` (rc_bench example, `--features spark`), `default` | `default` | `workspace` (full workspace compile in stitched tree) |
| uas-console | — (library/framework, no bins)              | `default` | `build` (cargo build + test in stitched tree) |

Systems: `aarch64-linux` (deploy), `x86_64-linux`, `aarch64-darwin` (dev).
All three repos build their Rust off nixos-unstable (rustc 1.95) inside the
flake; the companion host stays on nixos-25.05.

## 3. systemd unit sketches (companion hosts, aarch64-linux)

The muas-agent binary is the vehicle-side agent. Until uas-fleet grows a
first-class agent bin, the deployable artifacts are `sitl-checkride`
(uas-flight) and `rc-bench` (uas-fleet); the sketches below are written
against the eventual `packages.muas-agent` name — substitute what exists.

```nix
# nix/nixos/<hostname>/fleet.nix
{ flakeInputs, pkgs, ... }: {
  systemd.services.muas-agent = {
    description = "miniMUAS vehicle agent (uas-fleet)";
    wantedBy = [ "multi-user.target" ];
    after = [ "network-online.target" ];   # + the NDN forwarder unit when it exists
    wants = [ "network-online.target" ];
    serviceConfig = {
      ExecStart = "${flakeInputs.uas-fleet.packages.aarch64-linux.rc-bench}/bin/rc-bench";
      Restart = "on-failure";
      RestartSec = 2;
      # journals must survive power loss (mission replay): keep state on disk
      StateDirectory = "muas-agent";
      WorkingDirectory = "/var/lib/muas-agent";
      # serial access to the flight controller
      SupplementaryGroups = [ "dialout" ];
      # hardening floor
      NoNewPrivileges = true;
      ProtectSystem = "strict";
      ReadWritePaths = [ "/var/lib/muas-agent" ];
    };
    environment = {
      RUST_LOG = "info";
      # MAVLink endpoint to the FC (matches the ArduPilot serial/udp bridge)
      MUAS_MAVLINK_URL = "serialtcp/udp endpoint per airframe";
    };
  };

  # console side (ground station box), same pattern:
  systemd.services.muas-dashboard = {
    description = "miniMUAS fleet dashboard (uas-console)";
    wantedBy = [ "multi-user.target" ];
    after = [ "network-online.target" ];
    serviceConfig = {
      # uas-console exports no bin yet; when packages.dashboard lands:
      ExecStart = "${flakeInputs.uas-console.packages.aarch64-linux.dashboard}/bin/muas-dashboard";
      Restart = "on-failure";
      DynamicUser = true;
      StateDirectory = "muas-dashboard";
    };
  };
}
```

Wiring: the config flake already passes `flakeInputs` via `specialArgs`
(see current mini-muas-v2 flake.nix); v3 changes only what is consumed —
`flakeInputs.uas-fleet.packages.aarch64-linux.<pkg>` instead of building from
`flakeInputs.relay` source in-config.

## 4. Per-repo bump flow

For a change in repo X (say uas-flight):

1. In X: commit + push (packaging = X's own flake; `nix build` green in X).
2. In the config repo:
   - bump `?rev=` on `inputs.uas-flight` AND `inputs.uas-flight-src`
     (same rev, both places — flake consumption and stitched-source
     consumption must agree),
   - `nix flake lock --update-input uas-flight --update-input uas-flight-src`,
   - if the change affects dependents' Cargo.locks (new/updated registry
     deps), those repos must have committed their updated Cargo.lock first —
     the stitched builds vendor from each repo's OWN lockfile.
3. `nix build .#nixosConfigurations.<host>.config.system.build.toplevel`
   (or deploy tooling) → switch.

Sibling-source bumps (flotilla / ndn-rs / ndn-ext / ndf-rs) are step 2 only,
against their `-src` inputs; uas-fleet/uas-console rebuild automatically since
their stitched trees change.

Cargo.lock discipline: uas-fleet's and uas-console's Cargo.locks pin the
registry deps for the WHOLE stitched graph (path deps carry no `source` and
are compiled from the stitched tree). If a sibling bump adds a new crates.io
dep, the dependent repo must `cargo update -p <crate>`/`cargo build` and
commit its refreshed Cargo.lock before the config-repo bump will build.

## 5. Repo-side facts the config repo relies on

- Stitched layout inside fleet/console derivations mirrors ~/Documents/Dev:
  `{uas-flight, uas-fleet, uas-console, ndn-workspace/{flotilla,ndn-rs,ndn-ext}, ndf-rs}`
  — because Cargo path deps hop between them (ndf-core → ../../../ndn-workspace/ndn-rs/...,
  ndf-spark → ndn-ext's ndn-surface).
- Registry deps are vendored via `rustPlatform.importCargoLock`
  (`cargoLock.lockFile`); no git deps exist in any lockfile today, so no
  `outputHashes` needed.
- Linux builds need `pkg-config` + `udev` (serialport via mavlink
  direct-serial); already in the repo flakes' derivations.
- Repo-side `flake.lock`s: uas-flight has one (nixpkgs only). fleet/console
  cannot write theirs while sibling checkouts are git-dirty (nix refuses
  unlocked-input locks); harmless — the config repo's lock is the one that
  matters, and it re-locks everything through the follows-overrides.
