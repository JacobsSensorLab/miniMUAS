{ lib
, fetchFromGitHub
, runCommand
, makeRustPlatform
, rust-bin        # from rust-overlay (applied in default-system.nix overlays)
, pkg-config
, libusb1
}:

# ndn-fwd (Rust NDN forwarder, ndn-workspace) + ndn-tools, with the
# radio-libusb feature (RTL8812EU/8822E userspace named-data radio,
# USB 0bda:a81a). Powers the muas-fabric ndn-fwd/radio cells.
#
# SHA set = the mutually consistent revs from the ndn-workspace clean-house
# consolidation (all repos on main, committed + green + pushed). Includes the
# AfPacketBackend::inject TxIntent fix (worst-receiver rate) + the parse-everywhere
# MAC restructure that split the named-radio stack into the new ndn-radio repo.
#
# The named-radio stack was RESTRUCTURED: a new sibling repo `ndn-radio` (MAC +
# PHY faces, formerly ndn-face-monitor-wifi -> ndn-phy-wifi) now sits between
# ndn-radio-drivers and ndn-ext/ndn-fwd. Fetch set is now
#   {ndn-fwd, ndn-rs, ndn-ext, ndn-radio, ndn-radio-drivers}
# plus ndn-embedded (ndn-radio-drivers' firmware crates path-dep it). ndn-sim
# dropped out — nothing in the closure path-deps it any more.
let
  rust = rust-bin.stable."1.96.0".default; # ndn-fwd rust-toolchain.toml pin
  rustPlatform = makeRustPlatform { cargo = rust; rustc = rust; };

  srcs = {
    ndn-fwd = fetchFromGitHub {
      owner = "Quarmire"; repo = "ndn-fwd";
      rev = "86d6bc471bc0b2e45be9fe47b4bc1d516f10388c";
      hash = "sha256-9w8ZiINzpA5HimTaJN2AnvYvDCO0PhJSSc/iIubVt54=";
    };
    ndn-rs = fetchFromGitHub {
      owner = "Quarmire"; repo = "ndn-rs";
      # Branch fwd-config-strategy-choices (contains fleet-transport-fixes as
      # an ancestor): the validated pin a070c93e plus the
      # forwarding/cache fixes found on this fleet, and deliberately NOT ndn-rs
      # main (which also carries the unvalidated RetxSuppressionExponential
      # port). In order:
      #   - oldest-first retransmission, fast retransmit on ack ordering
      #     (both validated at mechanism level, inert at this link's loss rate)
      #   - CS hit/miss counters on faces/list
      #   - [cs] admit_unverified, so a disabled validator no longer disables
      #     the Content Store
      #   - Content Store consulted AFTER PIT aggregation, as NFD does
      #   - CanBePrefix+MustBeFresh returns the FRESH descendant instead of an
      #     arbitrary one (hits 87 -> 1015; telemetry went FLIGHT-READY)
      #   - never forward an Interest back to the NODE it came from (12 wire
      #     copies per Interest -> expected 9, matching NFD)
      #   - [[strategy]] boot config, so /muas keeps the multicast strategy
      #     across a forwarder restart instead of silently reverting to
      #     best-route and partitioning the NDNSF SVS sync group
      #   - NameTrie::remove prunes empty nodes. It used to clear the value and
      #     leave the node, so the CS prefix index leaked a chain per unique
      #     Data name -- ~1-1.5 GB/day with video running, which exhausted
      #     every 3.7 GB node in ~3 days and locked the fleet up (2026-09-21)
      rev = "b83ed4975b7b1cf30d3d9858011aae1e9b089f0c";
      hash = "sha256-8SftBQh3W5Y/Wb38DUYhM7kHFUiT+gAH7YvKCcuD6c4=";
    };
    ndn-ext = fetchFromGitHub {
      owner = "Quarmire"; repo = "ndn-ext";
      rev = "27646a78bb0fd91a7f1fba1f295d5a9ad220fef6";
      hash = "sha256-rpoL75naH0dYzEm8UCZY3+Ylgd7q7wpTz/V/nGBt3bc=";
    };
    ndn-radio-drivers = fetchFromGitHub {
      owner = "Quarmire"; repo = "ndn-radio-drivers";
      rev = "05862c948e33f7e2ad8091d8729a5cee6fdfcba0";
      hash = "sha256-dBGei3dw8U2tSIqHeuFIpC/vE0L3IXayraUa0RyVIyI=";
    };
    ndn-radio = fetchFromGitHub {
      owner = "Quarmire"; repo = "ndn-radio";
      rev = "b4c3d96817ea4c8e50aaeeedca47462c5ac4054e";
      hash = "sha256-Ig+c4IPJB7R7AfUaODeCHWAXrnOed5yf8UfVCrJdzhs=";
    };
    ndn-embedded = fetchFromGitHub {
      owner = "Quarmire"; repo = "ndn-embedded";
      rev = "867d11799a1163d09dbfe05dcee9855c6c323cf5";
      hash = "sha256-1ozL5opMwU1mdC52y26WFkY6lc6e89llz3dy6QChD/k=";
    };
  };

  workspaceSrc = runCommand "ndn-workspace-src" { } ''
    mkdir -p $out
    ${lib.concatStringsSep "\n"
      (lib.mapAttrsToList (name: src: "cp -r ${src} $out/${name}") srcs)}
    chmod -R u+w $out
  '';
in
rustPlatform.buildRustPackage {
  pname = "ndn-fwd";
  version = "unstable-2026-fec-bulk-only-validated";

  src = workspaceSrc;
  # ndn-fwd is itself a cargo workspace nested under the assembled root:
  # cargoRoot points the vendoring hook at its lockfile, buildAndTestSubdir
  # cds there to compile.
  cargoRoot = "ndn-fwd";
  buildAndTestSubdir = "ndn-fwd";

  cargoLock.lockFile = "${srcs.ndn-fwd}/Cargo.lock";

  nativeBuildInputs = [ pkg-config ];
  buildInputs = [ libusb1 ];

  # The forwarder (with the userspace named-data radio) + the ndn-ctl /
  # ndn-ping / ndn-peek / ndn-put diagnostics the fabric + pilot ladder use.
  cargoBuildFlags = [ "-p" "ndn-fwd" "-p" "ndn-tools" ];
  buildFeatures = [ "ndn-fwd/radio-libusb" ];

  doCheck = false;

  meta = with lib; {
    description = "ndn-fwd NDN forwarder + ndn-tools with named-data radio (RTL8812EU)";
    homepage = "https://github.com/Quarmire/ndn-fwd";
    platforms = platforms.linux;
  };
}
