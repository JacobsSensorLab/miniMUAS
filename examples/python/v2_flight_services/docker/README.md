# Linux Container Environment for the v2 NDNSF Stack

The NDNSF native stack (the matianxing1992 ndn-cxx fork, NDNSD, ndn-svs,
OpenABE, NAC-ABE, and NFD) is documented and built for Ubuntu/Linux only:
the installer is apt/ldconfig/`.so` specific and OpenABE is pinned to
OpenSSL 1.1. Rather than porting that chain to macOS, this directory runs
the real stack in an Ubuntu 22.04 container while miniMUAS and UAS-IPBRC
stay live-mounted from the host, so the edit/test loop is unchanged. This
also matches the eventual Linux companion-computer deployment targets.

Requirements: Docker Desktop (or any docker engine with BuildKit).

## Usage

```sh
cd examples/python/v2_flight_services/docker

# First build takes a while (it compiles the whole NDN dependency chain);
# later builds reuse cached layers and only rebuild NDNSF itself.
./run_v2_stack_container.sh build

# Full real-NDNSF v2 stack: controller, GCS provider, IUAS provider with
# real relay.flight primitive execution, WUAS user, local NFD.
./run_v2_stack_container.sh stack

# Just the offline primitive-execution smoke test inside Linux.
./run_v2_stack_container.sh smoke
./run_v2_stack_container.sh smoke -- --no-native-orbit

# Preflight checks or an interactive shell.
./run_v2_stack_container.sh preflight
./run_v2_stack_container.sh shell
```

Role logs land in `results/v2_ndnsf/` on the host because miniMUAS is
bind-mounted.

## How it is wired

- The docker build context is your `NDN_Service_Framework` checkout
  (default `~/Documents/Dev/NDN_Service_Framework`, override with
  `NDNSF_ROOT`), so local NDNSF changes are built into the image by the
  repo's own `install_ndnsf_stack.sh --no-dependencies`.
- The five external dependencies are separate cached image layers, so
  editing NDNSF sources does not recompile ndn-cxx/OpenABE/etc.
- The python wrapper is additionally built in-tree
  (`setup.py build_ext --inplace`) because the miniMUAS role scripts import
  `ndnsf` by inserting `pythonWrapper` into `sys.path`.
- `UAS_IPBRC_ROOT=/work/UAS-IPBRC` is set in the container, so the IUAS
  provider finds `relay.flight` and executes real primitive plans.
- NFD comes from the named-data PPA when available for the platform;
  otherwise it is built from source against the installed ndn-cxx fork.
  If the source fallback fails to configure, pin a compatible tag:
  `NFD_GIT_REF=NFD-22.12 ./run_v2_stack_container.sh build`.

## Troubleshooting

- OpenABE's bundled relic build is the most platform-sensitive step. If it
  fails on Apple Silicon (linux/arm64), build the amd64 image under
  emulation instead: `DOCKER_PLATFORM=linux/amd64 ./run_v2_stack_container.sh build`.
- To force a clean rebuild of one dependency layer, change its
  `--build-arg` repo URL or use `docker build --no-cache`.
- `stack` runs `nfd-start` inside the container; nothing touches NFD on
  the host.
