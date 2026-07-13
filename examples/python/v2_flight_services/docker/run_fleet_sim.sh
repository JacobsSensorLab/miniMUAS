#!/usr/bin/env bash
# Run the miniMUAS v2 MULTI-NODE fleet sim (dashboard + 4 roles) in the Linux
# container. This complements run_v2_stack_container.sh: that one runs the
# request/response stack once and exits; this one stands up the whole fleet on
# one NFD and serves the operator dashboard on :8080, or runs it headless.
#
# Usage:
#   ./run_fleet_sim.sh build                 # build the NDNSF image (same as the stack helper)
#   ./run_fleet_sim.sh sitl [start|stop|status]  # manage the host ArduPilot SITL fleet
#   ./run_fleet_sim.sh fleet [args...]       # stand up the fleet (SITL by default), publish :8080
#   ./run_fleet_sim.sh smoke [args...]       # headless end-to-end smoke + assertions
#   ./run_fleet_sim.sh shell                 # interactive shell in the image
#
# DEFAULT is ArduPilot SITL: every drone flies its own ArduCopter SITL instance.
# Start the SITL fleet on the host FIRST, then bring the docker fleet up:
#   ./run_fleet_sim.sh sitl start            # 3 SITL instances at the Memphis home
#   ./run_fleet_sim.sh fleet                 # agents connect to SITL SERIAL0 ports
#
# Lightweight kinematic bench (no SITL needed):
#   ./run_fleet_sim.sh fleet --kinematic
#
# Environment overrides (same as run_v2_stack_container.sh):
#   NDNSF_ROOT      NDN_Service_Framework checkout used as the BUILD CONTEXT.
#                   The image COPYs this tree, so local (even uncommitted)
#                   NDNSF changes — e.g. the ServiceResponse.timing fields —
#                   compile into the wrapper. (default: ~/Documents/Dev/NDN_Service_Framework)
#   UAS_IPBRC_ROOT  UAS-IPBRC checkout mounted in (relay.flight + deconfliction).
#   IMAGE           image tag (default: minimuas-v2-ndnsf, shared with the stack helper)
#   DOCKER_PLATFORM e.g. linux/amd64 if OpenABE/relic misbehaves on arm64
#   HTTP_PORT       host port to publish the dashboard on (default: 8080)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
V2_DIR="$(dirname "$SCRIPT_DIR")"
MINIMUAS_ROOT="$(cd "$V2_DIR/../../.." && pwd)"

NDNSF_ROOT="${NDNSF_ROOT:-$HOME/Documents/Dev/NDN_Service_Framework}"
UAS_IPBRC_HOST="${UAS_IPBRC_ROOT:-$HOME/Documents/Dev/UAS-IPBRC}"
IMAGE="${IMAGE:-minimuas-v2-ndnsf}"
HTTP_PORT="${HTTP_PORT:-8080}"
JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"

PLATFORM_ARGS=()
if [[ -n "${DOCKER_PLATFORM:-}" ]]; then
  PLATFORM_ARGS=(--platform "$DOCKER_PLATFORM")
fi

if [[ ! -f "$NDNSF_ROOT/install_ndnsf_stack.sh" ]]; then
  echo "NDNSF checkout not found at: $NDNSF_ROOT (set NDNSF_ROOT)" >&2
  exit 1
fi
if [[ ! -d "$UAS_IPBRC_HOST/relay/flight" ]]; then
  echo "UAS-IPBRC checkout not found at: $UAS_IPBRC_HOST (set UAS_IPBRC_ROOT)" >&2
  exit 1
fi

build_image() {
  local build_args=(--build-arg "JOBS=$JOBS")
  if [[ -n "${NFD_GIT_REF:-}" ]]; then
    build_args+=(--build-arg "NFD_GIT_REF=$NFD_GIT_REF")
  fi
  DOCKER_BUILDKIT=1 docker build "${PLATFORM_ARGS[@]}" "${build_args[@]}" \
    -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile" "$NDNSF_ROOT"
}

# Common docker run args: mount miniMUAS + UAS-IPBRC live, reach host SITL.
common_run_args() {
  echo \
    -v "$MINIMUAS_ROOT":/work/miniMUAS \
    -v "$UAS_IPBRC_HOST":/work/UAS-IPBRC \
    -e UAS_IPBRC_ROOT=/work/UAS-IPBRC \
    -e NDNSF_ROOT=/opt/NDN_Service_Framework \
    --add-host=host.docker.internal:host-gateway
}

ACTION="${1:-fleet}"
if [[ $# -gt 0 ]]; then shift; fi

case "$ACTION" in
  build)
    build_image
    ;;
  sitl)
    # Host-side ArduPilot SITL fleet (runs on the host, not in a container).
    exec "$V2_DIR/sim/start_sitl.sh" "${1:-start}"
    ;;
  fleet)
    build_image
    # -it so Ctrl-C tears the fleet down cleanly; publish the dashboard port.
    # shellcheck disable=SC2046
    docker run --rm -it "${PLATFORM_ARGS[@]}" $(common_run_args) \
      -p "${HTTP_PORT}:8080" \
      "$IMAGE" python3 examples/python/v2_flight_services/sim/launch_fleet.py \
      --ndnsf-root /opt/NDN_Service_Framework "$@"
    ;;
  smoke)
    build_image
    # headless; no TTY. Publish the port too so an operator could peek mid-run.
    # shellcheck disable=SC2046
    docker run --rm "${PLATFORM_ARGS[@]}" $(common_run_args) \
      -p "${HTTP_PORT}:8080" \
      "$IMAGE" python3 examples/python/v2_flight_services/sim/smoke.py \
      --ndnsf-root /opt/NDN_Service_Framework "$@"
    ;;
  shell)
    build_image
    # shellcheck disable=SC2046
    docker run --rm -it "${PLATFORM_ARGS[@]}" $(common_run_args) \
      -p "${HTTP_PORT}:8080" "$IMAGE" bash
    ;;
  -h|--help|help)
    sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Unknown action: $ACTION (use build|fleet|smoke|shell)" >&2
    exit 2
    ;;
esac
