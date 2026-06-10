#!/usr/bin/env bash
# Build and run the miniMUAS v2 NDNSF stack in a Linux container.
#
# The NDNSF native stack (ndn-cxx fork, NDNSD, ndn-svs, OpenABE, NAC-ABE,
# NFD) is Linux-only; this wraps it in Docker so the full real-NDN stack
# runs on macOS while miniMUAS and UAS-IPBRC stay live-mounted from the
# host for normal editing.
#
# Usage:
#   ./run_v2_stack_container.sh build               # build the image only
#   ./run_v2_stack_container.sh stack [args...]     # full NDNSF stack + NFD
#   ./run_v2_stack_container.sh smoke [args...]     # investigate_plan.py only
#   ./run_v2_stack_container.sh preflight [args...] # preflight checks
#   ./run_v2_stack_container.sh shell               # interactive shell
#
# Environment overrides:
#   NDNSF_ROOT       NDN_Service_Framework checkout used as build context
#                    (default: ~/Documents/Dev/NDN_Service_Framework)
#   UAS_IPBRC_ROOT   UAS-IPBRC checkout mounted into the container
#                    (default: ~/Documents/Dev/UAS-IPBRC)
#   IMAGE            image tag (default: minimuas-v2-ndnsf)
#   DOCKER_PLATFORM  e.g. linux/amd64 if OpenABE/relic misbehaves on arm64
#   NFD_GIT_REF      NFD tag for the source-build fallback (e.g. NFD-22.12)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
V2_DIR="$(dirname "$SCRIPT_DIR")"
MINIMUAS_ROOT="$(cd "$V2_DIR/../../.." && pwd)"

NDNSF_ROOT="${NDNSF_ROOT:-$HOME/Documents/Dev/NDN_Service_Framework}"
UAS_IPBRC_HOST="${UAS_IPBRC_ROOT:-$HOME/Documents/Dev/UAS-IPBRC}"
IMAGE="${IMAGE:-minimuas-v2-ndnsf}"
JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"

PLATFORM_ARGS=()
if [[ -n "${DOCKER_PLATFORM:-}" ]]; then
  PLATFORM_ARGS=(--platform "$DOCKER_PLATFORM")
fi

if [[ ! -f "$NDNSF_ROOT/install_ndnsf_stack.sh" ]]; then
  echo "NDNSF checkout not found at: $NDNSF_ROOT" >&2
  echo "Set NDNSF_ROOT to your NDN_Service_Framework checkout." >&2
  exit 1
fi
if [[ ! -d "$UAS_IPBRC_HOST/relay/flight" ]]; then
  echo "UAS-IPBRC checkout not found at: $UAS_IPBRC_HOST" >&2
  echo "Set UAS_IPBRC_ROOT to your UAS-IPBRC checkout." >&2
  exit 1
fi

build_image() {
  local build_args=(--build-arg "JOBS=$JOBS")
  if [[ -n "${NFD_GIT_REF:-}" ]]; then
    build_args+=(--build-arg "NFD_GIT_REF=$NFD_GIT_REF")
  fi
  DOCKER_BUILDKIT=1 docker build "${PLATFORM_ARGS[@]}" "${build_args[@]}" \
    -t "$IMAGE" \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$NDNSF_ROOT"
}

docker_run() {
  docker run --rm -it "${PLATFORM_ARGS[@]}" \
    -v "$MINIMUAS_ROOT":/work/miniMUAS \
    -v "$UAS_IPBRC_HOST":/work/UAS-IPBRC \
    -e UAS_IPBRC_ROOT=/work/UAS-IPBRC \
    -e NDNSF_ROOT=/opt/NDN_Service_Framework \
    "$IMAGE" "$@"
}

ACTION="${1:-stack}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "$ACTION" in
  build)
    build_image
    ;;
  stack)
    build_image
    docker_run python3 examples/python/v2_flight_services/run_ndnsf_stack.py \
      --run --start-local-nfd \
      --ndnsf-root /opt/NDN_Service_Framework "$@"
    ;;
  smoke)
    build_image
    docker_run python3 examples/python/v2_flight_services/investigate_plan.py "$@"
    ;;
  preflight)
    build_image
    docker_run python3 examples/python/v2_flight_services/preflight_ndnsf.py \
      --ndnsf-root /opt/NDN_Service_Framework --require-nfd "$@"
    ;;
  shell)
    build_image
    docker_run bash
    ;;
  -h|--help|help)
    sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Unknown action: $ACTION (use build|stack|smoke|preflight|shell)" >&2
    exit 2
    ;;
esac
