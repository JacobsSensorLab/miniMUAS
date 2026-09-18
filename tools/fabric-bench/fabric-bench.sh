#!/usr/bin/env bash
#
# fabric-bench — NDN fabric throughput/latency/loss over wireless, with NO
# dependency on the miniMUAS application stack.
#
# Why this exists
# ---------------
# Every forwarder comparison run through the miniMUAS video path was
# confounded by the application: a per-packet KeyChain construction, a 15 fps
# clamp, an FEC group sized per frame instead of per packet, and NDNSF
# re-verifying the producer's own signature. Each looked like a forwarder
# ceiling and none of them were. This harness removes the application
# entirely: ndn-iperf with `--sign-mode none` touches no keychain, no ABE, no
# NDNSF, and no agent -- so what it measures is the fabric.
#
# Method guards baked in, each one paid for by a wrong conclusion:
#   * SETTLE after any cell switch. A window sampled inside that period is
#     worthless, including as one arm of an A/B.
#   * Forwarder counters are read as BEFORE->AFTER DELTAS. They are cumulative
#     since forwarder start; absolute values silently span every previous run.
#   * Latency is reported as percentiles. A mean hides exactly the tail that
#     makes video stutter.
#   * --flows runs concurrent flows, because single-flow throughput says
#     nothing about whether a forwarder shares capacity.
#   * Runs are validated: a flow that returns no data is reported as FAILED,
#     never averaged into an aggregate.
#
# Usage
#   ./fabric-bench.sh [options]
#     --stack <nfd|ndn-fwd|both>   cell(s) to test           (default: both)
#     --server <host>              node that produces        (default: $BENCH_SERVER)
#     --client <host>              node that consumes        (default: $BENCH_CLIENT)
#     --duration <sec>             per-flow duration         (default: 20)
#     --flows <n>                  concurrent flows          (default: 3)
#     --size <bytes>               Data payload size         (default: 8192)
#     --window <n>                 client pipeline window    (default: 64)
#     --settle <sec>               wait after a cell switch  (default: 150)
#     --ping-count <n>             ndnping probes            (default: 100)
#     --no-switch                  measure the current cell, do not switch
#     --check                      validate the environment and exit
#
# Env: BENCH_SERVER, BENCH_CLIENT, BENCH_NODES (space-separated, for switching)
set -uo pipefail

STACKS="both"; DURATION=20; FLOWS=3; SIZE=8192; WINDOW=64
SETTLE=150; PING_COUNT=100; NO_SWITCH=0; CHECK_ONLY=0
SERVER="${BENCH_SERVER:-}"; CLIENT="${BENCH_CLIENT:-}"
NODES="${BENCH_NODES:-}"
PREFIX="/fabricbench/$$"

while [ $# -gt 0 ]; do
  case "$1" in
    --stack) STACKS="$2"; shift 2;;
    --server) SERVER="$2"; shift 2;;
    --client) CLIENT="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --flows) FLOWS="$2"; shift 2;;
    --size) SIZE="$2"; shift 2;;
    --window) WINDOW="$2"; shift 2;;
    --settle) SETTLE="$2"; shift 2;;
    --ping-count) PING_COUNT="$2"; shift 2;;
    --no-switch) NO_SWITCH=1; shift;;
    --check) CHECK_ONLY=1; shift;;
    -h|--help) sed -n '2,40p' "$0"; exit 0;;
    *) echo "unknown option: $1" >&2; exit 2;;
  esac
done
[ "$STACKS" = "both" ] && STACKS="ndn-fwd nfd"
# With --no-switch there is exactly one cell to measure; iterating "both" would
# just run the same configuration twice under two different labels.
[ "$NO_SWITCH" = 1 ] && STACKS="current"
[ -n "$SERVER" ] || { echo "need --server (or BENCH_SERVER)" >&2; exit 2; }
[ -n "$CLIENT" ] || { echo "need --client (or BENCH_CLIENT)" >&2; exit 2; }
[ -n "$NODES" ] || NODES="$SERVER $CLIENT"

# ---------------------------------------------------------------- logging --
# Everything is logged. Derived numbers are never the only record: each run
# keeps the raw tool output, the counters, and the node metadata, so a result
# can be re-analysed without re-running it. Today's parsing bugs (a throughput
# line missed because the parser read only the tail; a route add that printed
# success after a usage error) were both invisible in the summary and obvious
# in the raw log.
RUN_TS="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${BENCH_OUT:-$(cd "$(dirname "$0")" && pwd)/results}/${RUN_TS}"
mkdir -p "$RUN_DIR"
RUNLOG="${RUN_DIR}/runlog.txt"

log()  { printf '%s\n' "$*" | tee -a "$RUNLOG"; }
logf() { printf '%s\n' "$*" >> "$RUNLOG"; }

# Run a remote command, logging the command and its full output verbatim.
# Stores nothing behind the caller's back: stdout is returned as usual.
remote() {
  local host=$1; shift
  local cmd="$*" out rc
  out=$(timeout 180 ssh -o ConnectTimeout=15 -o BatchMode=yes "$host" "$cmd" 2>&1); rc=$?
  { echo "--- [$(date -u +%H:%M:%SZ)] ${host}: ${cmd}"
    echo "$out"
    echo "--- rc=${rc}"; } >> "$RUNLOG"
  printf '%s' "$out"
  return $rc
}

sshq() { timeout 120 ssh -o ConnectTimeout=15 -o BatchMode=yes "$@" 2>/dev/null; }

# Capture a remote file into the run directory.
fetch() { # host remote_path local_name
  local host=$1 rp=$2 ln=$3
  sshq "$host" "cat '$rp'" > "${RUN_DIR}/${ln}" 2>/dev/null || true
}

# Node metadata: what was actually running when this was measured.
snapshot_node() { # host label
  local host=$1 label=$2
  { echo "=== ${label} (${host}) ==="
    remote "$host" "hostname; uptime; readlink -f /run/current-system; \
      sudo muas-fabric status 2>/dev/null | head -5; \
      ndn-ctl face list 2>/dev/null"
  } >> "${RUN_DIR}/nodes.txt" 2>&1
}

# The server's fabric (mesh) address — the client needs a route toward it.
fabric_ip() { sshq "$1" "ip -4 -br addr show mesh0" | awk '{print $3}' | cut -d/ -f1; }

# Face on $1 whose remote is the given fabric IP on the NDN port.
face_to() {
  sshq "$1" "ndn-ctl face list" | awk -v ip="$2" '
    /^faceid=/ { fid=$1; sub("faceid=","",fid) }
    $0 ~ ("remote: udp4://" ip ":6363") { print fid; exit }'
}

# Route the bench prefix from client to server. Returns 0 on success.
# ndnping reporting "100% nacked" means THIS failed, not that the link is bad.
setup_route() {
  local prefix=$1 ip fid
  ip=$(fabric_ip "$SERVER")
  [ -n "$ip" ] || { echo "  ! cannot read server fabric IP (mesh0)"; return 1; }
  fid=$(face_to "$CLIENT" "$ip")
  [ -n "$fid" ] || { echo "  ! client has no face to ${ip}:6363"; return 1; }
  # Correct syntax is `--face <FACE> <PREFIX>`; the positional form silently
  # prints usage and exits non-zero. VERIFY the route landed rather than
  # trusting the command -- an unverified "routed" message that is actually a
  # usage error turns every subsequent number into a NoRoute Nack.
  if ! sshq "$CLIENT" "ndn-ctl route add --face ${fid} ${prefix}" >/dev/null; then
    echo "  ! 'ndn-ctl route add' failed for ${prefix} via face ${fid}"
    return 1
  fi
  if ! sshq "$CLIENT" "ndn-ctl route list" | grep -q -- "${prefix}"; then
    echo "  ! route ${prefix} is not in the client RIB after add"
    return 1
  fi
  echo "  routed ${prefix} -> face ${fid} (${ip})  [verified in RIB]"
  BENCH_ROUTE_FACE="$fid"
  return 0
}
teardown_route() {
  [ -n "${BENCH_ROUTE_FACE:-}" ] || return 0
  sshq "$CLIENT" "ndn-ctl route remove --face ${BENCH_ROUTE_FACE} $1" >/dev/null
}

need_tools() {
  local host=$1 missing=""
  for t in ndn-iperf ndnping ndnpingserver ndn-ctl; do
    sshq "$host" "command -v $t >/dev/null" || missing="$missing $t"
  done
  [ -z "$missing" ] || { echo "  MISSING on $host:$missing"; return 1; }
  echo "  ok: $host"
}

# Cumulative counters -> only deltas are meaningful. Emits "in_data out_data".
counters() {
  sshq "$1" "ndn-ctl face list" | awk '
    /remote: udp4:/ { u=1 }
    /^  in:/  { if(u) for(i=1;i<=NF;i++) if($i~/^data=/){d=$i;sub("data=","",d); ind+=d} }
    /^  out:/ { if(u) for(i=1;i<=NF;i++) if($i~/^data=/){d=$i;sub("data=","",d); outd+=d}; u=0 }
    END { printf "%d %d\n", ind+0, outd+0 }'
}

set_cell() {
  local stack=$1
  [ "$NO_SWITCH" = 1 ] && { echo "  (--no-switch: measuring current cell)"; return; }
  for n in $NODES; do sshq "$n" "sudo muas-fabric set ${stack} wifi" >/dev/null; done
  echo "  switched to '${stack} wifi'; settling ${SETTLE}s (a sample taken inside this window is worthless)"
  sleep "$SETTLE"
}

if [ "$CHECK_ONLY" = 1 ]; then
  echo "environment check:"
  rc=0
  for h in $SERVER $CLIENT; do need_tools "$h" || rc=1; done
  exit $rc
fi

cat > "${RUN_DIR}/manifest.txt" <<MANIFEST
timestamp_utc : ${RUN_TS}
stacks        : ${STACKS}
server        : ${SERVER}
client        : ${CLIENT}
nodes         : ${NODES}
duration_s    : ${DURATION}
flows         : ${FLOWS}
data_size_B   : ${SIZE}
window        : ${WINDOW}
settle_s      : ${SETTLE}
ping_count    : ${PING_COUNT}
no_switch     : ${NO_SWITCH}
prefix        : ${PREFIX}
MANIFEST
log "run directory: ${RUN_DIR}"
log "$(cat "${RUN_DIR}/manifest.txt")"

for stack in $STACKS; do
  echo "================================================================"
  echo "STACK: ${stack}   server=${SERVER}  client=${CLIENT}"
  echo "================================================================"
  set_cell "$stack"
  sshq "$CLIENT" "sudo muas-fabric status" | grep -E "^active|^health" | sed 's/^/  /'

  if ! setup_route "$PREFIX"; then
    echo "  SKIPPING ${stack}: no route means every result would be a Nack, not a measurement"
    continue
  fi

  # ---- latency / loss (independent of throughput; run first, unloaded) ----
  sshq "$SERVER" "pkill -f 'ndnpingserver ${PREFIX}'" >/dev/null
  sshq "$SERVER" "nohup ndnpingserver ${PREFIX} >/tmp/fb-pingsrv.log 2>&1 &" >/dev/null
  sleep 2
  log "  -- latency/loss (unloaded, ${PING_COUNT} probes) --"
  sshq "$CLIENT" "ndnping -c ${PING_COUNT} -i 20 ${PREFIX}" \
    | grep -E "packets transmitted|rtt" | sed 's/^/     /'

  # ---- throughput, N concurrent flows ----
  sshq "$SERVER" "pkill -f 'ndn-iperf server'" >/dev/null
  for f in $(seq 1 "$FLOWS"); do
    sshq "$SERVER" "nohup ndn-iperf server --prefix ${PREFIX}/f${f} --size ${SIZE} -q \
                      >/tmp/fb-iperf-s${f}.log 2>&1 &" >/dev/null
  done
  sleep 3

  snapshot_node "$SERVER" "server/${stack}"
  snapshot_node "$CLIENT" "client/${stack}"
  read -r cb_in cb_out <<<"$(counters "$CLIENT")"
  echo "${stack} before in_data=${cb_in} out_data=${cb_out}" >> "${RUN_DIR}/counters.txt"
  log "  -- throughput: ${FLOWS} concurrent flow(s), ${DURATION}s, ${SIZE}B Data --"
  for f in $(seq 1 "$FLOWS"); do
    sshq "$CLIENT" "ndn-iperf client --prefix ${PREFIX}/f${f} --duration ${DURATION} \
                      --window ${WINDOW} --sign-mode none -q > /tmp/fb-iperf-c${f}.out 2>&1" &
  done
  wait
  read -r ca_in ca_out <<<"$(counters "$CLIENT")"

  total=0; failed=0
  for f in $(seq 1 "$FLOWS"); do
    # Keep the RAW client output. Parsing only the tail previously reported a
    # successful flow as FAILED, because the throughput line sits above the
    # RTT block. Grep the whole file, and keep the file either way.
    fetch "$CLIENT" "/tmp/fb-iperf-c${f}.out" "${stack}-flow${f}-client.txt"
    fetch "$SERVER" "/tmp/fb-iperf-s${f}.log" "${stack}-flow${f}-server.txt"
    line=$(grep -iE "throughput|mbps|mbit" "${RUN_DIR}/${stack}-flow${f}-client.txt" 2>/dev/null | tail -1)
    rtt=$(grep -iE "p50=" "${RUN_DIR}/${stack}-flow${f}-client.txt" 2>/dev/null | tail -1)
    retx=$(grep -iE "retransmit" "${RUN_DIR}/${stack}-flow${f}-client.txt" 2>/dev/null | tail -1)
    if [ -z "$line" ]; then
      log "     flow ${f}: NO THROUGHPUT LINE (raw kept: ${stack}-flow${f}-client.txt)"
      failed=$((failed+1))
    else
      log "     flow ${f}:${line#*:}"
      [ -n "$rtt" ]  && log "              rtt:${rtt#*RTT:}"
      [ -n "$retx" ] && log "              ${retx##*( )}"
      v=$(echo "$line" | grep -oE "[0-9]+\.[0-9]+" | tail -1)
      total=$(awk -v a="$total" -v b="${v:-0}" 'BEGIN{print a+b}')
    fi
  done
  echo "     ---------------------------------------------"
  log "     aggregate_mbps: ${total} (sum of ${FLOWS} flows, ${failed} without a throughput line)"
  echo "     fairness:  compare per-flow values above; a wide spread means the"
  echo "                forwarder is not sharing capacity, which single-flow"
  echo "                throughput cannot reveal"
  echo "${stack} after  in_data=${ca_in} out_data=${ca_out}" >> "${RUN_DIR}/counters.txt"
  log "     client face Data delta over the run: in=$((ca_in-cb_in)) out=$((ca_out-cb_out))"
  {
    echo "{\"stack\":\"${stack}\",\"flows\":${FLOWS},\"duration_s\":${DURATION},"
    echo " \"data_size_B\":${SIZE},\"window\":${WINDOW},"
    echo " \"aggregate_mbps\":${total},\"failed_flows\":${failed},"
    echo " \"client_in_data_delta\":$((ca_in-cb_in)),\"client_out_data_delta\":$((ca_out-cb_out))}"
  } >> "${RUN_DIR}/summary.jsonl"

  sshq "$SERVER" "pkill -f 'ndn-iperf server'; pkill -f 'ndnpingserver ${PREFIX}'" >/dev/null
  teardown_route "$PREFIX"
done
log "artifacts written to ${RUN_DIR}:"
ls -1 "$RUN_DIR" | sed 's/^/  /' | tee -a "$RUNLOG"
log "FABRIC-BENCH-COMPLETE"
