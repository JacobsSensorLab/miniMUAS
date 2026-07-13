#!/usr/bin/env bash
# Launch one ArduPilot SITL (ArduCopter) instance PER fleet drone on the HOST,
# all at the sim's Memphis home so telemetry / localization / coordination line
# up with the docker fleet. Each drone agent (inside the container) connects to
# its instance's SERIAL0 over tcp:host.docker.internal:<port>, which also boots
# that SITL (SERIAL0 blocks "Waiting for connection" until a client attaches).
#
# Usage:
#   ./start_sitl.sh start      # launch COUNT instances (default 3), print port map
#   ./start_sitl.sh stop       # kill all sim_vehicle/arducopter processes
#   ./start_sitl.sh status     # show running instances + listening ports
#
# Env overrides:
#   ARDUPILOT_ROOT  ArduPilot checkout (default ~/Documents/Dev/ardupilot)
#   COUNT           number of instances / drones (default 3)
#   HOME_LL         home "lat,lon,alt,heading" (default Memphis 35.1208,-89.9347,50,0)
#   SPEEDUP         SITL speedup, 1 = wall clock (default 1; keep 1 so agent
#                   telemetry timestamps track real time)
#   REBUILD         set to 1 to let sim_vehicle rebuild arducopter first
#   LOG_DIR         where per-instance logs land
#
# Port map (SITL SERIAL0, what the agent connects to): instance I -> 5760 + 10*I
#   wuas-01 -> I0 -> 5760      iuas-01 -> I1 -> 5770      iuas-02 -> I2 -> 5780
# SERIAL1 (5762 + 10*I) is left free for an independent MAVLink observer.
set -euo pipefail

ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/Documents/Dev/ardupilot}"
COUNT="${COUNT:-3}"
HOME_LL="${HOME_LL:-35.1208,-89.9347,50,0}"
SPEEDUP="${SPEEDUP:-1}"
LOG_DIR="${LOG_DIR:-/tmp/muas-sitl}"
SIM_VEHICLE="$ARDUPILOT_ROOT/Tools/autotest/sim_vehicle.py"

# vehicle id per instance index (informational; the fleet uses the same order)
VEHICLES=(wuas-01 iuas-01 iuas-02 iuas-03 iuas-04 iuas-05)

port_for() { echo $((5760 + 10 * $1)); }

do_stop() {
  pkill -f 'sim_vehicle.py' 2>/dev/null || true
  pkill -f 'bin/arducopter' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'bin/arducopter' 2>/dev/null || true
  echo "stopped all SITL instances"
}

do_status() {
  echo "== sim_vehicle / arducopter processes =="
  pgrep -fl 'bin/arducopter' || echo "  (none)"
  echo "== listening MAVLink ports =="
  lsof -nP -iTCP -sTCP:LISTEN 2>/dev/null | grep -iE 'arducopter' || echo "  (none)"
}

do_start() {
  if [[ ! -f "$SIM_VEHICLE" ]]; then
    echo "sim_vehicle.py not found at $SIM_VEHICLE (set ARDUPILOT_ROOT)" >&2
    exit 1
  fi
  if [[ ! -x "$ARDUPILOT_ROOT/build/sitl/bin/arducopter" && "${REBUILD:-0}" != "1" ]]; then
    echo "arducopter SITL binary not built. Build once with:" >&2
    echo "  cd $ARDUPILOT_ROOT && ./waf configure --board sitl && ./waf copter" >&2
    echo "or re-run this with REBUILD=1 to let sim_vehicle build it." >&2
    exit 1
  fi
  local rebuild_flag="-N"
  [[ "${REBUILD:-0}" == "1" ]] && rebuild_flag=""

  mkdir -p "$LOG_DIR"
  echo "Launching $COUNT ArduCopter SITL instance(s) at home=$HOME_LL (speedup=$SPEEDUP)"
  echo "Port map (agent --mavlink-endpoint = tcp:host.docker.internal:<port>):"
  for ((i = 0; i < COUNT; i++)); do
    local port; port=$(port_for "$i")
    local veh="${VEHICLES[$i]:-drone-$i}"
    # Each instance runs in its own working dir so eeprom.bin/logs never collide.
    local wd="$LOG_DIR/i$i"
    mkdir -p "$wd"
    ( cd "$wd" && nohup python3 "$SIM_VEHICLE" -v ArduCopter -I"$i" $rebuild_flag \
        --no-mavproxy --speedup "$SPEEDUP" -A "--home $HOME_LL" \
        > "$LOG_DIR/i$i.log" 2>&1 & echo $! > "$LOG_DIR/i$i.pid" )
    printf "  %-8s I%d  SERIAL0 tcp:%d  (observer SERIAL1 tcp:%d)\n" \
      "$veh" "$i" "$port" $((port + 2))
  done
  echo
  echo "Warming up (boot + GPS/EKF lock) so the agents get position immediately..."
  warmup "$COUNT"
  echo
  echo "SITL fleet ready and GPS-locked. Start the docker fleet with:"
  echo "  cd $(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")/docker && ./run_fleet_sim.sh fleet"
  echo "Logs: $LOG_DIR/i<N>.log   |   stop with: $0 stop"
}

# Boot + GPS-lock each instance by briefly connecting to its SERIAL0, then
# disconnect. SITL only starts its clock when SERIAL0 is first connected and GPS
# lock takes ~30 s; the agent's MAVLink backend gives up after 30 s of no
# GLOBAL_POSITION_INT. Once locked, SITL keeps the fix across reconnect, so the
# agent that attaches next gets position instantly. Requires pymavlink on host.
warmup() {
  local count="$1"
  python3 - "$count" <<'PY'
import sys, time
try:
    from pymavlink import mavutil
except Exception as e:
    print("  (pymavlink not available on host; skipping warmup: %s)" % e)
    print("  Install with: pip3 install pymavlink   — otherwise the agents may")
    print("  time out waiting for GPS lock on a cold SITL.")
    sys.exit(0)
count = int(sys.argv[1])
for i in range(count):
    port = 5760 + 10 * i
    try:
        c = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % port)
        c.wait_heartbeat(timeout=20)
        c.mav.request_data_stream_send(c.target_system, c.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        t0 = time.time(); ok = False
        while time.time() - t0 < 70:
            m = c.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=4)
            if m and (m.lat != 0 or m.lon != 0):
                print("  I%d (tcp:%d): GPS locked at %.5f,%.5f in %.0fs"
                      % (i, port, m.lat/1e7, m.lon/1e7, time.time()-t0))
                ok = True; break
        if not ok:
            print("  I%d (tcp:%d): WARNING no GPS lock in 70s" % (i, port))
        c.close()
        time.sleep(1)
    except Exception as e:
        print("  I%d (tcp:%d): warmup error %s" % (i, port, e))
PY
}

case "${1:-start}" in
  start) do_start ;;
  stop)  do_stop ;;
  status) do_status ;;
  -h|--help|help) sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ;;
  *) echo "Unknown action: ${1:-} (use start|stop|status)" >&2; exit 2 ;;
esac
