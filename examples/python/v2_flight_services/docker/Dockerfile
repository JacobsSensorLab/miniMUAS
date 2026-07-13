# syntax=docker/dockerfile:1
# Linux build/runtime environment for the miniMUAS v2 NDNSF stack.
#
# The build context must be an NDN_Service_Framework checkout. Use the
# run_v2_stack_container.sh helper next to this file, or call docker
# directly:
#
#   DOCKER_BUILDKIT=1 docker build -t minimuas-v2-ndnsf \
#     -f <miniMUAS>/examples/python/v2_flight_services/docker/Dockerfile \
#     ~/Documents/Dev/NDN_Service_Framework
#
# The dependency chain mirrors install_ndnsf_stack.sh but as separate
# cached layers, so editing NDNSF sources only rebuilds the final layer.
# NFD is added on top because the installer does not cover it.

FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

# --- Layer 1: OS build/runtime packages (mirrors install_ndnsf_stack.sh) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git pkg-config cmake python3 python3-pip wget curl \
        ca-certificates python3-dev python3-setuptools python3-wheel \
        python3-venv autoconf automake libtool m4 bison flex ninja-build \
        libgmp-dev libssl-dev libboost-all-dev libsqlite3-dev libpcap-dev \
        libsodium-dev libz-dev liblog4cxx-dev sqlite3 \
        libsystemd-dev libcap-dev \
        software-properties-common gnupg lsb-release procps psmisc sudo \
    && rm -rf /var/lib/apt/lists/*

ARG JOBS=4
ARG NDNCXX_REPO_URL=https://github.com/matianxing1992/ndn-cxx.git
ARG NDNSD_REPO_URL=https://github.com/matianxing1992/NDNSD.git
ARG NDNSVS_REPO_URL=https://github.com/matianxing1992/ndn-svs.git
ARG NACABE_REPO_URL=https://github.com/matianxing1992/NAC-ABE.git
ARG OPENABE_REPO_URL=https://github.com/zeutro/openabe.git

ENV OPENABE_PREFIX=/opt/ndn-deps/local/openabe

# --- Layer 2: ndn-cxx (version-consistent fork) -----------------------------
RUN git clone --depth 1 "$NDNCXX_REPO_URL" /opt/ndn-deps/ndn-cxx \
    && cd /opt/ndn-deps/ndn-cxx \
    && ./waf configure && ./waf -j"$JOBS" && ./waf install && ldconfig

# --- Layer 3: ndn-svs --------------------------------------------------------
# Built before NDNSD: this NDNSD fork checks for libndn-svs at configure time.
RUN git clone --depth 1 "$NDNSVS_REPO_URL" /opt/ndn-deps/ndn-svs \
    && cd /opt/ndn-deps/ndn-svs \
    && ./waf configure && ./waf -j"$JOBS" && ./waf install && ldconfig

# --- Layer 4: NDNSD ----------------------------------------------------------
RUN git clone --depth 1 "$NDNSD_REPO_URL" /opt/ndn-deps/NDNSD \
    && cd /opt/ndn-deps/NDNSD \
    && ./waf configure && ./waf -j"$JOBS" && ./waf install && ldconfig

# --- Layer 5: OpenABE with its private OpenSSL 1.1 ---------------------------
# Ubuntu 22.04 ships OpenSSL 3 and GCC 11; OpenABE and its bundled relic
# 0.5.0 are pinned to the Ubuntu 20.04 era (GCC 11 rejects relic's blake2
# headers with "size of array element is not a multiple of its alignment").
# This layer builds the OpenABE chain with GCC 9 and OpenABE's private
# OpenSSL 1.1, mirroring install_ndnsf_stack.sh without replacing the system
# compiler or OpenSSL. Cross-version C++ linking is safe: GCC 9 and 11 share
# the same libstdc++ ABI.
RUN apt-get update && apt-get install -y --no-install-recommends gcc-9 g++-9 unzip libfl-dev \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --depth 1 "$OPENABE_REPO_URL" /opt/ndn-deps/openabe \
    && cd /opt/ndn-deps/openabe \
    && . ./env \
    && export CC=gcc-9 CXX=g++-9 \
    && make -C deps/openssl \
    && make -C deps/relic \
    && make -C deps/gtest \
    && BISON="$(command -v bison)" FLEX="$(command -v flex)" make \
    && make INSTALL_PREFIX="$OPENABE_PREFIX" install \
    && echo "$OPENABE_PREFIX/lib" > /etc/ld.so.conf.d/openabe.conf \
    && ldconfig

# --- Layer 6: NAC-ABE against the private OpenABE ----------------------------
RUN git clone --depth 1 "$NACABE_REPO_URL" /opt/ndn-deps/NAC-ABE \
    && cd /opt/ndn-deps/NAC-ABE \
    && env CMAKE_PREFIX_PATH="$OPENABE_PREFIX" \
        CMAKE_INCLUDE_PATH="$OPENABE_PREFIX/include" \
        CMAKE_LIBRARY_PATH="$OPENABE_PREFIX/lib" \
        CXXFLAGS="-I$OPENABE_PREFIX/include" \
        LDFLAGS="-L$OPENABE_PREFIX/lib -Wl,-rpath,$OPENABE_PREFIX/lib" \
        LD_LIBRARY_PATH="$OPENABE_PREFIX/lib" \
        cmake -S . -B build \
          -DCMAKE_BUILD_RPATH="$OPENABE_PREFIX/lib" \
          -DCMAKE_INSTALL_RPATH="$OPENABE_PREFIX/lib" \
    && cmake --build build -j"$JOBS" \
    && cmake --install build && ldconfig

# --- Layer 7: NFD (PPA when available, otherwise built from source) ----------
# NFD only needs to speak the NDN wire protocol over the local socket, so the
# PPA build (linked against its own packaged ndn-cxx) is fine alongside the
# fork in /usr/local. The source fallback builds against the fork; pin a tag
# with --build-arg NFD_GIT_REF=NFD-x.y if the default branch will not
# configure against it.
ARG NFD_GIT_REF=""
RUN ( add-apt-repository -y ppa:named-data/ppa \
      && apt-get update \
      && apt-get install -y --no-install-recommends nfd \
      && rm -rf /var/lib/apt/lists/* ) \
  || ( (add-apt-repository -ry ppa:named-data/ppa || true) \
      && git clone --recursive https://github.com/named-data/NFD /opt/ndn-deps/NFD \
      && cd /opt/ndn-deps/NFD \
      && if [[ -n "$NFD_GIT_REF" ]]; then \
           git checkout "$NFD_GIT_REF" && git submodule update --init; \
         fi \
      && ./waf configure && ./waf -j"$JOBS" && ./waf install && ldconfig \
      && cp /usr/local/etc/ndn/nfd.conf.sample /usr/local/etc/ndn/nfd.conf )

# --- Layer 8: NDNSF from the local checkout (the docker build context) -------
# protobuf is required by NDNSF's waf configure but only appears in the
# installer's optional --with-nfd-nlsr-deps package group. gtkmm-3.0 is an
# unconditional configure check (only the UAV GUI apps link it, but configure
# refuses to pass without it), so the dev headers are installed too.
#
# The waf steps are run directly instead of via install_ndnsf_stack.sh: the
# installer's three editable pip installs all register as package
# "UNKNOWN-0.0.0" (their setup.py files have no name=), so each install
# uninstalls the previous one and the installer's own `import ndnsf` smoke
# check fails. The miniMUAS role scripts import ndnsf by inserting
# pythonWrapper into sys.path, so the in-tree extension build below is the
# only Python install actually needed.
COPY . /opt/NDN_Service_Framework
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libprotobuf-dev protobuf-compiler libgtkmm-3.0-dev \
    && rm -rf /var/lib/apt/lists/* \
    && cd /opt/NDN_Service_Framework \
    && ./waf configure && ./waf -j"$JOBS" && ./waf install && ldconfig \
    && python3 -m pip install --no-cache-dir pybind11 \
    && cd pythonWrapper \
    && python3 setup.py build_ext --inplace \
    && python3 -c "import sys; sys.path.insert(0, '.'); import ndnsf; print('ndnsf in-tree import OK')"

# --- Layer 8b: python runtime deps for the flight path AND the dashboard ------
# pymavlink: MAVLink-backed IUAS flight (run_*_agent.py --mavlink-endpoint, e.g.
#   tcp:host.docker.internal:5762 to reach ArduPilot SITL on the host).
# pyyaml: relay.drone.mavlink imports relay.scenario, which needs it.
# aiohttp: run_dashboard.py serves its operator UI + /ws over aiohttp, and
#   sim/ws_driver.py connects to it. The older run_ndnsf_stack.py path never
#   launched the dashboard, so this was not needed until the multi-node sim.
# A separate layer so toggling these never invalidates the NDNSF build above.
RUN python3 -m pip install --no-cache-dir pymavlink pyyaml aiohttp

# --- Layer 9: container-friendly nfd-start/nfd-stop ---------------------------
# The PPA's nfd-start is a systemd wrapper, and containers have no systemd
# PID 1. These shims sit earlier in PATH (/usr/local/bin) and run the nfd
# daemon directly, with a socket-readiness wait so callers can proceed
# immediately after nfd-start returns.
COPY <<'EOF' /usr/local/bin/nfd-start
#!/usr/bin/env bash
set -e
if pgrep -x nfd >/dev/null 2>&1; then
    exit 0
fi
conf=""
for candidate in /etc/ndn/nfd.conf /usr/local/etc/ndn/nfd.conf; do
    if [ -f "$candidate" ]; then conf="$candidate"; break; fi
done
if [ -z "$conf" ]; then
    for sample in /etc/ndn/nfd.conf.sample /usr/local/etc/ndn/nfd.conf.sample; do
        if [ -f "$sample" ]; then
            conf="${sample%.sample}"
            cp "$sample" "$conf"
            break
        fi
    done
fi
if [ -z "$conf" ]; then
    echo "nfd-start shim: no nfd.conf found" >&2
    exit 1
fi
# systemd normally provides RuntimeDirectory=nfd for the unix socket path.
# Derive the actual socket path from nfd.conf and point the system-wide
# ndn-cxx client.conf at it: the packaged client.conf can lag behind the
# nfd.conf socket location, which breaks nfd-status and every other local
# NDN client.
sock="$(awk '$1=="path"{gsub(/;/,"",$2); print $2; exit}' "$conf" 2>/dev/null)"
if [ -z "$sock" ]; then
    sock=/run/nfd/nfd.sock
fi
mkdir -p "$(dirname "$sock")" /run/nfd /etc/ndn
printf 'transport=unix://%s\n' "$sock" > /etc/ndn/client.conf
# The deb config drops privileges to the nfd user, whose keychain lives
# under $HOME/.ndn; systemd normally sets HOME for the service. Point HOME
# at the nfd user's writable home instead of inheriting /root.
nfd_home="$(getent passwd nfd 2>/dev/null | cut -d: -f6)"
if [ -z "$nfd_home" ]; then
    nfd_home=/var/lib/nfd
fi
mkdir -p "$nfd_home"
chown -R nfd:nfd "$nfd_home" 2>/dev/null || true
HOME="$nfd_home" nohup nfd --config "$conf" >/var/log/nfd.log 2>&1 &
for _ in $(seq 1 50); do
    if [ -S "$sock" ] && nfd-status >/dev/null 2>&1; then
        exit 0
    fi
    sleep 0.2
done
echo "nfd-start shim: nfd did not become ready" >&2
echo "nfd-start shim: socket=$sock exists=$([ -S \"$sock\" ] && echo yes || echo no)" >&2
echo "nfd-start shim: nfd-status says:" >&2
nfd-status 2>&1 | head -n 5 >&2 || true
tail -n 50 /var/log/nfd.log >&2 || true
exit 1
EOF
COPY <<'EOF' /usr/local/bin/nfd-stop
#!/usr/bin/env bash
pkill -x nfd 2>/dev/null || true
for _ in $(seq 1 25); do
    if ! pgrep -x nfd >/dev/null 2>&1; then
        exit 0
    fi
    sleep 0.2
done
pkill -9 -x nfd 2>/dev/null || true
exit 0
EOF
RUN chmod +x /usr/local/bin/nfd-start /usr/local/bin/nfd-stop \
    # The PPA nfd binary carries file capabilities (cap_net_admin for
    # ethernet/multicast faces); Docker's default bounding set lacks
    # NET_ADMIN, which makes execve fail with EPERM. The local stack only
    # uses the unix socket, so the file caps are stripped.
    && if command -v setcap >/dev/null 2>&1; then \
         setcap -r /usr/bin/nfd 2>/dev/null || true; \
         setcap -r /usr/local/bin/nfd 2>/dev/null || true; \
       fi

ENV NDNSF_ROOT=/opt/NDN_Service_Framework \
    UAS_IPBRC_ROOT=/work/UAS-IPBRC

WORKDIR /work/miniMUAS
CMD ["bash"]
