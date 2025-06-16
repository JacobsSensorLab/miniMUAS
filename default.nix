let
  pkgs = import <nixpkgs> { overlays = [(import ./deps/overlay.nix)]; };

  stdenv = pkgs.stdenv;
  lib = pkgs.lib;
  src = pkgs.src;
  pkg-config = pkgs.pkg-config;
  wafHook = pkgs.wafHook;
  boost = pkgs.boost;
  openssl = pkgs.openssl;
  ndn-cxx = pkgs.ndn-cxx;
  ndn-svs = pkgs.ndn-svs;
  nac-abe = pkgs.nac-abe;
  ndnsd = pkgs.ndnsd;
  mavsdk = pkgs.mavsdk;
  sqlite = pkgs.sqlite;
  protobuf = pkgs.protobuf_21;
  gtkmm3 = pkgs.gtkmm3;
  cv = pkgs.opencv;
  ndnsf = pkgs.ndnsf;
  cmake = pkgs.cmake;

  # lib-path = with pkgs; lib.makeLibraryPath [
  #   stdenv.cc.cc
  # ];
in stdenv.mkDerivation {
  pname = "minimuas-apps";
  version = "2025-06-11";

  src = ./.;

  nativeBuildInputs = [ cmake pkg-config ];

  buildInputs = [ boost openssl ndn-cxx ndn-svs sqlite nac-abe ndnsd ndnsf protobuf mavsdk gtkmm3 cv ];

  installPhase = ''
    mkdir -p $out/sec

    ndnsec delete /muas
    ndnsec delete /muas/aa
    ndnsec delete /muas/gcs-01
    ndnsec delete /muas/wuas-01
    ndnsec delete /muas/iuas-01

    ndnsec key-gen -t r /muas > /tmp/muas.key
    ndnsec key-gen -t r /muas/aa > /tmp/aa.key
    ndnsec key-gen -t r /muas/gcs-01 > /tmp/gcs-01.key
    ndnsec key-gen -t r /muas/wuas-01 > /tmp/wuas-01.key
    ndnsec key-gen -t r /muas/iuas-01 > /tmp/iuas-01.key

    ndnsec cert-dump -i /muas > $out/sec/muas.cert
    ndnsec cert-gen -s /muas -i default /tmp/aa.key > $out/sec/aa.cert
    ndnsec cert-gen -s /fossn -i default /tmp/gcs-01.key > $out/sec/gcs-01.cert
    ndnsec cert-gen -s /fossn -i default /tmp/sys-01.key > $out/sec/sys-01.cert
    ndnsec cert-gen -s /fossn -i default /tmp/sys-01.key > $out/sec/sys-02.cert

    ndnsec-export -P 123456 -o $out/sec/muas.ndnkey -i /muas
    ndnsec-export -P 123456 -o $out/sec/aa.ndnkey -i /muas/aa
    ndnsec-export -P 123456 -o $out/sec/gcs-01.ndnkey -i /muas/gcs-01
    ndnsec-export -P 123456 -o $out/sec/wuas-01.ndnkey -i /muas/wuas-01
    ndnsec-export -P 123456 -o $out/sec/iuas-01.ndnkey -i /muas/iuas-01

    mkdir -p $out/bin
    cp iuas $out/bin/
    cp wuas $out/bin/
    cp gcs $out/bin/
    cp gcs-shell $out/bin/
    cp service-controller $out/bin/

    mkdir -p $out/config
    cp config/minimuas.policies $out/config/
    cp config/trust-any.conf $out/config/
    cp config/trust-schema.conf $out/config/
  '';

  meta = with lib; {
    description = "miniMUAS applications using the ndn secure framework";
    homepage = https://github.com/JacobsSensorLab/miniMUAS-Apps;
  };
}
