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
    mkdir -p $out/bin
    cp iuas $out/bin/
    cp wuas $out/bin/
    cp gcs $out/bin/
    cp gcs_shell $out/bin/
    cp ndnsf_controller $out/bin/
  '';

  meta = with lib; {
    description = "miniMUAS applications using the ndn secure framework";
    homepage = https://github.com/JacobsSensorLab/miniMUAS-Apps;
  };
}
