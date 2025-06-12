let
  pkgs = import <nixpkgs> { overlays = [(import ./deps/overlay.nix)]; };
  python = pkgs.python313;
  pythonPackages = python.pkgs;
  lib-path = with pkgs; lib.makeLibraryPath [
    stdenv.cc.cc
    pkgs.ndnsf
  ];
in with pkgs; mkShell {

  packages = [
    pkg-config
    python
    pythonPackages.jinja2
    pythonPackages.pyyaml
    waf
    opencv
  ];

  buildInputs = [
    git
    openssl
    boost
    sqlite
    protobuf_21

    gtkmm3
    libsysprof-capture
    pcre2
    xorg.libXdmcp
    libthai
    lerc
    libdatrie
    util-linux
    libsepol
    xorg.libXtst
    libxkbcommon
    libselinux
    libepoxy

    opencv

    pkgs.ndn-cxx
    pkgs.ndn-svs
    pkgs.ndnsd
    pkgs.nac-abe
    pkgs.openabe
    pkgs.ndnsf

    pkgs.mavsdk
    pkgs.tinyxml2
    curl
    jsoncpp
  ];

  shellHook = ''
    SOURCE_DATE_EPOCH=$(date +%s)
    export "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${lib-path}"
  '';
}
