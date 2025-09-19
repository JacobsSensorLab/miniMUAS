let
  pkgs = import <nixpkgs> { overlays = [(import ./deps/overlay.nix)]; };
  python = pkgs.python313;
  pythonPackages = python.pkgs;
  lib-path = with pkgs; lib.makeLibraryPath [
    stdenv.cc.cc
    ndnsf
    ndn-cxx
    ndn-svs
    ndnsd
    nac-abe
    openabe
    mavsdk
    opencv
  ];
  # mcc-env = (pkgs.callPackage /home/pmle/mini_compile_commands {}).wrap pkgs.clangStdenv;

in with pkgs; mkShell.override {} {

  packages = [
    pkg-config
    python
    pythonPackages.jinja2
    pythonPackages.pyyaml
    waf
    opencv
    cmake
    clang-tools
    ndnsf
    ndn-cxx
    ndn-svs
    ndnsd
    nac-abe
    openabe
    mavsdk
    opencv
    gcc
  ];

  nativeBuildInputs = [
    gcc
    cmake
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

    ndn-cxx
    ndn-svs
    ndnsd
    nac-abe
    openabe
    ndnsf

    mavsdk
    tinyxml2
    curl
    jsoncpp
  ];

  buildInputs = [
    gcc
    cmake
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

    ndn-cxx
    ndn-svs
    ndnsd
    nac-abe
    openabe
    ndnsf

    mavsdk
    tinyxml2
    curl
    jsoncpp
  ];

  shellHook = ''
    export "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${lib-path}"
    SOURCE_DATE_EPOCH=$(date +%s)
    export CC=clang
    export CXX=clang++
    export CXXFLAGS="-stdlib=libc++"
  '';
}

# /home/pmle/.config/Code/User/globalStorage/llvm-vs-code-extensions.vscode-clangd/install/20.1.8/clangd_20.1.8/bin/clangd