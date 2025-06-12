{ lib
, stdenv
, fetchFromGitHub
, pkg-config
, cmake
, python311
}:

let
  python = python311.pkgs;
  lxml = python.lxml;
  future = python.future;
  wheel = python.wheel;
  setuptools = python.setuptools;
  fastcrc = python.fastcrc;
  pymavlink = python.pymavlink;
  python3 = python311.buildEnv.override {
    extraLibs = [
        lxml
        future
        wheel
        setuptools
        fastcrc
        pymavlink
    ];
    ignoreCollisions = true;
  };

in
stdenv.mkDerivation rec {
  name = "mavlink";
  version = "5e3a42b8f3f53038f2779f9f69bd64767b913bb8";

  src = fetchFromGitHub {
    owner = "mavlink";
    repo = "mavlink";
    rev = "5e3a42b8f3f53038f2779f9f69bd64767b913bb8";
    sha256 = "sha256-bSnig/WrzL5HYgyd1hCHtoPs9JvD/mz2tN+WNS3+yqw=";
    fetchSubmodules = true;
  };

  cmakeFlags = [
    "-DCMAKE_INSTALL_PREFIX=install"
  ];

  buildInputs = [ python3 ];

  propagatedBuildInputs = [ lxml future wheel setuptools fastcrc pymavlink ];

  nativeBuildInputs = [ pkg-config cmake ];

  enableParallelBuilding = true;

  patches = [
    ./mavlink.patch
    ./mavlink_no_pip.patch
  ];

  preConfigure = ''
  '';

  # installPhase = "install -Dm755  $out/bin/mavlink/install";

  meta = with lib; {
    description = "Marshalling / communication library for drones.";
    homepage    = https://github.com/mavlink/mavlink;
    license     = licenses.gpl3;
    inherit version;
  };
}
