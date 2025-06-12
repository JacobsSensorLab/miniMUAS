{ lib
, stdenv
, fetchFromGitHub
, pkg-config
, cmake
, buildPackages
, git
, curl
, xz
, mavlink
, jsoncpp
, tinyxml2
}:

let
  libevents = fetchFromGitHub {
    owner = "mavlink";
    repo = "libevents";
    rev = "7c1720749dfe555ec2e71d5f9f753e6ac1244e1c";
    sha256 = "sha256-qzY2FOgc+iD5bYXJGj6ftWUA3cEC14Dwz0ZFr7WCtro=";
  };
  
in
stdenv.mkDerivation rec {
  name = "mavsdk";
  version = "3.2.0";

  src = fetchFromGitHub {
    owner = "mavlink";
    repo = "MAVSDK";
    rev = "c4a070f974b3d91375f096095d952cb8ef416849";
    sha256 = "sha256-bMg7FPH78k2PDnJA/pGLmkFy7rh14uHACzF0pzK11qM=";
    fetchSubmodules = true;
  };

  # Removed "-DCMAKE_PREFIX_PATH=${mavlink}/install"
  cmakeFlags = [ 
    "-DSUPERBUILD=OFF"
    "-DTINYXML2_LIBRARY=${tinyxml2}"
  ];

  buildInputs = [ mavlink curl xz tinyxml2 jsoncpp ];

  nativeBuildInputs = [ pkg-config cmake git ];

  enableParallelBuilding = true;

  patches = [
    ./no-download.patch
  ];

  preConfigure = ''
    mkdir -p /build/source/build/third_party/libevents/libevents-prefix/src
    cp -r ${libevents}/* /build/source/build/third_party/libevents/libevents-prefix/src
    chmod -R +w /build/source/build/third_party/libevents/libevents-prefix/src
  '';

  meta = with lib; {
    description = "API and library for MAVLink compatible systems written in C++17";
    homepage    = https://github.com/mavlink/MAVSDK;
    license     = licenses.bsd3;
    inherit version;
  };
}
