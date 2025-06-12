{ lib
, stdenv
, fetchFromGitHub
, pkg-config
, cmake
, boost
, openssl
, ndn-cxx
, openabe
, gmpxx
}:

stdenv.mkDerivation rec {
  pname = "NAC-ABE";
  version = "2024-09-27";

  src = fetchFromGitHub {
    owner = "matianxing1992";
    repo = "NAC-ABE";
    rev = "9804ab17b5fa5d2742e8fccfce02986fbcba1dd0";
    sha256 = "sha256-O7nB88Ekvo4+ZXm1HupKRDjgjs5PrYHjaGUvEa5P7Q4=";
  };

  cmakeFlags = [
    "-DCMAKE_CXX_FLAGS=-DNAC_ABE_CMAKE_BUILD"
  ];

  patches = [
    ./NAC-ABE.patch
  ];

  nativeBuildInputs = [ cmake pkg-config openabe gmpxx];

  buildInputs = [ boost openssl ndn-cxx ];
}
