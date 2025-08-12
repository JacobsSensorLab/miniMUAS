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
    owner = "UCLA-IRL"; # forked by matianxing1992
    repo = "NAC-ABE";
    rev = "5ac3eb991d6ed7eef36e6a265e97912961e9807f";
    sha256 = "sha256-/0zUFYe/EFDdeL6ZwOHw2x/zPbn7OPN5O7E9c1tXnzs=";
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
