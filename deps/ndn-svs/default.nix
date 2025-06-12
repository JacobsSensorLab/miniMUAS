{ lib
, stdenv
, fetchFromGitHub
, pkg-config
, python313
, wafHook
, ndn-cxx
, boost
, openssl
}:

stdenv.mkDerivation rec {
  pname = "ndn-svs";
  version = "12_11_2024";

  src = fetchFromGitHub {
    owner = "matianxing1992";
    repo = "ndn-svs";
    rev = "2ac6180ab56b566506c03554b8f2ba1b6cea25fd";
    sha256 = "sha256-dzGPO0bvpApOe8anugn4zWoPOT0gP9GHk8De9e2DEy4=";
  };

  nativeBuildInputs = [ pkg-config python313 wafHook ];

  buildInputs = [ boost openssl ndn-cxx ];

  wafConfigureFlags = [
    "--boost-includes=${boost.dev}/include"
    "--boost-libs=${boost.out}/lib"
    "--with-tests"
  ];

  doCheck = true;
  checkPhase = ''
    runHook preCheck
    LD_PRELOAD=build/libndn-svs.so HOME=. build/unit-tests
    runHook postCheck
  '';

  meta = with lib; {
    homepage = "https://github.com/matianxing1992/ndn-svs";
    description = "State Vector Sync library for distributed realtime applications for NDN";
    longDescription = ''
      This library provides an implementation of the State Vector Sync (SVS) protocol and the Pub/Sub API for state synchronization between multiple clients over NDN.
    '';
    license = licenses.lgpl3;
    platforms = platforms.unix;
    maintainers = with maintainers; [ matianxing1992 ];
  };
}
