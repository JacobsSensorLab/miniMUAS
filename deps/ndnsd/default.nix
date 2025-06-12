{ lib
, stdenv
, fetchFromGitHub
, pkg-config
, python313
, wafHook
, boost
, openssl
, ndn-cxx
, ndn-svs
}:

stdenv.mkDerivation rec {
  pname = "ndnsd";
  version = "2025-03-04";

  src = fetchFromGitHub {
    owner = "matianxing1992";
    repo = "ndnsd";
    rev = "b0e2c9b95e2b20688d58ef007dd3f298892fc696";
    sha256 = "sha256-AgRV4XeaRBVznOgmm+0byBomHvvoKbl0jWUTfmiTZRc=";
  };

  wafConfigureFlags = [
    "--boost-includes=${boost.dev}/include"
    "--boost-libs=${boost.out}/lib"
  ];


  nativeBuildInputs = [ pkg-config python313 wafHook ];

  buildInputs = [ boost openssl ndn-svs ndn-cxx ];
}
