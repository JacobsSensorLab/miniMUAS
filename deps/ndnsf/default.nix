{ lib
, stdenv
, fetchFromGitHub
, pkg-config
, python313
, wafHook
, ndn-cxx
, ndn-svs
, boost
, openssl
, sqlite
, nac-abe
, ndnsd
, protobuf_21
, mavsdk
, gtkmm3
}:

stdenv.mkDerivation rec {
  pname = "NDNSF";
  version = "2025-03-04";

  src = fetchFromGitHub {
    owner = "matianxing1992";
    repo = "NDN_Service_Framework";
    rev = "1033c9c910fc9f256407e2740c593f4d562c6393";
    sha256 = "sha256-9sHffACf4qnGRdYxcgt5kwwEh49c16kBQED2Oj1vqzU=";
  };

  patches = [
    ./NDNSF.patch
  ];

  nativeBuildInputs = [ pkg-config python313 wafHook ];

  buildInputs = [ boost openssl ndn-cxx ndn-svs sqlite nac-abe ndnsd protobuf_21 mavsdk gtkmm3 ];

  wafConfigureFlags = [
    "--boost-includes=${boost.dev}/include"
    "--boost-libs=${boost.out}/lib"
  ];

  meta = with lib; {
    homepage = "https://github.com/matianxing1992/NDN_Service_Framework";
    description = "A secured service framework for Named Data Networking";
    longDescription = ''
      A secured service framework for Named Data Networking 
    '';
    license = licenses.lgpl3;
    platforms = platforms.unix;
    maintainers = with maintainers; [ matianxing1992 ];
  };
}
