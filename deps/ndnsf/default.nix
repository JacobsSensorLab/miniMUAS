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
  version = "2025-08-12";

  src = fetchFromGitHub {
    owner = "matianxing1992";
    repo = "NDN_Service_Framework";
    rev = "90b8fbe1487ee1297b0056dc544da963905c731e";
    sha256 = "sha256-M49+kK3cTirQfKAe5pvmQXTw3kiYKIs1if2mW+pqkN4=";
  };

  patches = [
    ./NDNSF_90b8fbe.patch
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
