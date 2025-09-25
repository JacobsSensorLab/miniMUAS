{ lib
, stdenv
, fetchFromGitHub
, ndn-cxx
}:

stdenv.mkDerivation rec {
  pname = "minimuas-files";
  version = "2025-09-15";

  src = ../../.;

  buildInputs = [ ndn-cxx ];

  installPhase = ''
    mkdir -p $out/sec
    mkdir -p $out/config

    # ndnsec needs $HOME to maintain its database
    export HOME=$(mktemp -d)

    ${ndn-cxx}/bin/ndnsec key-gen -t r /muas > $out/muas.key
    ${ndn-cxx}/bin/ndnsec key-gen -t r /muas/aa > $out/aa.key
    ${ndn-cxx}/bin/ndnsec key-gen -t r /muas/gcs-01 > $out/gcs-01.key
    ${ndn-cxx}/bin/ndnsec key-gen -t r /muas/wuas-01 > $out/wuas-01.key
    ${ndn-cxx}/bin/ndnsec key-gen -t r /muas/iuas-01 > $out/iuas-01.key

    ${ndn-cxx}/bin/ndnsec cert-dump -i /muas > $out/sec/muas.cert
    ${ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/aa.key > $out/sec/aa.cert
    ${ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/gcs-01.key > $out/sec/gcs-01.cert
    ${ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/wuas-01.key > $out/sec/wuas-01.cert
    ${ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/iuas-01.key > $out/sec/iuas-01.cert

    ${ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/muas.ndnkey -i /muas
    ${ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/aa.ndnkey -i /muas/aa
    ${ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/gcs-01.ndnkey -i /muas/gcs-01
    ${ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/wuas-01.ndnkey -i /muas/wuas-01
    ${ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/iuas-01.ndnkey -i /muas/iuas-01

    cp config/minimuas.policies $out/config/
    cp config/trust-any.conf $out/config/
    cp config/trust-schema.conf $out/config/

    # cleanup
    rm $out/muas.key
    rm $out/aa.key
    rm $out/gcs-01.key
    rm $out/wuas-01.key
    rm $out/iuas-01.key
  '';

  meta = with lib; {
    description = "miniMUAS config and certificates";
    homepage = https://github.com/JacobsSensorLab/miniMUAS-Apps;
  };
}