{ lib
, stdenv
, fetchFromGitHub
, fetchurl
, pkg-config
, perl
, cmake
, boost
, glib
, gmp
, bison
, flex
, nodejs_24
, python313
, unzip
, gcc11
, buildPackages
}:

stdenv.mkDerivation rec {
  pname = "openabe";
  version = "1.0.0";
  gtestVersion = "1.8.0";

  src = fetchFromGitHub {
    owner = "zeutro";
    repo = "openabe";
    rev = "b8f9d3c8a2620c1185ca972248f7af39c1eae68c";
    sha256 = "sha256-3SBLu2S128PFHlfYG704fe76AN5gZuEemmNInnS9HRk=";
  };

  nativeBuildInputs = [ pkg-config perl cmake boost glib gmp unzip gcc11 bison flex nodejs_24 python313 ];

  dontUseCmakeConfigure = true;

  postPatch = let
    relicSrc = fetchFromGitHub {
      owner = "relic-toolkit";
      repo = "relic";
      rev = "4e70b5f43cd25ed60bb42b24d6091707c7e4489b";
      hash = "sha256-i+6AECSo4wWf7JNpoEvX9RtesSlG183aeEY4GnUcCD4=";
      postFetch = ''
        substituteInPlace $out/src/md/blake2.h \
        --replace-fail 'ALIGNME( 64 ) typedef struct __blake2s_state' 'typedef struct __blake2s_state'

        substituteInPlace $out/src/md/blake2.h \
        --replace-fail 'ALIGNME( 64 ) typedef struct __blake2b_state' 'typedef struct __blake2b_state'
        
        substituteInPlace $out/src/md/blake2s-ref.c \
        --replace-fail 'blake2s_state S[1];' 'blake2s_state S;'
        
        substituteInPlace $out/src/md/blake2s-ref.c \
        --replace-fail 'if( blake2s_init_key( S, outlen, key, keylen ) < 0 ) return -1;' 'if( blake2s_init_key( &S, outlen, key, keylen ) < 0 ) return -1;'

        substituteInPlace $out/src/md/blake2s-ref.c \
        --replace-fail 'if( blake2s_init( S, outlen ) < 0 ) return -1;' 'if( blake2s_init( &S, outlen ) < 0 ) return -1;'

        substituteInPlace $out/src/md/blake2s-ref.c \
        --replace-fail 'blake2s_update( S, ( const uint8_t * )in, inlen );' 'blake2s_update( &S, ( const uint8_t * )in, inlen );'
        
        substituteInPlace $out/src/md/blake2s-ref.c \
        --replace-fail 'blake2s_final( S, out, outlen );' 'blake2s_final( &S, out, outlen );'
      '';
    };
    opensslSrc = fetchFromGitHub {
      owner = "openssl";
      repo = "openssl";
      rev = "560096f804a3712eea161726a8f085beefe8838a";
      hash = "sha256-JxJDvEii8gOp9WqKMYuppOraJQT+6AGApxHX5f3zsPY=";
      postFetch = ''
        patchShebangs $out/Configure
      '';
    };
    gtestSrc = fetchurl {
      version = "${gtestVersion}";
      url = "https://github.com/google/googletest/archive/refs/tags/release-${gtestVersion}.zip";
      hash = "sha256-8+07WFEe/ScusHSjptb7edfC5qDjdDI9HmvLzB7xQb8=";
    };
    string1 = lib.strings.concatLines[ "#include <openabe/utils/zerror.h>" "#include <cstdint>" ];
    string2 = lib.strings.concatLines[ "#define __ZCRYPTOUTILS_H__" "#include <cstdint>" ];
    string3 = lib.strings.concatLines[ "#include <iostream>" "#include <cstdint>" ];
  in ''
    patchShebangs deps/*/*.sh

    patchShebangs cli/*.sh

    cp -r ${opensslSrc} ./openssl_tmp
    
    substituteInPlace openssl_tmp/config \
      --replace-fail '/usr/bin/env' '${buildPackages.coreutils}/bin/env'

    substituteInPlace Makefile.common \
      --replace-fail 'OPENSSL_CONFIG = config shared' 'OPENSSL_CONFIG = config'

    substituteInPlace examples/Makefile \
      --replace-fail 'LIBS = ' 'LIBS = -lssl '

    substituteInPlace deps/relic/download_relic.sh \
      --replace-fail 'git ' '# git'

    substituteInPlace deps/openssl/download_openssl.sh \
      --replace-fail 'git ' '# git '

    substituteInPlace deps/gtest/download_gtest.sh \
      --replace-fail 'wget ' 'pwd # wget'

    substituteInPlace src/include/openabe/zml/zelement.h \
      --replace-fail ' CMP_EQ' ' RLC_EQ' \
      --replace-fail ' CMP_LT' ' RLC_LT' \
      --replace-fail ' CMP_GT' ' RLC_GT' \
      --replace-fail ' CAT(' ' RLC_CAT(' \
      --replace-fail ' BN_NEGATIVE                   BN_NEG' ' BN_NEGATIVE                   RLC_NEG' \
      --replace-fail ' BN_POSITIVE                   BN_POS' ' BN_POSITIVE                   RLC_POS'
    
    substituteInPlace src/zml/zelement.c \
      --replace-fail ' CMP_EQ' ' RLC_EQ' \
      --replace-fail ' CMP_LT' ' RLC_LT' \
      --replace-fail ' CMP_GT' ' RLC_GT' \
      --replace-fail ' CMP_NE' ' RLC_NE' \
      --replace-fail ' BN_NEG' ' RLC_NEG' \
      --replace-fail ' BN_POS' ' RLC_POS' \
      --replace-fail ' STS_OK' ' RLC_OK' \
      --replace-fail ' FP_DIGS' ' RLC_FP_DIGS' \
      --replace-fail '(FP_DIGS ' '(RLC_FP_DIGS '
      
    substituteInPlace src/zml/zelement_bp.cpp \
      --replace-fail ' CMP_EQ' ' RLC_EQ' \
      --replace-fail ' CMP_LT' ' RLC_LT' \
      --replace-fail ' CMP_GT' ' RLC_GT' \

    substituteInPlace src/include/openabe/zobject.h \
      --replace-fail '#include <openabe/utils/zerror.h>' '${string1}'

    substituteInPlace src/include/openabe/utils/zcryptoutils.h \
      --replace-fail '#define __ZCRYPTOUTILS_H__' '${string2}'
      
    substituteInPlace src/include/openabe/utils/zbytestring.h \
      --replace-fail '#include <iostream>' '${string3}'
            
    mkdir -p deps/relic/relic-toolkit-0.5.0.git
    tar czf deps/relic/relic-toolkit-0.5.0.test.tar.gz --mode 'u+w' -C ${relicSrc} .

    mkdir -p deps/openssl/openssl-1.1.1-dev.git
    tar czf deps/openssl/openssl-1.1.1-dev.test.tar.gz --mode 'u+w' -C ./openssl_tmp .

    cp ${gtestSrc} deps/gtest/googletest-release-${gtestVersion}.zip
  '';

  preBuild = ''
     # emulate env file
     ZROOT=$(pwd)
     DEPS_INSTALL_ZROOT=''${ZROOT}/deps/root
     OABE_LIBS_ROOT=''${ZROOT}/root
     BISON=${bison}/bin/bison
     INSTALL_PREFIX=$out

     export ZROOT DEPS_INSTALL_ZROOT OABE_LIBS_ROOT BISON INSTALL_PREFIX
  '';

  buildInputs = [ ];
}
