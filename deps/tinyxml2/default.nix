{ lib
, stdenv
, fetchFromGitHub
, pkg-config
, cmake
, buildPackages
, git
}:

stdenv.mkDerivation rec {
  name = "tinyxml2";
  version = "9.0.0";

  src = fetchFromGitHub {
    owner = "leethomason";
    repo = "tinyxml2";
    rev = "9.0.0";
    sha256 = "sha256-AQQOctXi7sWIH/VOeSUClX6hlm1raEQUOp+VoPjLM14=";
  };

  buildInputs = [ ];

  nativeBuildInputs = [ pkg-config cmake git ];

  patches = [
    # ./cmake-3.10.2.patch
    # ./no-lfs64.patch
  ];

  postPatch = ''
    ls
    substituteInPlace cmake/tinyxml2.pc.in \
      --replace-fail '/@CMAKE_INSTALL_LIBDIR@' '@CMAKE_INSTALL_LIBDIR@'
    substituteInPlace cmake/tinyxml2.pc.in \
      --replace-fail '/@CMAKE_INSTALL_INCLUDEDIR@' '@CMAKE_INSTALL_INCLUDEDIR@'
    '';

  meta = with lib; {
    description = "TinyXML2 is a simple, small, efficient, C++ XML parser that can be easily integrated into other programs.";
    homepage    = https://github.com/leethomason/tinyxml2;
    license     = licenses.zlib;
    inherit version;
  };
}
