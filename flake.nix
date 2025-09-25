{
  description = "Framework for multi drone control on small UAS testbed";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
  };

  outputs = { self, nixpkgs }: let
    inputs = { inherit nixpkgs; };
    system = "x86_64-linux";

    pkgs = import nixpkgs {
      inherit system;
      config = { allowUnfree = true; };
      overlays = [
        (import ./deps/overlay.nix)
      ];
    };

    commonInputs = with pkgs; [
      boost
      openssl
      ndn-cxx
      ndn-svs
      sqlite
      nac-abe
      ndnsd
      ndnsf
      protobuf_21
      mavsdk
      gtkmm3
      opencv
    ];

    # a text file containing the paths to the flake inputs in order to stop
    # them from being garbage collected
    pleaseKeepMyInputs = pkgs.writeTextDir "bin/.please-keep-my-inputs"
      (builtins.concatStringsSep " " (builtins.attrValues inputs));
  in {
    packages.x86_64-linux.minimuas = pkgs.stdenv.mkDerivation {
      pname = "minimuas";
      version = "2025-09-15";

      src = ./.;

      nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];

      buildInputs = commonInputs;

      installPhase = ''
        mkdir -p $out/bin
        mkdir -p $out/sec
        mkdir -p $out/config

        cp iuas $out/bin/
        cp wuas $out/bin/
        cp gcs $out/bin/
        cp gcs-shell $out/bin/
        cp service-controller $out/bin/

        # ndnsec needs $HOME to maintain its database
        export HOME=$(mktemp -d)

        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas > $out/muas.key
        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas/aa > $out/aa.key
        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas/gcs-01 > $out/gcs-01.key
        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas/wuas-01 > $out/wuas-01.key
        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas/iuas-01 > $out/iuas-01.key

        ${pkgs.ndn-cxx}/bin/ndnsec cert-dump -i /muas > $out/sec/muas.cert
        ${pkgs.ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/aa.key > $out/sec/aa.cert
        ${pkgs.ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/gcs-01.key > $out/sec/gcs-01.cert
        ${pkgs.ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/wuas-01.key > $out/sec/wuas-01.cert
        ${pkgs.ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/iuas-01.key > $out/sec/iuas-01.cert

        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/muas.ndnkey -i /muas
        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/aa.ndnkey -i /muas/aa
        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/gcs-01.ndnkey -i /muas/gcs-01
        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/wuas-01.ndnkey -i /muas/wuas-01
        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/iuas-01.ndnkey -i /muas/iuas-01

        cp ../config/minimuas.policies $out/config/
        cp ../config/trust-any.conf $out/config/
        cp ../config/trust-schema.conf $out/config/

        # cleanup
        rm $out/muas.key
        rm $out/aa.key
        rm $out/gcs-01.key
        rm $out/wuas-01.key
        rm $out/iuas-01.key
      '';

      meta = with pkgs.lib; {
        description = "miniMUAS applications using the ndn secure framework";
        homepage = https://github.com/JacobsSensorLab/miniMUAS-Apps;
      };
    };

    packages.x86_64-linux.minimuas-nix = pkgs.stdenv.mkDerivation {
      pname = "minimuas";
      version = "2025-09-15";

      src = ./.;

      patches = [
        ( pkgs.replaceVars ./patches/ndnget_path.patch { ndn-tools = pkgs.ndn-tools; } )
        ( pkgs.replaceVars ./patches/config_dir.patch { minimuas-files = pkgs.minimuas-files; } )
      ];

      nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];

      buildInputs = commonInputs;

      installPhase = ''
        mkdir -p $out/bin
        mkdir -p $out/sec
        mkdir -p $out/config

        cp iuas $out/bin/
        cp wuas $out/bin/
        cp gcs $out/bin/
        cp gcs-shell $out/bin/
        cp service-controller $out/bin/

        # ndnsec needs $HOME to maintain its database
        export HOME=$(mktemp -d)

        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas > $out/muas.key
        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas/aa > $out/aa.key
        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas/gcs-01 > $out/gcs-01.key
        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas/wuas-01 > $out/wuas-01.key
        ${pkgs.ndn-cxx}/bin/ndnsec key-gen -t r /muas/iuas-01 > $out/iuas-01.key

        ${pkgs.ndn-cxx}/bin/ndnsec cert-dump -i /muas > $out/sec/muas.cert
        ${pkgs.ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/aa.key > $out/sec/aa.cert
        ${pkgs.ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/gcs-01.key > $out/sec/gcs-01.cert
        ${pkgs.ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/wuas-01.key > $out/sec/wuas-01.cert
        ${pkgs.ndn-cxx}/bin/ndnsec cert-gen -s /muas -i default $out/iuas-01.key > $out/sec/iuas-01.cert

        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/muas.ndnkey -i /muas
        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/aa.ndnkey -i /muas/aa
        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/gcs-01.ndnkey -i /muas/gcs-01
        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/wuas-01.ndnkey -i /muas/wuas-01
        ${pkgs.ndn-cxx}/bin/ndnsec-export -P 123456 -o $out/sec/iuas-01.ndnkey -i /muas/iuas-01

        cp ../config/minimuas.policies $out/config/
        cp ../config/trust-any.conf $out/config/
        cp ../config/trust-schema.conf $out/config/

        # cleanup
        rm $out/muas.key
        rm $out/aa.key
        rm $out/gcs-01.key
        rm $out/wuas-01.key
        rm $out/iuas-01.key
      '';

      meta = with pkgs.lib; {
        description = "miniMUAS applications using the ndn secure framework";
        homepage = https://github.com/JacobsSensorLab/miniMUAS-Apps;
      };
    };

    packages.x86_64-linux.default = self.packages.x86_64-linux.minimuas;

    devShell."${system}" = pkgs.mkShell {
      buildInputs = [
        pkgs.ndn-cxx
        pkgs.ndn-tools
        pkgs.nfd
        pkgs.psync
        pkgs.nlsr

        pleaseKeepMyInputs
      ];
    };


  };
}