{
  description = ''
  '';

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    pndriver-lib = {
      type = "gitlab";
      owner = "esab";
      repo = "abw%2Fpndriver";
      ref = "main";
      rev = "84e789ba7b664108d701befed18c7377aa83f9f5";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    flake-utils.url = "github:numtide/flake-utils";
    infra-utils = {
      type = "gitlab";
      owner = "esab";
      repo = "abw%2Finfra-and-test";
      ref = "main";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    pn-interface = {
      type = "gitlab";
      owner = "esab";
      repo = "abw%2Fplc%2Fpn-interface";
      ref = "main";
      rev = "b6e0d58661e11cde075504eb0edd9e4efdf36bfb";
    };
    pylon-software = {
      type = "gitlab";
      owner = "esab";
      repo = "abw%2Fpylon-software";
      ref = "main";
      rev = "fde3d1f55ecce1b5e30847079da294233b921cf0";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, pndriver-lib, infra-utils, pn-interface, pylon-software, ... }:
    flake-utils.lib.eachSystem [ flake-utils.lib.system.x86_64-linux ] (system:
    let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [
          (final: prev: {
            opencv = (prev.opencv.override {
               enableGtk2 = true;
            }).overrideAttrs {
              patches = prev.opencv.patches ++ [
                ./fix-opencv.patch
              ];
            };

            pn-interface = pn-interface.packages.${final.system}.default;

            # Override ceres-solver to disable SuiteSparse, CXSparse and LAPACK
            #
            # Merely linking against libceres.so (with above packages enabled) causes
            # thread creation (pthread_create) to fail inside civetweb (mg_start call).
            # Disabling these packages removes several library dependencies including
            # a direct libpthread dependency from LAPACK.
            #
            # Note: CivetWeb is linked indirectly via prometheus-cpp-pull.so,
            # so this issue occurs even if the application doesn't link CivetWeb directly.
            ceres-solver = prev.ceres-solver.overrideAttrs (oldAttrs: {
              cmakeFlags = (oldAttrs.cmakeFlags or []) ++ [
                "-DSUITESPARSE=OFF"
                "-DCXSPARSE=OFF"
                "-DLAPACK=OFF"
              ];
            });
          })
        ];
        config.allowUnfree = true;
        config.permittedInsecurePackages = [
        ];
      };

      pndriver = pndriver-lib.defaultPackage.${system};

      pylon = pylon-software.defaultPackage.${system};

    in rec
    {
      ################################
      ## Package derivations
      ################################

      packages.default = packages.adaptio;

      packages = {
        pre-commit = infra-utils.packages.${system}.pre-commit;
        gitlint = infra-utils.packages.${system}.gitlint;
        gittemplate = infra-utils.packages.${system}.gittemplate;
      };

      packages.adaptio = let
        src = ./.;
        python-packages = ps: with ps; [
          jinja2
          pytest
          pyyaml
          pyzmq
          dataclasses-json
          setuptools
          numpy
          matplotlib
          websocket-client
          pandas
        ];
        revShort = if (self ? rev) then self.shortRev else if (self ? dirtyRev) then self.dirtyShortRev else "dirty-inputs";
      in pkgs.llvmPackages_19.stdenv.mkDerivation {
        pname = "adaptio";
        version = pkgs.lib.strings.fileContents "${src}/version.txt";
        inherit src;

        separateDebugInfo = true;

        PnDriver_ROOT="${pndriver}";

        Pylon_ROOT = "${pylon}";

        cmakeFlags = [ "-DGIT_REV_SHORT=${revShort}" ];

        ninjaFlags = [ "adaptio" ];

        buildInputs = with pkgs; [
          # Utilities
          boost181
          fmt
          nlohmann_json

          # Infrastructure
          cppzmq
          openssl

          # Data collection
          prometheus-cpp

          # Persistent storage
          yaml-cpp
          sqlite
          sqlitecpp

          # Hardware interfaces
          pndriver
          pylon

          # Testing
          doctest # Unit tests
          trompeloeil # Mocking library

          # Maths
          eigen
          opencv
          ceres-solver
          pcl # point cloud library (ransac)

          libtiff
        ];

        nativeBuildInputs = with pkgs; [
          (python311.withPackages python-packages)
          cmake
          curl
          doxygen
          git
          graphviz
          ninja
          autoPatchelfHook
        ];

        env = {
          PN_INTERFACE = "${pkgs.pn-interface}";
        };

        installPhase = ''
          runHook preInstall

          # Create output directories
          mkdir -p $out/bin

          # Install the specific adaptio binary from src directory
          install -m755 src/adaptio $out/bin/adaptio

          cp -r ../assets/ $out/

          runHook postInstall
        '';

        postFixup = ''
          mv $out/bin/adaptio $out/bin/.adaptio-wrapped

          cat > $out/bin/adaptio <<EOF
          #!/usr/bin/env bash

          ulimit -c unlimited
          exec $out/bin/.adaptio-wrapped "\$@"
        EOF

          chmod a+rx $out/bin/adaptio
        '';
      };

      packages.adaptio-debug = packages.adaptio.overrideAttrs {
        pname = "adaptio-debug";
        cmakeBuildType = "Debug";
        dontStrip = true;
      };

      packages.adaptio-tests = let
        src = ./.;
        python-packages = ps: with ps; [
          jinja2
          pytest
          pyyaml
          pyzmq
          dataclasses-json
          setuptools
          numpy
          matplotlib
          websocket-client
          pandas
        ];
        revShort = if (self ? rev) then self.shortRev else if (self ? dirtyRev) then self.dirtyShortRev else "dirty-inputs";
      in pkgs.llvmPackages_19.stdenv.mkDerivation {
        pname = "adaptio-tests";
        version = pkgs.lib.strings.fileContents "${src}/version.txt";
        inherit src;

        separateDebugInfo = true;
        cmakeBuildType = "Debug";
        dontStrip = true;

        PnDriver_ROOT="${pndriver}";

        Pylon_ROOT = "${pylon}";

        cmakeFlags = [ 
          "-DGIT_REV_SHORT=${revShort}"
          "-DCMAKE_BUILD_TYPE=Debug"
        ];

        ninjaFlags = [ "adaptio-unit-tests" "adaptio-block-tests" ];

        buildInputs = with pkgs; [
          # Utilities
          boost181
          fmt
          nlohmann_json

          # Infrastructure
          cppzmq
          openssl

          # Data collection
          prometheus-cpp

          # Persistent storage
          yaml-cpp
          sqlite
          sqlitecpp

          # Hardware interfaces
          pndriver

          # Testing
          doctest # Unit tests
          trompeloeil # Mocking library

          # Maths
          eigen
          opencv
          ceres-solver 
          pcl # point cloud library (ransac)

          libtiff

          # Hardware interfaces
          pylon
        ];

        nativeBuildInputs = with pkgs; [
          (python311.withPackages python-packages)
          cmake
          curl
          doxygen
          git
          graphviz
          ninja
          autoPatchelfHook
        ];

        env = {
          PN_INTERFACE = "${pkgs.pn-interface}";
        };

        installPhase = ''
          runHook preInstall
          mkdir -p $out/bin
          cp src/adaptio-unit-tests $out/bin/
          cp src/adaptio-block-tests $out/bin/
          runHook postInstall
        '';
      };

      packages.adaptio-dev = pkgs.stdenv.mkDerivation {
        name = "adaptio-dev";

        src = ./adaptio.sh;

        dontUnpack = true;
        dontBuild = true;

        installPhase = ''
          mkdir -p $out/bin
          cp $src $out/bin/adaptio
        '';
      };

      packages.adaptio-docker =
      let
        adaptio = self.packages.${system}.default;
      in pkgs.dockerTools.buildLayeredImage {
        name = adaptio.name;
        tag = adaptio.version;
        created = builtins.substring 0 8 self.lastModifiedDate;

        contents = [
          adaptio
          pkgs.bash
        ];

        config = {
          Cmd = [ "${pkgs.bash}/bin/bash" ];
          Volumes = {
            "/etc/timezone:/etc/timezone:ro" = { };
            "/etc/localtime:/etc/localtime:ro" = { };
          };
        };
      };


      ################################
      ## Environment
      ################################
      packages.env = let
        llvmPackages = pkgs.llvmPackages_19;
      in pkgs.mkShell.override { stdenv = llvmPackages.stdenv; } {
        PnDriver_ROOT="${pndriver}";
        PN_INTERFACE="${pkgs.pn-interface}";
        Pylon_ROOT="${pylon}";
        packages = with pkgs; [
          pkgs.pn-interface
          valgrind
          perf-tools
          packages.adaptio-dev # Our dev script
          llvmPackages.libllvm # Puts llvm-symbolizer in path for use with ASAN
          lldb_19
          hotspot
          clang-tools_19
          graphviz
          markdownlint-cli2
        ] ++ builtins.filter (x: x != git) packages.adaptio.nativeBuildInputs ++ packages.adaptio.buildInputs;
      };

      devShells.default = packages.env;
    });
}
