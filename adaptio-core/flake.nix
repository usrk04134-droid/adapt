{
  description = ''
  '';

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    unstable.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    pylon-software = {
      type = "gitlab";
      owner = "esab";
      repo = "abw%2Fpylon-software";
      ref = "main";
      rev = "6f89cbac45fac0e5fff46504f9fbde66d155004c";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    infra-utils = {
      type = "gitlab";
      owner = "esab";
      repo = "abw%2Finfra-and-test";
      ref = "main";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = inputs@{ self, nixpkgs, flake-utils, pylon-software, infra-utils, ... }:
    flake-utils.lib.eachSystem [ flake-utils.lib.system.x86_64-linux ] (system:
    let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [
          (final: prev: {
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

            unstable = import inputs.unstable {
              system = final.system;
              config.allowUnfree = true;
            };
            opencv = (prev.opencv.override {
              enableGtk2 = true;
            }).overrideAttrs {
              patches = prev.opencv.patches ++ [
                ./fix-opencv.patch
              ];
            };
          })
        ];
        config.allowUnfree = true;
        config.permittedInsecurePackages = [
        ];
      };

      pylon = pylon-software.defaultPackage.${system};

      revShort = if (self ? rev) then self.shortRev else if (self ? dirtyRev) then self.dirtyShortRev else "dirty-inputs";
      version_txt = pkgs.lib.strings.fileContents ./version.txt;

      cmakeBuild = { target, buildType, pname ? "", extraCmakeFlags ? [] }:
      pkgs.llvmPackages_19.stdenv.mkDerivation {
        pname = if (pname != "") then pname else target;
        version = if (buildType == "Release") then version_txt else "${version_txt}-dev";
        src = ./.;

        Pylon_ROOT = "${pylon}";

        cmakeBuildType = buildType;

        cmakeFlags = [
          "-DGIT_REV_SHORT=${revShort}"
        ] ++ extraCmakeFlags;

        buildPhase = ''
          cmake --build . ${if (target != "") then "--target ${target}" else ""}
        '';

        installPhase = ''
          mkdir -p $out
          cmake --install . --prefix $out ${if (target != "") then "--component ${target}" else ""}
        '';

        buildInputs = with pkgs; [
          # Utilities
          boost181
          fmt

          # Hardware interfaces
          pylon

          # Testing
          doctest # Unit tests

          # Persistent storage
          yaml-cpp

          # Maths
          opencv
          pcl # point cloud library (ransac)
          eigen # Linear algebra
          ceres-solver # Solve non-linear systems

          libtiff

          # Metrics
          prometheus-cpp
        ];

        nativeBuildInputs = with pkgs; [
          cmake
          ninja
          doxygen
          graphviz
          curl
        ];
      };

    in rec
    {
      ################################
      ## Package derivations
      ################################

      defaultPackage = packages.adaptio-core;

      packages.pylon = pylon;

      packages.adaptio-core = cmakeBuild {
        target = "adaptio-core";
        buildType = "Release";
      };

      packages.adaptio-core-dev = cmakeBuild {
        target = "adaptio-core";
        buildType = "Debug";
      };

      packages.adaptio-core-all = cmakeBuild {
        target = "";
        buildType = "Release";
        pname = "adaptio-core-all";
      };

      packages.adaptio-core-all-dev = cmakeBuild {
        target = "";
        buildType = "Debug";
        pname = "adaptio-core-all";
      };

      packages.adaptio-core-ci-dev = cmakeBuild {
        target = "";
        buildType = "Debug";
        pname = "adaptio-core-all";
        extraCmakeFlags = [
          "-DENABLE_CPU_PROCESSING_TIME=ON" # Used by test-images
        ];
      };

      packages.adaptio-core-tests-dev = cmakeBuild {
        target = "adaptio-core-tests";
        buildType = "Debug";
      };

      packages.data-set-runner = cmakeBuild {
        target = "data-set-runner";
        buildType = "Release";
      };

      packages.data-set-runner-dev = cmakeBuild {
        target = "data-set-runner";
        buildType = "Debug";
      };

      packages.image-annotation = cmakeBuild {
        target = "image-annotation";
        buildType = "Release";
      };

      packages.image-annotation-dev = cmakeBuild {
        target = "image-annotation";
        buildType = "Debug";
      };

      packages.test-images = cmakeBuild {
        target = "test-images";
        buildType = "Release";
      };

      packages.test-images-dev = cmakeBuild {
        target = "test-images";
        buildType = "Debug";
      };

      packages = {
        pre-commit = infra-utils.packages.${system}.pre-commit;
        gitlint = infra-utils.packages.${system}.gitlint;
        gittemplate = infra-utils.packages.${system}.gittemplate;
      };

      ################################
      ## Environment
      ################################
      packages.env = let
        python-packages = ps: with ps; [
          setuptools
        ];
        llvmPackages = pkgs.llvmPackages_19;
      in pkgs.mkShell.override { stdenv = llvmPackages.stdenv; } {
        Pylon_ROOT="${pylon}";

        packages = with pkgs; [
          valgrind
          perf-tools
          llvmPackages.libllvm # Puts llvm-symbolizer in path for use with ASAN
          lldb_19
          hotspot
          clang-tools_19
          julia
          (python311.withPackages python-packages)
        ] ++ packages.adaptio-core.nativeBuildInputs ++ packages.adaptio-core.buildInputs;
      };

      devShells.default = packages.env;
    }
  );
}
