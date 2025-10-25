#!/usr/bin/env bash
shopt -s globstar

if [ ! -e adaptio.sh ]
then
  echo "Script must be run in root project directory."
  exit 1
fi

COMMAND=""
DEBUG=true

usage() {
 echo "Usage: $(basename "$0") [OPTIONS]"
 echo "Options:"
 echo " -h, --help        Display this help message"
 echo " -b, --build       Builds adaptio binary"
 echo " -t, --build-tests Builds adaptio tests binary"
 echo " --build-test-images  Builds test images binary"
 echo " -r, --run         Runs adaptio, everything after -- will be passed as arguments to the binary"
 echo "--release          If passed, this script will build or run the release binary"
 echo "--check-includes   Uses clang-tidy to check for missing/unused includes"
 echo "--fix-includes     Uses clang-tidy to fix missing/unused includes"
 echo "--check-clang-tidy Uses clang-tidy to check the code base"
 echo "--fix-clang-tidy   Uses clang-tidy to fix clang-tidy violations in the code base"
 echo "--run-all          Runs all docker images and then starts the adaptio binary"
 echo "--unit-tests       Runs all unit tests, arguments passed after -- will be passed to the test runner"
 echo "--block-tests      Runs all block tests, arguments passed after -- will be passed to the test runner"
 echo "--docs             Opens the doxygen docs in the standard browser, docs are built with --build --release"
 echo "--clean-direnv     Removes the .direnv directory"
 echo ""
 echo "Example: Build release binary: $(basename "$0") --build --release"
 echo "Example: Run block tests with debug output: $(basename "$0") --block-tests -- --debug"
}

if [ $# -eq 0 ]
then
  usage
  exit 1
fi

while [ $# -gt 0 ]
do
  case "$1" in
    -h | --help ) usage; exit 0 ;;
    -b | --build ) COMMAND="build" ;;
    -t | --build-tests ) COMMAND="build-tests" ;;
    --build-test-images ) COMMAND="build-test-images" ;;
    -r | --run ) COMMAND="run" ;;
    --release ) DEBUG=false ;;
    --check-includes ) COMMAND="check-includes" ;;
    --fix-includes ) COMMAND="fix-includes" ;;
    --check-clang-tidy ) COMMAND="check-clang-tidy" ;;
    --fix-clang-tidy ) COMMAND="fix-clang-tidy" ;;
    --run-all ) COMMAND="run-all" ;;
    --unit-tests ) COMMAND="unit-tests" ;;
    --block-tests ) COMMAND="block-tests" ;;
    --docs ) COMMAND="docs" ;;
    --clean-direnv ) COMMAND="clean-direnv" ;;
    -- ) shift; break ;;
    * ) echo "Unknown argument: $1"; echo ""; usage; exit 1 ;;
  esac

  shift
done

check_includes() {
  nr_jobs=$(($(nproc) + 1))
  echo "Checking includes jobs=${nr_jobs}..."
  run-clang-tidy -checks="-*,misc-include-cleaner" -warnings-as-errors="-*,misc-include-cleaner" \
    -extra-arg="-Wno-unknown-pragmas" -j "${nr_jobs}" -p build/debug/ src/**/*.cc
}

fix_includes() {
  nr_jobs=$(($(nproc) + 1))
  echo "Fixing includes jobs=${nr_jobs}..."
  run-clang-tidy -checks="-*,misc-include-cleaner" -warnings-as-errors="-*,misc-include-cleaner" \
    -extra-arg="-Wno-unknown-pragmas" -fix -j "${nr_jobs}" -p build/debug/ src/**/*.cc
}

check_clang_tidy() {
  nr_jobs=$(($(nproc) + 1))
  run-clang-tidy -config-file ".clang-tidy-limited" -j "${nr_jobs}" -p build/debug/ src/**/*.{h,cc}
}

fix_clang_tidy() {
  nr_jobs=$(($(nproc) + 1))
  run-clang-tidy -config-file ".clang-tidy-limited" -fix -j "${nr_jobs}" -p build/debug/ src/**/*.{h,cc}
}

build_adaptio() {
  debug=$1
  nr_jobs=$(($(nproc) + 1))

  if [ "$debug" = true ]
  then
    echo "Building adaptio debug jobs=${nr_jobs}..."
    mkdir -p build/debug
    cd build/debug || return 1
    cmake -DCMAKE_BUILD_TYPE=Debug ../../ || return 1
    cmake --build . -j "${nr_jobs}" --target adaptio || return 1
    cd - || return 1
  else
    echo "Building adaptio release jobs=${nr_jobs}..."
    mkdir -p build/release
    cd build/release || return 1
    cmake -DCMAKE_BUILD_TYPE=Release ../../ || return 1
    cmake --build . -j "${nr_jobs}" --target adaptio || return 1
    cd - || return 1
  fi
}

build_tests() {
  debug=$1
  nr_jobs=$(($(nproc) + 1))

  if [ "$debug" = true ]
  then
    echo "Building adaptio-tests debug jobs=${nr_jobs}..."
    mkdir -p build/debug
    cd build/debug || return 1
    cmake -DCMAKE_BUILD_TYPE=Debug ../../ || return 1
    cmake --build . -j "${nr_jobs}" --target adaptio-unit-tests --target adaptio-block-tests || return 1
    cd - || return 1
  else
    echo "Building adaptio-tests release jobs=${nr_jobs}..."
    mkdir -p build/release
    cd build/release || return 1
    cmake -DCMAKE_BUILD_TYPE=Release ../../ || return 1
    cmake --build . -j "${nr_jobs}" --target adaptio-unit-tests --target adaptio-block-tests || return 1
    cd - || return 1
  fi

}

build_test_images() {
  debug=$1
  nr_jobs=$(($(nproc) + 1))

  if [ "$debug" = true ]
  then
    echo "Building test-images debug jobs=${nr_jobs}..."
    mkdir -p build/debug
    cd build/debug || return 1
    cmake -DCMAKE_BUILD_TYPE=Debug ../../ || return 1
    cmake --build . -j "${nr_jobs}" --target test-images || return 1
    cd - || return 1
  else
    echo "Building test-images release jobs=${nr_jobs}..."
    mkdir -p build/release
    cd build/release || return 1
    cmake -DCMAKE_BUILD_TYPE=Release ../../ || return 1
    cmake --build . -j "${nr_jobs}" --target adaptio-unit-tests || return 1
    cd - || return 1
  fi

}

start_adaptio() {
  debug=$1

  if [ "$debug" = true ]
  then
    echo "Starting adaptio debug..."
    ./build/debug/src/adaptio "$@"
  else
    echo "Starting adaptio release..."
    ./build/release/src/adaptio "$@"
  fi
}

run_unit_tests() {
  echo "Running unit tests..."
  debug=$1
  shift

  if [ "$debug" = true ]
  then
    ./build/debug/src/adaptio-unit-tests "$@"
  else
    ./build/release/src/adaptio-unit-tests "$@"
  fi
}

run_block_tests() {
  echo "Running block tests..."
  debug=$1
  shift

  if [ "$debug" = true ]
  then
    ./build/debug/src/adaptio-block-tests "$@"
  else
    ./build/release/src/adaptio-block-tests "$@"
  fi
}

open_docs() {
  xdg-open build/release/html/index.html 1>/dev/null 2>&1
}

case "$COMMAND" in
  build )
    build_adaptio $DEBUG
    ;;
  build-tests )
    build_tests $DEBUG
    ;;
  build-test-images )
    build_test_images $DEBUG
    ;;
  run )
    start_adaptio $DEBUG "$@"
    ;;
  check-includes )
    check_includes
    ;;
  fix-includes )
    fix_includes
    ;;
  check-clang-tidy )
    check_clang_tidy
    ;;
  fix-clang-tidy )
    fix_clang_tidy
    ;;
  build-docker )
    build_docker_images
    ;;
  run-docker )
    start_docker_containers
    ;;
  stop-docker )
    stop_docker_containers
    ;;
  run-all )
    build_adaptio $DEBUG || exit 1
    build_docker_images || exit 1
    start_docker_containers
    start_adaptio $DEBUG "$@"
    stop_docker_containers
    ;;
  unit-tests )
    run_unit_tests $DEBUG "$@"
    ;;
  block-tests )
    run_block_tests $DEBUG "$@"
    ;;
  docs )
    open_docs
    ;;
  clean-direnv ) # To apply changes when hacking on this file, run this and then reenter the development shell.
    rm -rf .direnv/flake-profile-*
    ;;
esac
