#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RENOIR_DIR="$SCRIPT_DIR/../renoir"
BUILD_DIR="$SCRIPT_DIR/build"

BUILD_TYPE="${1:-release}"
ENABLE_IPC="${2:-ON}"
ENABLE_FOXGLOVE="${3:-ON}"

usage() {
    echo "Usage: $0 [debug|release|lint] [ON|OFF] [ON|OFF]"
    echo ""
    echo "  arg 1: build type / lint      (default: release)"
    echo "  arg 2: ENABLE_IPC             (default: ON)"
    echo "  arg 3: ENABLE_FOXGLOVE        (default: ON)"
    echo ""
    echo "Examples:"
    echo "  $0                  # release + IPC + Foxglove"
    echo "  $0 debug            # debug + IPC + Foxglove"
    echo "  $0 release OFF      # release, no IPC"
    echo "  $0 lint             # build + run clang-tidy static analysis"
    exit 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
fi

RUN_LINT=false
if [[ "$BUILD_TYPE" == "lint" ]]; then
    RUN_LINT=true
    BUILD_TYPE="release"
fi

NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

if [[ "$ENABLE_IPC" == "ON" ]]; then
    echo "=== Building renoir ($BUILD_TYPE) ==="

    if [[ ! -d "$RENOIR_DIR" ]]; then
        echo "ERROR: renoir not found at $RENOIR_DIR"
        exit 1
    fi

    CARGO_FLAGS="--features c-api"
    if [[ "$BUILD_TYPE" == "release" ]]; then
        CARGO_FLAGS="--release $CARGO_FLAGS"
    fi

    (cd "$RENOIR_DIR" && cargo build $CARGO_FLAGS)

    if [[ "$BUILD_TYPE" == "release" ]]; then
        RENOIR_LIB="$RENOIR_DIR/target/release"
    else
        RENOIR_LIB="$RENOIR_DIR/target/debug"
    fi

    if [[ ! -f "$RENOIR_LIB/librenoir.so" ]]; then
        echo "ERROR: librenoir.so not found at $RENOIR_LIB"
        exit 1
    fi

    echo ""
fi

echo "=== Building mujoco_drones ($BUILD_TYPE, IPC=$ENABLE_IPC) ==="

CMAKE_BUILD_TYPE="Release"
if [[ "$BUILD_TYPE" == "debug" ]]; then
    CMAKE_BUILD_TYPE="Debug"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake "$SCRIPT_DIR" \
    -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE" \
    -DENABLE_IPC="$ENABLE_IPC" \
    -DENABLE_FOXGLOVE="$ENABLE_FOXGLOVE"

make -j"$NPROC"

if [[ "$RUN_LINT" == true ]]; then
    echo ""
    echo "=== Running static analysis (clang-tidy) ==="
    make lint
fi

echo ""
echo "=== Done ==="
echo "Binary: $BUILD_DIR/hummingbird"

if [[ "$ENABLE_IPC" == "ON" ]]; then
    echo ""
    echo "Run with:"
    echo "  cd $BUILD_DIR && ./hummingbird"
    echo "  cd $BUILD_DIR && ./hummingbird --headless --duration 10"
    echo "  cd $BUILD_DIR && ./hummingbird --no-ipc"
fi
