#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
BINARY="$BUILD_DIR/hummingbird"

usage() {
    echo "Usage: $0 <command> [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  all        Build and run the simulator with Foxglove bridge"
    echo "  sim        Build and run the simulator without Foxglove"
    echo "  analyze    Build with sanitizers and run dynamic analysis"
    echo ""
    echo "Options (passed to hummingbird):"
    echo "  --headless       Run without visualization"
    echo "  --duration SEC   Sim duration in seconds (headless, default 10)"
    echo "  --altitude M     Initial target altitude (default 1.0)"
    echo "  --model PATH     Path to MJCF model file"
    echo "  --no-ipc         Disable IPC transport"
    echo "  --lidar-rays N   Number of LiDAR rays (default 36)"
    echo ""
    echo "Examples:"
    echo "  $0 all"
    echo "  $0 all --headless --duration 30"
    echo "  $0 analyze --headless --duration 5"
    exit 1
}

if [[ $# -lt 1 ]]; then
    usage
fi

CMD="$1"
shift

case "$CMD" in
    all)
        echo "=== Building with IPC + Foxglove ==="
        "$SCRIPT_DIR/build.sh" release ON ON
        echo ""
        echo "=== Running hummingbird (Foxglove on ws://0.0.0.0:8765) ==="
        cd "$BUILD_DIR"
        exec ./hummingbird "$@"
        ;;

    sim)
        echo "=== Building with IPC (no Foxglove) ==="
        "$SCRIPT_DIR/build.sh" release ON OFF
        echo ""
        echo "=== Running hummingbird ==="
        cd "$BUILD_DIR"
        exec ./hummingbird "$@"
        ;;

    analyze)
        echo "=== Building with AddressSanitizer + UBSan ==="
        ANALYZE_BUILD="$BUILD_DIR-analyze"
        mkdir -p "$ANALYZE_BUILD"
        cd "$ANALYZE_BUILD"

        cmake "$SCRIPT_DIR" \
            -DCMAKE_BUILD_TYPE=Debug \
            -DENABLE_IPC=ON \
            -DENABLE_FOXGLOVE=ON \
            -DENABLE_ASAN=ON

        NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
        make -j"$NPROC"

        echo ""
        echo "=== Running with sanitizers ==="
        export ASAN_OPTIONS="detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1:detect_leaks=1:halt_on_error=0"
        export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=0"
        exec ./hummingbird --headless --duration "${1:-10}" "${@:2}"
        ;;

    *)
        echo "Unknown command: $CMD"
        echo ""
        usage
        ;;
esac
