#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
: "${RENOIR_DIR:?set RENOIR_DIR to a checkout of renoir whose release library is built}"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
epoch="${SOURCE_DATE_EPOCH:-$(git -C "$root" log -1 --format=%ct)}"
targets=(hummingbird urdf_import send_command sensor_tap pilot_tool)
build() {
  local dir="$1"
  mkdir -p "$dir/src"
  git -C "$root" archive HEAD | tar -x -C "$dir/src"
  (
    cd "$dir"
    export SOURCE_DATE_EPOCH="$epoch"
    cmake -S src -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_IPC=ON -DRENOIR_DIR="$RENOIR_DIR" \
      "-DCMAKE_C_FLAGS=-ffile-prefix-map=$dir=/build -fdebug-prefix-map=$dir=/build" \
      -DCMAKE_EXE_LINKER_FLAGS=-Wl,--build-id=none >/dev/null
    cmake --build build -j"$(nproc)" --target "${targets[@]}" >/dev/null
  )
  for target in "${targets[@]}"; do
    sha256sum "$dir/build/$target" | awk -v name="$target" '{print name " " $1}'
  done
}
first="$(build "$work/a")"
second="$(build "$work/b")"
echo "build A:"
echo "$first"
echo "build B:"
echo "$second"
if [ "$first" = "$second" ]; then
  echo "REPRODUCIBLE"
else
  echo "NOT REPRODUCIBLE"
  exit 1
fi
