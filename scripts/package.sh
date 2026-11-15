#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
version="${1:?usage: package.sh <version> [output-dir]}"
out="${2:-$root/dist}"
: "${RENOIR_DIR:?set RENOIR_DIR to a checkout of renoir whose release library is built}"
mujoco_dir="${MUJOCO_DIR:-$HOME/.mujoco/mujoco}"
epoch="${SOURCE_DATE_EPOCH:-$(git -C "$root" log -1 --format=%ct)}"
name="mujoco_drones-${version}-linux-x86_64"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
build="$work/build"
stage="$work/$name"
export SOURCE_DATE_EPOCH="$epoch"
cmake -S "$root" -B "$build" -DCMAKE_BUILD_TYPE=Release -DENABLE_IPC=ON -DRENOIR_DIR="$RENOIR_DIR" \
  "-DCMAKE_C_FLAGS=-ffile-prefix-map=$root=/src -fdebug-prefix-map=$root=/src" \
  -DCMAKE_EXE_LINKER_FLAGS=-Wl,--build-id=none >/dev/null
cmake --build "$build" -j"$(nproc)" --target hummingbird urdf_import send_command sensor_tap pilot_tool >/dev/null
mkdir -p "$stage/bin" "$stage/lib"
for target in hummingbird urdf_import send_command sensor_tap pilot_tool; do
  install -m 0755 "$build/$target" "$stage/bin/$target"
  patchelf --set-rpath '$ORIGIN/../lib' "$stage/bin/$target"
done
cp -a "$RENOIR_DIR"/target/release/librenoir.so* "$stage/lib/"
cp -a "$mujoco_dir"/lib/libmujoco.so* "$stage/lib/"
cp -r "$root/model" "$root/meshes" "$root/scenarios" "$root/LICENSES" "$stage/"
install -m 0644 "$root/README.md" "$root/THIRD_PARTY.md" "$stage/"
install -m 0644 "$mujoco_dir/LICENSE" "$stage/LICENSES/MuJoCo-Apache-2.0.txt"
mkdir -p "$out"
(
  cd "$work"
  find "$name" -print0 | sort -z | tar --null -T - --no-recursion --sort=name --mtime="@$epoch" \
    --owner=0 --group=0 --numeric-owner -cf - | gzip -n > "$out/$name.tar.gz"
)
sha256sum "$out/$name.tar.gz"
