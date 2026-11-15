#!/bin/sh
set -eu
BIN_DIR=$1
SRC_DIR=$2
fail() { echo "FAIL: $1"; exit 1; }
for dir in "$SRC_DIR"/tests/golden/importer/*/; do
    name=$(basename "$dir")
    class=$(cat "$dir/class.txt")
    work=$(mktemp -d)
    mkdir -p "$work/in" "$work/model/airframes" "$work/src/vehicle/airframes"
    cp "$SRC_DIR/model/airframes/$name/${name}_source.urdf" "$work/in/"
    if [ -d "$SRC_DIR/model/airframes/$name/meshes" ]; then
        cp -r "$SRC_DIR/model/airframes/$name/meshes" "$work/in/"
    fi
    (cd "$work" && "$BIN_DIR/urdf_import" in "$name" --class "$class" > "$work/log.txt" 2>&1) ||
        { cat "$work/log.txt"; fail "$name: importer exited with an error"; }
    cmp -s "$work/model/airframes/$name/$name.xml" "$dir/$name.xml" || fail "$name: generated MJCF differs from the golden file"
    cmp -s "$work/src/vehicle/airframes/${name}_spec.c" "$dir/${name}_spec.c" || fail "$name: generated airframe spec differs from the golden file"
    rm -rf "$work"
    echo "ok $name"
done

work=$(mktemp -d)
if (cd "$work" && "$BIN_DIR/urdf_import" /nonexistent_dir quadx >/dev/null 2>&1); then fail "a missing input directory was accepted"; fi
if (cd "$work" && "$BIN_DIR/urdf_import" "$SRC_DIR/model/airframes/quadx" "Bad Name" >/dev/null 2>&1); then fail "an invalid output name was accepted"; fi
if (cd "$work" && "$BIN_DIR/urdf_import" >/dev/null 2>&1); then fail "missing arguments were accepted"; fi
rm -rf "$work"
echo "PASS"
