#!/bin/sh
set -eu
BIN_DIR=$1
SRC_DIR=$2
fail() { echo "FAIL: $1"; exit 1; }
for dir in "$SRC_DIR"/tests/golden/importer_sdf/*/; do
    name=$(basename "$dir")
    class=$(cat "$dir/class.txt")
    work=$(mktemp -d)
    mkdir -p "$work/model/airframes" "$work/src/vehicle/airframes"
    (cd "$work" && "$BIN_DIR/urdf_import" "$dir/in" "$name" --class "$class" > "$work/log.txt" 2>&1) ||
        { cat "$work/log.txt"; fail "$name: importer exited with an error"; }
    cmp -s "$work/model/airframes/$name/$name.xml" "$dir/$name.xml" || fail "$name: generated MJCF differs from the golden file"
    cmp -s "$work/src/vehicle/airframes/${name}_spec.c" "$dir/${name}_spec.c" || fail "$name: generated airframe spec differs from the golden file"
    if grep -q "warning" "$work/log.txt"; then cat "$work/log.txt"; fail "$name: an absent optional pose produced a warning"; fi
    rm -rf "$work"
    echo "ok $name"
done

work=$(mktemp -d)
mkdir -p "$work/model/airframes" "$work/src/vehicle/airframes"
(cd "$work" && "$BIN_DIR/urdf_import" "$SRC_DIR/tests/golden/importer_sdf/pose_chain_sdf/in" pose_chain_sdf --class rotor_multicopter > log.txt 2>&1) || { cat "$work/log.txt"; fail "the pose chain did not import"; }
urdf="$work/model/airframes/pose_chain_sdf/pose_chain_sdf_source.urdf"
grep -q '<origin xyz="0.2 0.05 0" rpy="0 0 1.57079633"/>' "$urdf" ||
    fail "the joint origin is not inverse(parent pose) * child pose * joint pose"
grep -q '<origin xyz="0.3 0 0" rpy="0 0 -1.57079633"/>' "$urdf" ||
    fail "a child of a rotated parent has the wrong joint origin"
grep -q '<axis xyz="0 1 0"/>' "$urdf" ||
    fail "an axis given in the model frame was not converted into the joint frame"
grep -q '<visual name="arm_visual"><origin xyz="0.05 0 0" rpy="0 0 0"/>' "$urdf" ||
    fail "the child visual was not re-based into the joint frame"
grep -q '<inertial><origin xyz="0.05 0 0" rpy="0 0 0"/><mass value="0.1"/>' "$urdf" ||
    fail "the child inertial was not re-based into the joint frame"
rm -rf "$work"

work=$(mktemp -d)
mkdir -p "$work/in" "$work/model/airframes" "$work/src/vehicle/airframes"
cp "$SRC_DIR/tests/golden/importer_sdf/mini_quad_sdf/in/mini_quad_sdf.sdf" "$work/in/"
sed "s#<pose>-0.1 0.1 0.03 0 0 0</pose>#<pose relative_to=\"base_link\">-0.1 0.1 0.03 0 0 0</pose>#" "$work/in/mini_quad_sdf.sdf" > "$work/in/rel.tmp"
mv "$work/in/rel.tmp" "$work/in/mini_quad_sdf.sdf"
(cd "$work" && "$BIN_DIR/urdf_import" in relative_to_pose --class rotor_multicopter > log.txt 2>&1) || { cat "$work/log.txt"; fail "a relative_to pose stopped the import"; }
grep -q "relative_to" "$work/log.txt" || fail "an unsupported relative_to pose was accepted silently"
rm -rf "$work"

work=$(mktemp -d)
mkdir -p "$work/in" "$work/model/airframes" "$work/src/vehicle/airframes"
cp "$SRC_DIR/tests/golden/importer_sdf/mini_quad_sdf/in/mini_quad_sdf.sdf" "$work/in/"
sed 's#<pose>0.1 0.1 0.03 0 0 0</pose>#<pose>not a pose</pose>#' "$work/in/mini_quad_sdf.sdf" > "$work/in/bad.sdf.tmp"
mv "$work/in/bad.sdf.tmp" "$work/in/mini_quad_sdf.sdf"
(cd "$work" && "$BIN_DIR/urdf_import" in bad_pose --class rotor_multicopter > log.txt 2>&1) || { cat "$work/log.txt"; fail "an unreadable pose stopped the import"; }
grep -q "unreadable pose 'not a pose'" "$work/log.txt" || fail "an unreadable pose was accepted silently"
grep -q '<body name="rotor_0">' "$work/model/airframes/bad_pose/bad_pose.xml" ||
    fail "an unreadable pose was applied instead of the identity"
rm -rf "$work"

work=$(mktemp -d)
mkdir -p "$work/in"
cp "$SRC_DIR/tests/golden/importer_sdf/mini_quad_sdf/in/mini_quad_sdf.sdf" "$work/in/a.sdf"
cp "$SRC_DIR/tests/golden/importer_sdf/mini_quad_sdf/in/mini_quad_sdf.sdf" "$work/in/b.sdf"
if (cd "$work" && "$BIN_DIR/urdf_import" in two_models >/dev/null 2>&1); then fail "two model files in one directory were accepted"; fi
printf '<?xml version="1.0"?>\n<notamodel/>\n' > "$work/in/a.sdf"
rm "$work/in/b.sdf"
if (cd "$work" && "$BIN_DIR/urdf_import" in not_a_model >/dev/null 2>&1); then fail "a document that is neither URDF nor SDF was accepted"; fi
rm -rf "$work"
echo "PASS"
