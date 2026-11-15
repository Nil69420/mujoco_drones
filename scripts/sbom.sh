#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
out="${1:-$root/dist}"
version="${2:?usage: sbom.sh <output-dir> <version>}"
: "${RENOIR_COMMIT:?set RENOIR_COMMIT to the renoir commit the build used}"
: "${MUJOCO_VERSION:?set MUJOCO_VERSION}"
: "${MUJOCO_SHA256:?set MUJOCO_SHA256}"
commit="$(git -C "$root" rev-parse HEAD)"
epoch="${SOURCE_DATE_EPOCH:-$(git -C "$root" log -1 --format=%ct)}"
mkdir -p "$out"
python3 - "$out/mujoco_drones.cdx.json" "$version" "$commit" "$epoch" <<'PY'
import datetime, json, os, sys, uuid

path, version, commit, epoch = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
renoir = os.environ["RENOIR_COMMIT"]
mujoco = os.environ["MUJOCO_VERSION"]
mujoco_sha = os.environ["MUJOCO_SHA256"]
document = {
    "bomFormat": "CycloneDX",
    "specVersion": "1.5",
    "serialNumber": "urn:uuid:" + str(uuid.uuid5(uuid.NAMESPACE_URL, "mujoco_drones@" + commit)),
    "version": 1,
    "metadata": {
        "timestamp": datetime.datetime.fromtimestamp(epoch, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "component": {
            "type": "application",
            "bom-ref": "mujoco_drones",
            "name": "mujoco_drones",
            "version": version,
            "purl": "pkg:github/Nil69420/mujoco_drones@" + commit,
        },
    },
    "components": [
        {
            "type": "library",
            "bom-ref": "mujoco",
            "name": "mujoco",
            "version": mujoco,
            "purl": "pkg:github/google-deepmind/mujoco@" + mujoco,
            "licenses": [{"license": {"id": "Apache-2.0"}}],
            "hashes": [{"alg": "SHA-256", "content": mujoco_sha}],
        },
        {
            "type": "library",
            "bom-ref": "renoir",
            "name": "renoir",
            "version": renoir,
            "purl": "pkg:github/Nil69420/renoir@" + renoir,
        },
    ],
    "dependencies": [{"ref": "mujoco_drones", "dependsOn": ["mujoco", "renoir"]}],
}
with open(path, "w") as handle:
    json.dump(document, handle, indent=2, sort_keys=True)
    handle.write("\n")
PY
sha256sum "$out/mujoco_drones.cdx.json"
