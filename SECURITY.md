# Security policy

## Reporting a vulnerability

Report it privately through GitHub: Security tab, then "Report a vulnerability". Please do not
open a public issue for a vulnerability. Include the version or commit, the command or scenario
that triggers it, and what you observed.

## Security model

mujoco_drones is a local simulator. Processes that share its Renoir shared-memory topics
trust each other at the operating-system level; see the Renoir security model for what shared
memory does and does not defend against.

- **Network exposure.** The Foxglove bridge listens on 127.0.0.1 only by default. It has no
  authentication and no TLS, so set `MUJOCO_DRONES_FOXGLOVE_BIND` to another address only on a
  trusted network. Clients can read channels and manage their own subscriptions; there is no
  command path through it. Its limits (clients, queue size, message size, timeouts) are
  documented on the Foxglove page. The ArduPilot, PX4 and Betaflight bridges also bind loopback
  only.
- **Hostile input that is tested.** The WebSocket handshake, frame and message parsers, the
  ArduPilot servo-packet decoder, the MAVLink v2 parser (with a mutator that repairs the CRC so
  inputs get past the checksum) and the controller-profile parser each have a libFuzzer
  target that runs in CI. A 27-scenario adversarial suite attacks a live Foxglove bridge under
  AddressSanitizer in CI.
- **Input that is trusted.** Scenario files, URDF and SDF models, configuration files and
  command-line options are local files and arguments chosen by the person running the
  simulator. The importer bounds its parsing depth and sizes and has golden and error-path
  tests, but it is not fuzzed and should not be pointed at untrusted model files.
- **Supply chain.** Every GitHub Action is pinned by commit, the MuJoCo release tarball is
  verified against a pinned SHA-256 before use, and Renoir is built from a pinned commit. The
  flight stacks used by the end-to-end tests (ArduPilot, PX4, Betaflight) are pinned by
  commit. Dependabot proposes action updates weekly, CodeQL and OpenSSF Scorecard run on a
  schedule, and runner images are pinned to Ubuntu 24.04 with a nightly canary on the latest.

## Release integrity

Release packages are built by the `Release` workflow from a version tag. The build is checked
for reproducibility first (two builds at different paths must produce identical binaries), and
the package is a deterministic archive. Each release carries a CycloneDX SBOM, a build
provenance attestation, an SBOM attestation, and a keyless cosign signature over the checksum
file. To verify a download:

```bash
sha256sum -c SHA256SUMS
gh attestation verify mujoco_drones-<version>-linux-x86_64.tar.gz --repo Nil69420/mujoco_drones
cosign verify-blob SHA256SUMS --signature SHA256SUMS.sig --certificate SHA256SUMS.pem \
  --certificate-identity-regexp 'https://github.com/Nil69420/mujoco_drones/.github/workflows/release.yml@refs/tags/.*' \
  --certificate-oidc-issuer https://token.actions.githubusercontent.com
```

The package bundles `libmujoco` (Apache-2.0) and `librenoir`; the system libraries `libEGL`
and `libglfw` are not bundled.
