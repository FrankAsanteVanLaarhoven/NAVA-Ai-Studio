# Verification & Validation Plan

1. **Unit** — parsers, codegen, IK, follower math.
2. **Integration** — /api/solve → path → follower → cmd_vel.
3. **System** — XR/Web, haptics, metrics SSE, Evidence Pack generation.
4. **Performance** — PGO scenarios, CI shards, multi-URDF catalog.
5. **Audit** — signatures, SBOM/SLSA, Release manifest summary.

Each test stores inputs, config, and outputs in an **Evidence Pack** that is
machine-verifiable and reproducible.
