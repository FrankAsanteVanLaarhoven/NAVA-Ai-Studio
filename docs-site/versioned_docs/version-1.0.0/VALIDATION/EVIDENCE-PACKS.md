# Evidence Packs

A pack is a signed zip containing:
- config (presets, versions)
- inputs (intent, URDF fingerprint)
- outputs (path, metrics, violations)
- plots (latency, curvature, risk)
- signatures and optional attestations

Packs are reproducible; CI can reject merges when thresholds are exceeded.
