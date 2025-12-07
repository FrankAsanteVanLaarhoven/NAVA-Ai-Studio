# Claims → Tests → Evidence Traceability

| Claim | Test ID | Evidence | Pass Criteria |
|---|---|---|---|
| Timing contracts enforced | RT-001 | EvidencePack JSON + charts | DMR < 1%, AJ < 5 ms |
| Occupancy-aware slowdown | OCC-002 | Path logs + SSE | v scaled vs ρ along path |
| Keep-out respected | COL-010 | FCL logs + min distance | min(d) ≥ margin |
| Approach cone satisfied | IK-020 | Pose logs | terminal orientation error ≤ α |
