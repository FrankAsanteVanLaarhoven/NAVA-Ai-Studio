# Code Generation Targets

NAVΛ emits portable kernels with identical semantics:

| Target | Usage | Runtime |
|---|---|---|
| Rust | Planner kernels, real-time gates | `no_std`-friendly subsets supported |
| C++17/20 | ROS 2 nodes, IK | With Eigen / FCL |
| Python | Prototyping, gluing | NumPy; asyncio SSE helpers |
| TypeScript | WebXR front-end | Fetch/SSE clients |

**Prefix templates** are included in `/TEMPLATES/CODEGEN-PREFIXES/` to unify logging,
metrics hooks, and error surfaces across languages.
