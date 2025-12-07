# NAVΛ Manual — Introduction

NAVΛ is a **language-first robotics stack**. Operators write intents such as:

```
approach target:bin_12 from +X within 20 deg avoid keepout:zone_A
```

The runtime compiles this into constraints, plans a collision-safe, time-feasible
path, and emits commands with **live metrics**. All runs produce an **Evidence Pack**
you can audit, sign, and attach to releases.

**Design goals**
- Deterministic core, real-time friendly
- Multi-target codegen (Rust/C++/Python/TS)
- Safety-by-construction constraints
- VR/WebXR front-end with haptics tied to constraint satisfaction
