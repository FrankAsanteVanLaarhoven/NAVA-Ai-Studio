# Overview

NAVΛ (pronounced "nav-lambda") is a **decision + motion** framework that unifies
language-level intents, spatial reasoning, and robot control under one roof.
It compiles to Rust/C++/Python/TypeScript and backstops every decision with
**Timing Contracts** and **Evidence Packs**:

- **Timing Contracts**: formal bounds that the runtime enforces (and benchmarks) on
  compute + control cycles.
- **Evidence Packs**: signed bundles logging inputs, constraints, metrics, and verdicts.

This overview maps the user manual, language spec, math formalism, and V&V plan.
It also enumerates **symbols, acronyms, and abbreviations** used across the suite.
