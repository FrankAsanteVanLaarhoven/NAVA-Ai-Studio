# NAVΛ (NAVA) — Production Documentation Suite

This repository is a **production-ready documentation** corpus for the NAVΛ framework:
language, runtime, mathematics, verification, and SDKs. It is structured for direct
publishing via MkDocs Material or Docusaurus and mirrors the code layout of your
Univarm stack.

> Scope: language spec, math formalism, real‑time metrics (DMR/AJ/TTP), timing contracts,
> constraint calculus, IK, collision safety, VR/WebXR semantics, and evidence-backed V&V.

If you are reading this offline, start with **Overview**:

- `OVERVIEW.md` — exec and technical bridge
- `NAVLA-MANUAL/` — user/developer manual
- `NAVLA-LANGUAGE/` — full language spec + codegen
- `NAVLA-MATHS/` — formal math & proofs (with reproducible steps)
- `VALIDATION/` — claims → tests → evidence
- `TEMPLATES/` — theorem & experiment blueprints, code prefixes

---

## Quick local preview (MkDocs)
```bash
pip install -U mkdocs mkdocs-material
mkdocs serve
```
