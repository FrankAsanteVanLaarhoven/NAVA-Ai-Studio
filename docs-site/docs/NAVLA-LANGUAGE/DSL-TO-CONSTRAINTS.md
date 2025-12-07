# DSL → Constraints Mapping

`approach target:bin_12 from +X within 20deg`
→ orientation cone constraint (axis + tolerance), terminal translation,
keep-out set from scene graph, and speed bound if requested.

We provide a JSON IR for constraints which the planner consumes.
