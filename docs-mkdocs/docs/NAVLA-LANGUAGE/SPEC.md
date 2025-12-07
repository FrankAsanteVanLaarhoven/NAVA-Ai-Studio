# NAVΛ Language Specification

## Lexical
- Identifiers: `[A-Za-z_][A-Za-z0-9_]*`
- Numbers: decimal floats with SI suffix optional (e.g., `0.5m`, `15deg`)

## Grammar (EBNF excerpt)
```
intent     := "approach" target from? angle? constraints*
target     := IDENT | "target:" IDENT
from       := "from" axis
axis       := "+X" | "-X" | "+Y" | "-Y"
angle      := "within" NUMBER "deg"
constraints:= "avoid" IDENT | "keepout:" IDENT | "speed<" NUMBER "m/s"
```

## Semantics
- `approach t from +X within A`: set desired end-effector orientation and angular cone.
- `avoid k`: soft constraint with penalty weight; `keepout:k`: hard constraint (FCL).
- `speed< v`: follower velocity cap.

## Type System
- `Pose`, `Frame`, `Distance`, `Angle`, `Speed`. Units enforced at compile time.
- Constraint sets are first-class values; combinators produce new sets.
