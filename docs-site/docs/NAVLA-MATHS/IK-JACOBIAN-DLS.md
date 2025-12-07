# IK via Jacobian Transpose + Damped Least Squares

Given task error \\(e = x_d - x(q)\\), Jacobian \\(J(q)\\), and damping \\(\\lambda\\):

**JT step**: \\(\\Delta q = \\alpha J^T e\\)

**DLS step**: \\(\\Delta q = J^T (J J^T + \\lambda^2 I)^{-1} e\\)

**Joint-limit avoidance**: add gradient term \\(\\nabla U(q)\\) with weight \\(w\\).

**Task priority** (position over orientation): stack errors with weights or use
nullspace projection \\(N = I - J_1^\\# J_1\\).
We provide reference code in Rust/C++ in the main repo.
