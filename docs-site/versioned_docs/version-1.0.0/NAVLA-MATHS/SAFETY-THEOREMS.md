# Safety & Collision Theorems (Operational)

**Assumptions**: accurate URDF geometry, FCL broadphase/narrowphase, bounded latency.

**Claim A (Keep-out compliance)** — If path samples obey \\(d(\\gamma(s),\\mathcal{K}) \\ge r\\)
and follower tracking error \\(\\le r/2\\), then executed path avoids \\(\\mathcal{K}\\).
*Proof*: triangle inequality on closest-point distances.

**Claim B (Approach cone)** — With orientation cone \\(\\alpha\\), the end-effector
error is bounded by \\(\\alpha\\) at the terminal pose when JT/DLS is run to convergence
under Lipschitz Jacobian conditions. *Proof sketch*: monotone descent with bounded curvature.
