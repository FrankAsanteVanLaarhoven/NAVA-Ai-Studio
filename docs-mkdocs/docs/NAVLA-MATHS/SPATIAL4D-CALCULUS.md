# Spatial4D Calculus (NAVΛ-Σ)

We model occupancy as a field \\(\\rho: \\mathbb{R}^2 \\times \\mathbb{R}_{\\ge 0} \\to [0,1]\\).

**Constant-velocity rollout.**
For an entity with pose \\(p_0=(x_0,y_0)\\) and velocity \\(v=(v_x,v_y)\\),
the predicted position at horizon \\(t\\) is \\(p(t) = p_0 + v t\\). Occupancy
is the convolution of a kernel \\(K\\) with world entities:

\\[
\\rho(x,y,t) = 1 - \\prod_i \\bigl(1 - (K \\ast \\chi_{i,t})(x,y)\\bigr).
\\]

We use \\(\\sigma\\)-controlled Gaussian kernels for **risk intensity**. The follower
applies a **limit scale** \\(s = 1-\\rho^\\star\\) where \\(\\rho^\\star\\) is the
max risk sampled along the near-horizon of the path.
