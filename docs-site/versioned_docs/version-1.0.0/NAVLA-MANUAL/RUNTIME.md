# Runtime Model

1. **Intent Parse** → constraint set \(\mathcal{C}\)
2. **Spatial4D** occupancy rollout → risk field \(\rho(x,y,t)\)
3. **Planner** → path \(\gamma\) with curvature profile \(\kappa(s)\)
4. **Follower** → \((v,\omega)\) under dynamic limits scaled by \(\rho\)
5. **Timing Gate** enforces cycle budgets; records DMR/AJ/TTP
6. **Evidence Pack** sealed with content hash and optional signatures
