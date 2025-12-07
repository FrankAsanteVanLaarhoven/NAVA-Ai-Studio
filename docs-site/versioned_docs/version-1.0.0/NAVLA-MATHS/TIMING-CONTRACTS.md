# Timing Contracts and Real-Time Metrics

Let \\(\\Delta t\\) be the control period and \\(B\\) the budget. We record:

- **DMR** (deadline miss rate): fraction of cycles with runtime \\(> B\\).
- **AJ** (average jitter): mean absolute deviation of cycle times.
- **TTP** (time to plan): wall time of the planning call.

**Theorem (Gate Safety)** — If the follower command is held at the last good command
whenever runtime \\(> B\\) and the planner is preemptable with bounded worst-case
overrun \\(\\epsilon\\), then closed-loop command stream is piecewise constant with
bounded total variation \\(\\le f(B,\\epsilon)\\). *Proof sketch*: partition time line
by contract events and apply variation bounds per segment; details included in appendix.
