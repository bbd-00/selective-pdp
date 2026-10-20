# Review: time-window tightening and pre-processing in `three_index_SPDPTW2OCC_Final_code_experiments.py`

This note compares the time-window tightening and courier–request elimination in
`three_index_SPDPTW2OCC_Final_code_experiments.py` with the current solvers in the
`selective-PDP` repository (`solve_exact_mip.py` and `Branch and Price/branch_and_price.py`).
The repository solvers do not tighten time windows at all.

Line numbers refer to `three_index_SPDPTW2OCC_Final_code_experiments.py` unless stated otherwise.

## Convention used throughout

Time windows bound the **end** of service, as in the 3-index model in the same file. Its time
constraints have the form `T[i,k] − T[j,k] + (t_ij + s_j)·x[i,j,k] ≤ …`, i.e. `T_j ≥ T_i + t_ij + s_j`,
and `e_i·y ≤ T[i,k] ≤ l_i·y`. So service at node `j` starts at `max(arrival, e_j − s_j)` and must
end by `l_j` and by the courier's shift end `L_k`. Travel times equal Euclidean distances.

Notation: `E_i`, `L_i` are the (current) earliest and latest end-of-service times of node `i`;
`s_i` its service time; `t_ij` travel time; `E_k`, `L_k`, `d_ki` a courier's start time, shift end,
and distance from its start location to node `i`.

## 1. Time-window tightening (`tighten_time_windows`, lines 558–603, and courier block, lines 716–725)

### What the code does

1. Computes courier–request elimination (`z_UB`) on the original windows (line 699).
2. Calls `tighten_time_windows(ei, li, si, dij)` (line 710). It modifies `ei` and `li` in place,
   looping up to 100 times over all pickups and deliveries until the total change in a pass is
   below 0.01. Each pass applies four rules to every node `i`:
   - **Earliest arrival from predecessors** — `i` cannot be finished before the earliest finish
     of any possible predecessor (another node, or a courier start) plus travel and service.
   - **Earliest departure time to successors** — if every possible next stop opens later,
     finishing `i` earlier only adds waiting, so `E_i` can be raised.
   - **Latest arrival time from predecessors** — `i` cannot be finished later than the latest
     possible arrival from any predecessor plus service.
   - **Latest departure time to successors** — `i` must finish early enough to reach and serve
     some next stop (or, for a delivery, the end depot by the courier's shift end).
3. Tightens courier windows: start moved to the latest time at which the courier can still reach
   its earliest reachable pickup; end moved to the latest delivery deadline among the requests it
   can serve. The original courier windows (`Ek_org`, `Lk_org`) are kept for the committed cost.
4. Computes arc elimination (`x_UB`, `x_ind`, `aijk`) from the tightened windows.

Good points: the latest-arrival rule uses `L[j]` (the earlier generator version used `L[i]`, so
that rule never tightened anything); the end depot is included as a successor of every delivery,
which keeps the successor rules valid when a delivery is the last stop; courier starts are used
as predecessors of pickups; tightening runs before arc elimination.

### Service-time placement

**Wrong — the window becomes tighter than it really is, so feasible routes can be cut off:**

| Line | Rule | Code | Should be |
|---|---|---|---|
| 573 | Earliest arrival from predecessors (from a node) | `E[j] + S[j] + t_ji` | `E[j] + t_ji + S[i]` (wrong when `s_j > s_i`) |
| 580 | Earliest departure time to successors (to a node) | `E[j] − t_ij` | `E[j] − t_ij − S[j]` |
| 588 | Latest arrival time from predecessors (from a node) | `L[j] + t_ji` | `L[j] + t_ji + S[i]` |
| 589 | Latest arrival time from predecessors (from a courier) | `Lk[k] + d_ki` | `Lk[k] + d_ki + S[i]` |
| 720 | Courier earliest start | `ei[p] − d_kp` | `ei[p] − si[p] − d_kp` |

**Valid, but weaker than it could be:**

| Line | Rule | Code | Should be |
|---|---|---|---|
| 574 | Earliest arrival from predecessors (from a courier) | `Ek[k] + d_ki` | `Ek[k] + d_ki + S[i]` |
| 595 | Latest departure time to successors (to a node) | `L[j] − t_ij` | `L[j] − t_ij − S[j]` |

**Correct:** line 581 (end depot as successor, earliest), line 596 (end depot as successor,
latest), line 723 (courier latest end).

Line 573 follows a start-of-service convention, while the other lines leave service time out
entirely, so the function is consistent with neither convention. Under a start-of-service
convention the correct terms would be `E_j + s_j + t_ji`, `E_j − t_ij − s_i`, `L_j + s_j + t_ji`
and `L_j − t_ij − s_i`.

*Example (latest arrival from predecessors).* Node `i` has `s_i = 3` and a single predecessor `j`
that closes at 100 and is 10 away. The latest `i` can be finished is 100 + 10 + 3 = 113; the code
sets `L_i = 110`, which forbids the feasible route that finishes `j` at 100 and goes straight to `i`.

### How much it matters in the current instances

- `InstanceswithDemand/` (service times 1–5): compared with a version with service times placed
  correctly, 5 node earliest times and 35 courier start times end up too tight, by at most 4 time
  units. No latest time ends up too tight.
- On the 12 instances with REQ 10 and 20, every optimal route (solved on the original windows)
  is still feasible under the tightened windows, so the optimum did not change there. This is
  not a guarantee for other instances.
- `UpdatedInstances/` (Ropke and Sun conversions): all service times are 0, so these errors
  disappear there.

### What is missing

- **Pickup–delivery rule.** For each request `p` with delivery `d = p + n`:
  `L_p ≤ L_d − t_pd − s_d` and `E_d ≥ E_p + t_pd + s_d`. On every instance checked it would
  tighten every pickup. It also removes a known generator gap: delivery windows are the pickup
  windows shifted by travel time only, so a pickup finished in the last `s_d` time units of its
  window cannot be delivered in time. The general rules cannot capture this, because they take
  the minimum or maximum over all nodes instead of the request's own partner.
- **Weak predecessor/successor sets.** `A2` is still unfiltered when tightening runs, so every
  node is a possible predecessor and successor of every other — including a request's own
  delivery as a "predecessor" of its pickup. This is valid but weak: total window width drops by
  only 6–14% (without the pickup–delivery rule).
- **`z_UB` is not recomputed** after tightening.
- **Empty windows are not detected.** Capping with `min(L[i], …)` keeps `E ≤ L`, so a request
  that can never be served is not flagged or removed.
- `tol` and `max_iters` are ignored; the 100 passes and 0.01 threshold are hard-coded.

## 2. Courier–request elimination (`z_UB`)

A courier `k` can serve request `p` (with delivery `d`) exactly when, with
`pickup_end = max(E_k + d_kp, e_p − s_p) + s_p` and
`delivery_end = max(pickup_end + t_pd, e_d − s_d) + s_d`, we have
`pickup_end ≤ min(l_p, L_k)` and `delivery_end ≤ min(l_d, L_k)`.

- **Experiments script, line 701 (third condition):** `Lk ≥ max(e_d, e_p + s_p + t_pd) + s_d`.
  It adds `s_d` to `e_d`, which is already an end-of-service time, and starts from `e_p + s_p`
  although the pickup can end at `e_p`. **It wrongly removes 65 feasible courier–request pairs**
  in 30 of the 60 `InstanceswithDemand/` files, all with REQ 50–100.
  *Example:* `Instance_REQ100_CDR10_ODR0_FS0_SS0`, request 83 with courier 0: the courier can
  finish the delivery at 115.07, within both its shift end (120) and the delivery deadline
  (170.08), but the pair is removed. Whether this changes optimal values has not been checked;
  those instances are too large to solve quickly.
- **Repository code** (`solve_exact_mip.py`, `Branch and Price/branch_and_price.py`, and the
  `Lagrangian/`, `Cuts/` and `Formulations/` scripts): previously had a milder version of the same
  mistake — `l_p − s_p ≥ max(arrival, e_p)` and `l_k ≥ e_d + s_d`. It removed no feasible pair in
  any instance, and has now been corrected to `l_p − s_p ≥ max(arrival, e_p − s_p)` and
  `l_k ≥ e_d`.

## 3. Other differences noticed while reading (not tested)

- **Revenue (line 767):** `rev_order` is used for every request, so the per-request revenues of
  the Sun instances (13th column of each request line) are ignored. The repository solvers use
  each request's `rev`.
- **Per-courier cap on served requests (line 420):** the cap comes from solving each courier on
  its own. It is a valid bound only if that solve reaches optimality; if it stops at the time
  limit, the cap can be too low.
- **Valid-inequality switch (lines 288–384):** the branches are chained with `elif`, so `'ALL'`
  (including the single-courier subproblems) only adds the Arc cuts; `'2Pc'` is a typo for
  `'2PC'`. Neither matters while `vineq = 'None'`.
- **Load constraint (line 277):** uses `≥` instead of `=`, with load variables shared by all
  couriers. Still valid. The fixed version (`Formulations/three_index_SPDPTW2OCC_baseline.py`)
  uses `=`, and the repository models now use the same shared `L[i,j]` form.
