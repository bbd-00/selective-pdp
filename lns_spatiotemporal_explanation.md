# Spatiotemporal Neighborhood LNS — Design Explanation

## What will be FREE (re-optimized by Gurobi)

| What | Why |
|------|-----|
| `y[i,k]` for all `k` for any **duplicated** request `i` (served by 2+ couriers) | These are the infeasible assignments — the root cause of the conflict |
| `y[i,k]` for all `k` for any **unserved** request `i` | No courier to fix to |
| `y[i,k]` for all `k` for the **k-nearest requests** to each duplicate | Freeing only the duplicate itself is often infeasible to repair; the k spatiotemporally nearest requests must also be re-routed to absorb the re-assignment |
| Any arc `(i, j, k)` where **at least one endpoint** is a pickup or delivery node of a request in the free set | The route segments touching the conflict region need to be re-formed |

## What will be FIXED (locked to the Lagrangian solution)

| What | Why |
|------|-----|
| `y[i, k_star] = 1` and `y[i, k≠k_star] = 0` for **singleton requests outside the neighborhood** | These are confident, uncontested assignments — no reason to re-evaluate them |
| Any arc `(i, j, k)` where **both endpoints are far** from all free requests | The route skeleton far from the conflict is valid — preserve it to speed up the repair MIP |

---

## Spatiotemporal Distance Metric

The distance metric answers: **"how related is request j to the duplicated request d?"** — the smaller the value, the more related they are, and the more they get freed together.

### Spatial component — physical proximity in the delivery network

For requests `d` and `j`, there are 4 combinations of their nodes:

- pickup_d → pickup_j
- pickup_d → delivery_j
- delivery_d → pickup_j
- delivery_d → delivery_j

We take the **minimum** of these four Euclidean distances. The idea: if any part of request `j` is geographically close to any part of request `d`, they are spatially related. A courier that can serve `d` can likely detour to serve `j`.

Divided by `D_max` (the longest pairwise distance in the instance) to normalize to [0, 1].

### Temporal component — proximity in time

We compare the **midpoint of d's pickup time window** to the **midpoint of j's pickup time window**:

```
|( e_pickup_d + l_pickup_d ) / 2  −  ( e_pickup_j + l_pickup_j ) / 2|
```

If two requests must be picked up at similar times, they compete for the same courier slots — freeing one without the other leaves the problem under-constrained. Delivery TWs are omitted because they are already correlated with pickup TWs through the precedence constraint (pickup must come before delivery on the same route).

Divided by `H` (planning horizon = 600) to normalize to [0, 1].

### Combined formula

```
dist(d, j) = alpha × spatial(d, j) / D_max  +  beta × temporal(d, j) / H
```

Default: `alpha = beta = 0.5` (equal weight). Both are tunable parameters.

- A value near **0** → j is a close neighbor of d → free it
- A value near **1** → j is far from d in both space and time → keep it fixed

The `neighborhood_k` nearest requests (by this score) around each duplicate are added to the free set.

---

## Arc-Level Fixing Logic

Once the free set of requests is built, a **close node set** is derived:

```
close_node_set = { i : i ∈ free_set }          # pickup nodes
               ∪ { i + n : i ∈ free_set }      # delivery nodes
```

Depot nodes (0 and 2n+1) are **not** in the close node set.

For each arc `(i, j, k)` where solution data exists:

- **Fix** if both `i` and `j` are outside `close_node_set`
- **Leave free** if at least one of `i`, `j` is inside `close_node_set`

This preserves the route skeleton at the beginning and end of each route (far from the conflict), while freeing the middle segments that pass through the conflict region.
