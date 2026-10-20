# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an academic optimization research project for the **Selective Pickup-Delivery Problem with Time Windows (SPDPTW) with Two Types of Couriers** — a variant of the vehicle routing problem (VRP). The goal is to maximize profit from serving delivery requests minus operating costs, subject to time windows, capacity, and driver availability constraints. Serving a request is optional.

Two courier types:
- **Committed drivers** (CDR): Permanent employees with block-based availability windows. Pay a fixed block cost `wC·(l_k − e_k)` whether used or not, plus `c_com_d` per unit distance on every arc, including start → first pickup.
- **Occasional drivers** (ODR): Gig workers with limited availability. Pay `c_occ_o` per request served plus `c_occ_d` per unit distance on every pickup/delivery arc (`A2`); the start → first pickup arc is free.

The current main line of work is the Branch-and-Price solver in `Branch and Price/`.

## Dependencies

No `requirements.txt` exists. Key packages required:
- `gurobipy` — Gurobi MILP solver (commercial license required)
- `numpy`
- `matplotlib`
- `openpyxl`

## Running the Code

**Branch-and-Price experiments** (GraphSearch vs labeling pricing, optionally vs the exact MIP):
```bash
python "Branch and Price/bnp_pricing_comparison.py"
```
Settings are constants at the top of the script (`INSTANCE_DIR`, `MAX_REQ`, `RUN_EXACT`, `TIME_LIMIT`, `EXACT_TIME_LIMIT`, `MIP_TIME_PER_NODE`, `MIP_HEURISTIC_MODES`, `ADD_ALL_COLUMNS`, `TIGHTEN_MODES`). It runs every `.txt` file in `INSTANCE_DIR` or its subfolders (relative to the repo root, e.g. `UpdatedInstances` or `UpdatedInstances/converted_sun_120_rev`) whose name contains `_REQ{n}_CDR{c}_ODR{o}` with `n ≤ MAX_REQ`, grouped by subfolder; the `instance_file` column shows paths relative to `INSTANCE_DIR`. Methods are generated as {GraphSearch, labeling} × `MIP_HEURISTIC_MODES` × `TIGHTEN_MODES` (e.g. `BnP_GS_mip_tw`, `BnP_Label_nomip_notw`), and the exact MIP is run once per tightening mode (`Exact_notw`, `Exact_tw`); the Comparison sheet compares every method against the first, every no-MIP method against its with-MIP counterpart, every tightened method against its untightened counterpart, and `Exact_tw` against `Exact_notw`. Preprocessing time is reported separately (`prep_time`) and excluded from all solve times. It writes `bnp_pricing_comparison_{dataset}_{cgall|cgfirst}_{modes}_{twmodes}.xlsx` (dataset = last folder of `INSTANCE_DIR`) into its own folder and works from any working directory. To solve one instance verbosely:
```bash
python "Branch and Price/branch_and_price.py" InstanceswithDemand/Instance_REQ20_CDR6_ODR4_FS0_SS0.txt
```

**Run a formulation directly:**
```bash
python Formulations/three_index_SPDPTW2OCC_SuperValid3D_Furtado_MTZ.py
python Formulations/SPDPTW2OCC_formulations.py
```

**Run Lagrangian relaxation:**
```bash
python Lagrangian/lagrangian_relaxation.py                    # base subgradient implementation
python Lagrangian/lagrangian_comparison_gurobi_GS.py          # compare Gurobi vs GraphSearch subproblems
python Lagrangian/lagrangian_comparison_lns_every_end.py      # Lagrangian + LNS feasibility repair
python Lagrangian/lagrangian_with_dfs.py                      # Lagrangian with DFS-based subproblems
```

**Run standalone GraphSearch heuristic:**
```bash
python Heuristics/GraphSearch/main.py
```

**Generate new instances:**
```bash
jupyter notebook InstancesGeneratorTTW2.ipynb
```

Results are written to `.txt` and `.xlsx` files next to the scripts that produce them. Solver time limit is 3600 seconds by default.

## Architecture

### Shared modules (repo root)

- **`instance_loader.py`**: canonical instance parser, used by `Branch and Price/`, `Lagrangian/` and `solve_exact_mip.py`. `Formulations/` and `Cuts/` keep their own, older copies — do not assume they match.
- **`preprocessing.py`**: `preprocess_instance(instance, tighten)` — the elimination and time-window tightening shared by `solve_exact_mip` and the B&P solver, so both always see the same windows, courier–request pairs (`z_UB`, exact direct-route check) and arcs (`x_UB`, `x_ind`). With `tighten=True` it alternates node-window tightening (four end-of-service rules over the surviving arcs, plus a pickup–delivery rule), courier-window tightening, `z_UB` and arc elimination until nothing changes, and marks requests no courier can serve as unservable (`z_UB = 0`; their windows stay original). It returns a tightened copy of the instance in which every courier keeps its original window in `e_orig`/`l_orig` — **the committed block cost must use those** (GraphSearch, labeling, the B&P empty columns and the exact MIP do). Tightening cannot change B&P's master LP bound (the set of feasible routes is unchanged); it tightens the exact MIP's LP relaxation.
- **`solve_exact_mip.py`**: `solve_exact_mip(instance_path, time_limit, verbose, tighten)` — the 3-index MIP solved with Gurobi. Returns `obj`, `lp_relax_ub` (LP relaxation, solved first and not included in `time`), `mip_gap`, `status`, `time`, `time_to_best` (Gurobi runtime of the final best incumbent, via a callback), `routes`, `prep_time`, `prep_stats`. Load is modelled with one variable per arc shared by all couriers, `L[i,j]` (each arc is used by at most one courier), with flow `=`; the Lagrangian full/LNS models use the same form.

### Branch and Price (`Branch and Price/`)

Self-contained apart from the two root modules above and `InstanceswithDemand/`.

- **`branch_and_price.py`**: `solve_instance_bnp(instance_path, time_limit, mip_time_per_node, use_mip_heuristic, add_all_columns, pricing_algo, gap_tol, verbose, tighten)`. Preprocessing comes from `preprocessing.preprocess_instance`; `total_time` starts after it.
  - Master: set-packing LP over route columns — packing row per request (dual `π_i ≥ 0`), one-route row per courier (dual `μ_k`, free; every courier always has an empty-route column), and a forcing row per forced courier–request pair (dual `σ_ik`, free).
  - Pricing per courier with duals `π_i (+ σ_ik for requests forced onto k)`; a column is added when `pricing_objval − μ_k > 1e-6`. `add_all_columns=True` adds one column per courier per round.
  - Branching: conflict branching with forcing (the only rule). Pick the request with the most couriers serving it fractionally (ties: largest `π_i`, then smallest index); create one child per such courier forced onto it (master FORCE row; all other couriers forbidden) plus one child forbidding all of them. Forbidding = `z_UB[i,k] = 0` in pricing + dropping inherited columns. No artificial variables are needed: inherited columns keep every child feasible.
  - Best-first search; optional per-node MIP over the column pool (`use_mip_heuristic`). Returns bounds, gaps, node/column/iteration counts, timings (incl. `pricing_time`, `time_to_best`), pruning counters and eliminated-request counts.
- **`graph_search.py`**: `GraphSearchSolver` — DFS pricing (Qiu et al. 2017, adapted). Copy of `Lagrangian/graph_search.py`.
- **`labeling.py`**: `LabelingSolver` — ESPPRC label-setting pricing with dominance on (profit, time, open set, unreachable set). Drop-in replacement for `GraphSearchSolver` (same constructor and `solve()` output).
- **`pricing_bounds.py`**: `TimeBudgetBound` — fractional-knapsack time-budget bound used by both pricing solvers. Copy of `Lagrangian/pricing_bounds.py`.
- **`bnp_pricing_comparison.py`**: experiment runner; methods are listed in its `METHODS` registry, and sheets/columns are generated from it. Its "Solution" sheet has one row per instance and method (exact MIP included) with statistics of the final solution.
- **`solution_stats.py`**: `solution_statistics(instance, routes, reported_profit)` re-simulates a solution's routes with the solvers' timing and cost rules and returns service, fleet-usage, profit-breakdown, route-structure and time-window statistics. Its `profit_check` (reported − recomputed profit), `tw_violations` and `capacity_violations` must be 0 — nonzero values mean a solver and the model disagree. `solve_exact_mip` also returns `routes` (read from the arc variables) for this.

Both pricing solvers share the same exact pruning rules: requests with gain `rev − π_i − c_k ≤ 0` are removed before the search (`c_k = c_occ_o` for occasional, 0 for committed); pickups that can no longer be completed in time are dropped for the rest of a branch; a partial route is abandoned once an on-board delivery can no longer be made; and the bound is the time-budget bound. These rely on travel times = Euclidean distances (triangle inequality). They must return identical optimal values — when changing either solver, re-check the two against each other.

### Lagrangian (`Lagrangian/`)

All Lagrangian scripts share the same subgradient loop structure; they differ in how the single-vehicle subproblem is solved:

- **`lagrangian_relaxation.py`**: Base implementation — solves subproblems with Gurobi.
- **`lagrangian_comparison_gurobi_GS.py`**: Side-by-side comparison of Gurobi vs `GraphSearchSolver` as subproblem solvers.
- **`lagrangian_with_dfs.py`**: Uses a DFS-based solver for subproblems.
- **`lagrangian_comparison_lns_every_end.py`**: Adds LNS feasibility repair after the subgradient loop (optionally after each iteration via `lns_every_iter=True`).
- **`lagrangian_bnb_fresh.py`**, **`lagrangian_reopt.py`**, **`lagrangian_lns_overlapping*.py`**: Lagrangian branch-and-bound and repair variants.
- **`lagrangian_comparison_reopt_bnb_updated_exact.py`**, **`lagrangian_comparison_full.py`**: comparison runners (the latter has a `METHODS` registry including B&P).
- **`graph_search.py`** / **`pricing_bounds.py`**: the Lagrangian copies used by these scripts. `graph_search_original.py` is the pre-modification version.

The scripts import `instance_loader` from the repo root (each adds the root to `sys.path`).

### Formulations (`Formulations/`)

- **`three_index_SPDPTW2OCC_baseline.py`**: the colleague's 3-index experiments script (per-courier "max requests" pre-solves, then the full profit model), with corrected preprocessing: end-of-service time-window tightening (`tighten_time_windows`, incl. a pickup–delivery rule and detection of unservable requests) alternated with courier–request (`compute_z_UB`) and arc (`compute_arc_elimination`) elimination until nothing changes. Its instance path is hard-coded; it imports `Formulations/instance_loader.py` (old copy, so per-request `rev` falls back to `rev_order`).

### Cuts (`Cuts/`)

- **`three_index_SPDPTW2OCC_SuperValid3D_Furtado_MTZ_validineq*.py`**: 3-index formulation variants that add valid inequalities.
- **`cut_experimentation.py`**: Tests cutting planes to tighten LP relaxations.

### Heuristics (`Heuristics/GraphSearch/`)

Standalone multi-vehicle heuristic with its own instance generator (does not use the instance folders):

- **`main.py`**: Entry point — randomly generates small instances and benchmarks DFS vs Gurobi.
- **`dfs_solver.py`**: DFS-based heuristic for multi-vehicle SPDPTW.
- **`gurobi_solver.py`**: Exact Gurobi baseline for comparison.
- **`instance_generator.py`**: Self-contained random instance generator (independent of `InstancesGeneratorTTW2.ipynb`).

### Node & Vehicle Indexing Convention

All formulations share the same node scheme (required to read any formulation file):

| Set | Indices | Meaning |
|-----|---------|---------|
| `V` | `0, 1..n, n+1..2n, 2n+1` | source depot, pickups, deliveries, sink depot |
| `P` | `1..n` | pickup nodes |
| `D` | `n+1..2n` | delivery node for request `i` is `i + n` |
| `K` | `KC + KO` | all vehicles (committed first, then occasional) |

The 3-index formulation uses 0-based vehicle indices; the 2-index formulation (`SPDPTW2OCC_formulations.py`) uses 1-based vehicle indices. Arc sets: `A1` (source→pickups), `A2` (pickup/delivery↔pickup/delivery), `A3` (deliveries→sink, plus the empty-route arc source→sink; all zero cost).

**Time windows apply to the end of service**: service at `j` starts at `max(arrival, e_j − s_j)` and must end by `l_j` (and by the courier's `l_k`). This convention is used consistently by the MIP, GraphSearch and labeling.

### Instance Format

Instance files follow this naming convention:
```
Instance_REQ{n}_CDR{c}_ODR{o}_FS0_SS{seed}.txt
```

File sections: `PARAMETERS` → `COMMITTED_DRIVERS` → `OCCASIONAL_DRIVERS` → `REQUESTS`

`instance_loader.load_instance_from_txt(path)` returns a dict with keys: `H`, `b`, `W`, costs (`c_com_h`, `c_com_d`, `c_occ_o`, `c_occ_d`, `rev_order`), and lists `committed`, `occasional`, `requests`. Each request has a `rev`: the optional 13th column of its line (per-request revenue), otherwise `rev_order`. `solve_exact_mip.py` and the B&P pricing solvers use `rev`.

- **`InstanceswithDemand/`**: the original generated set. Fleets of 10 couriers (CDR/ODR = 0/10 … 10/0), REQ 10–100, seed `SS0` only. Includes courier capacities and request demands.
- **`UpdatedInstances/`**: converted benchmarks in the same format, same fleet mixes and costs, but different parameters — `H` = 1000 (Ropke) / 840 (Sun), service times all 0, window widths 60–120 (`W` in the file is not the actual width), capacities 15–40 with demands 5–20. `converted_ropke_instances/`: REQ 30–75, revenue 20 for all. `converted_sun_{120,180,240}_rev/` (number = committed window length): REQ 10–45, per-request revenues 11–76 in the 13th column (also in `*_request_profits.csv`). File names differ (`AA30.3_REQ30_CDR4_ODR6.txt`, `Prob-10A-50_REQ10_CDR4_ODR6_CTW120_OTW60_random.txt`).
- **`InstancesTTW/`**: older format without capacities/demands — the root `instance_loader.py` cannot read it. Despite the name, its windows are not tightened (all exactly `W = 60` wide).

Known generator quirk (`InstancesGeneratorTTW2.ipynb`, delivery windows): the delivery window is the pickup window shifted by travel time only, without the delivery's service time. Under the end-of-service convention, a pickup ending in the last `delay_d` time units of its window cannot be delivered on time. All instances in both folders have this. The `tighten_time_windows` routine (generator notebook and two `Formulations/` scripts) is not applied to `InstanceswithDemand/`, and its rule 3 uses `L[i]` where `L[j]` is intended, so that rule never tightens anything.

### Key Solver Parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `H` | 600 | Planning horizon (time units) |
| `W` | 60 | Time window width |
| `b` | 60 | Committed driver block length |
| Grid | 50×50 | Coordinate space for instance generation |
| Costs | `c_com_h=0.4`, `c_com_d=0.2`, `c_occ_o=0.5`, `c_occ_d=0.2`, `rev_order=20` | Values in `InstanceswithDemand/` |
| Time limit | 3600s | Gurobi solver limit |

## Known Issues

- The Lagrangian scripts (including `Lagrangian/graph_search.py`) still use the single `rev_order` for every request, so they ignore per-request revenues (Sun instances). Only `solve_exact_mip.py` and the `Branch and Price/` solvers use `rev`.

- `Branch and Price/` and `Lagrangian/` each have their own `graph_search.py` and `pricing_bounds.py` (currently identical). Changes to one are not picked up by the other. `Lagrangian/lagrangian_comparison_full.py` imports both folders in one process, so all its methods share a single `graph_search` module (whichever loads first).
- `Lagrangian/lagrangian_comparison_full.py` passes `max_cg_iters` to `solve_instance_bnp`, which does not accept it, so its B&P rows fail.
