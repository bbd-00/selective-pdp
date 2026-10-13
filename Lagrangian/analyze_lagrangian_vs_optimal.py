# analyze_lagrangian_vs_optimal.py
#
# Runs the Lagrangian relaxation (base: lagrangian_comparison_lns_every_end.py) on
# InstanceswithDemand/Instance_REQ30_CDR10_ODR0_FS0_SS0.txt, then solves:
#   (A) LNS repair MIP  – same fixing rule as the base file
#   (B) Full optimal MIP – no fixing, binary, time-limited
#
# Prints routes for both and a comparison table focused on how duplicated
# Lagrangian assignments are resolved in each solution.

import os
import sys
import math
import time
import gurobipy as gp
from gurobipy import GRB

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root: instance_loader
from instance_loader import load_instance_from_txt
from graph_search import GraphSearchSolver

# ── Configuration ─────────────────────────────────────────────────────────────
INSTANCE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "InstanceswithDemand",
    "Instance_REQ30_CDR8_ODR2_FS0_SS0.txt",
)
LAG_MAX_ITERS   = 100
LAG_LAMBDA0     = 2.0
LAG_MAX_NOIMPV  = 7
LAG_LAMBDA_MIN  = 1e-4
LNS_TIME_LIMIT  = 300   # seconds for LNS repair MIP
OPT_TIME_LIMIT  = 600   # seconds for full optimal MIP


# ── Helpers ───────────────────────────────────────────────────────────────────

def reconstruct_route(k, arcs_k, A, n):
    """Walk arc dict {(i,j): val} → ordered node list between depots (exclusive)."""
    used = {(i, j) for (i, j) in A if arcs_k.get((i, j), 0.0) > 0.5}
    if not used:
        return []
    succ = {i: j for (i, j) in used}
    route, cur = [], succ.get(0)
    while cur is not None and cur != 2 * n + 1:
        route.append(cur)
        cur = succ.get(cur)
    return route


def print_section(title):
    bar = "─" * 70
    print(f"\n{bar}")
    print(f"  {title}")
    print(bar)


def print_routes(label, K, KC, n, A, Ek, Lk, ei, li, si,
                 arcs_by_k, assignment_by_k=None):
    """
    Print per-courier route summary.

    arcs_by_k      : {k: {(i,j): float}}
    assignment_by_k: {k: set of pickup nodes served} – optional, used for labelling
    """
    print_section(label)
    P_set = set(range(1, n + 1))

    for k in K:
        route = reconstruct_route(k, arcs_by_k.get(k, {}), A, n)
        reqs  = [nd for nd in route if nd in P_set]
        tag   = "CDR" if k in KC else "ODR"
        print(f"\n  Courier {k} ({tag})  window=[{Ek[k]:.1f}, {Lk[k]:.1f}]"
              f"  served={reqs if reqs else '(none)'}")
        if route:
            # Simple timing simulation
            cur_t = Ek[k]
            path  = "0"
            for idx, nd in enumerate(route):
                travel = (arcs_by_k.get(k, {}).get(
                    (0, nd) if idx == 0 else (route[idx - 1], nd), 0.0))
                arr   = cur_t + travel
                start = max(arr, ei[nd] - si[nd])
                end_  = start + si[nd]
                node_type = "P" if nd in P_set else "D"
                req_id    = nd if nd in P_set else nd - n
                path += f" → {nd}({node_type}{req_id} [{ei[nd]:.0f},{li[nd]:.0f}] arr={arr:.1f})"
                cur_t = end_
            path += f" → {2*n+1}"
            print(f"    {path}")


# ── Instance setup (shared with base file) ────────────────────────────────────

print(f"\nLoading: {INSTANCE}")
instance = load_instance_from_txt(INSTANCE)

b   = int(instance["b"])
H   = float(instance["H"])
num_comm_drivers = len(instance["committed"])
num_occ_drivers  = len(instance["occasional"])
num_requests     = len(instance["requests"])

P  = list(range(1, num_requests + 1))
D  = [i + num_requests for i in P]
KC = list(range(0, num_comm_drivers))
KO = list(range(num_comm_drivers, num_comm_drivers + num_occ_drivers))
n  = len(P)
V  = [0] + P + D + [2 * n + 1]
K  = KC + KO

A1 = [(0, j) for j in P]
A2 = [(i, j) for i in P + D for j in P + D if i != j]
A3 = [(i, 2 * n + 1) for i in D] + [(0, 2 * n + 1)]

coords = {}
for req in instance["requests"]:
    pid = req["id"]
    coords[pid]          = req["pickup"]
    coords[pid + n]      = req["dest"]

ok   = ([d["location"] for d in instance["committed"]] +
        [d["location"] for d in instance["occasional"]])

dij_A2 = {(i, j): math.dist(coords[i], coords[j]) for (i, j) in A2}
dij_A3 = {(i, j): 0 for (i, j) in A3}
dij    = {**dij_A2, **dij_A3}
dijk   = {(i, j, k): math.dist(ok[k], coords[j]) for (i, j) in A1 for k in K}
tij    = dij.copy()
tijk   = dijk.copy()

si = {0: 0, 2 * n + 1: 0}
ei, li, dem = {}, {}, {}
for req in instance["requests"]:
    pid = req["id"]
    ei[pid], li[pid]             = req["e_pickup"],  req["l_pickup"]
    ei[pid + n], li[pid + n]     = req["e_dest"],    req["l_dest"]
    si[pid]                      = req["delay_p"]
    si[pid + n]                  = req["delay_d"]
    dem[pid]                     = req["l_dem"]
    dem[pid + n]                 = -req["l_dem"]
dem[0] = dem[2 * n + 1] = 0

Ek = ({d["id"]: d["e"] for d in instance["committed"]} |
      {d["id"]: d["e"] for d in instance["occasional"]})
Lk = ({d["id"]: d["l"] for d in instance["committed"]} |
      {d["id"]: d["l"] for d in instance["occasional"]})
Qk = ({d["id"]: d["cap"] for d in instance["committed"]} |
      {d["id"]: d["cap"] for d in instance["occasional"]})

p      = {i: instance["rev_order"] for i in P}
wC     = instance["c_com_h"] / b
cO     = instance["c_occ_o"]
deltaC = instance["c_com_d"]
deltaO = instance["c_occ_d"]
Lk_org = Lk.copy()

z_UB = {
    (pickup, driver): (
        1 if (li[pickup] - si[pickup] >=
              max(dijk[0, pickup, driver] + Ek[driver], ei[pickup])
              and Lk[driver] >= ei[pickup + n] + si[pickup + n])
        else 0
    )
    for driver in K for pickup in P
}

x_UB   = {(i, j): (0 if (ei[i] + dij[i, j] + si[j] > li[j] or
                          (i in D and j == i - n)) else 1)
           for (i, j) in A2}
UBxIPC1 = {(n2, d2): 0 if (d2 in D and d2 - n != n2 and
                             x_UB.get((d2 - n, n2), 0) == 0) else 1
            for (n2, d2) in A2}
UBxIPC2 = {(p2, n2): 0 if (p2 in P and p2 + n != n2 and
                             x_UB.get((n2, p2 + n), 0) == 0) else 1
            for (p2, n2) in A2}
x_ind  = {(i, j): x_UB[i, j] * UBxIPC1[i, j] * UBxIPC2[i, j]
          for (i, j) in A2}

A2     = [(i, j) for (i, j) in A2 if x_ind[i, j] == 1]
A      = A1 + A2 + A3
PK     = [(pp, kk) for pp in P for kk in K if z_UB[pp, kk] == 1]

print(f"n={n}, CDR={num_comm_drivers}, ODR={num_occ_drivers}, |K|={len(K)}")
print(f"|A|={len(A)}  |PK|={len(PK)}")


# ── Build full MIP (binary, no fixing) ───────────────────────────────────────

def build_full_mip(time_limit, label="mip"):
    """Full binary PDPTW MIP with no fixing constraints."""
    m = gp.Model(label)
    m.Params.OutputFlag = 0
    m.Params.TimeLimit  = time_limit

    x    = m.addVars(A, K, vtype=GRB.BINARY,      name="x")
    Lvar = m.addVars(A2, K, vtype=GRB.CONTINUOUS,  name="L")
    y    = m.addVars(P + D, K, vtype=GRB.BINARY,   name="y")
    T    = m.addVars(P + D + [0, 2 * n + 1], K,
                     vtype=GRB.CONTINUOUS, lb=0, name="T")

    obj = gp.quicksum(p[i] * gp.quicksum(y[i, k] for k in K) for i in P)
    for k in KC:
        obj -= wC * (Lk_org[k] - Ek[k]) + deltaC * (
            gp.quicksum(dijk[0, j, k] * x[0, j, k] for (_, j) in A1) +
            gp.quicksum(dij[i, j]     * x[i, j, k] for (i, j) in A2)
        )
    for k in KO:
        obj -= cO * gp.quicksum(y[i, k] for i in P) + deltaO * (
            gp.quicksum(dij[i, j] * x[i, j, k]
                        for (i, j) in A2)
        )
    m.setObjective(obj, GRB.MAXIMIZE)

    for i in P:
        m.addConstr(gp.quicksum(x[i, j, k]
                                for k in K for j in V if (i, j) in A) <= 1)
    for i in P + D:
        for k in K:
            m.addConstr(gp.quicksum(x[j, i, k] for j in V if (j, i) in A) -
                        gp.quicksum(x[i, j, k] for j in V if (i, j) in A) == 0)
    for i in P:
        for k in K:
            m.addConstr(
                gp.quicksum(x[i, j, k] for j in V if (i, j) in A) -
                gp.quicksum(x[j, i + n, k]   for j in V if (j, i + n) in A) == 0)
    for k in K:
        m.addConstr(gp.quicksum(x[0, j, k] for j in V if (0, j) in A) == 1)
        m.addConstr(gp.quicksum(x[j, 2*n+1, k]
                                for j in V if (j, 2*n+1) in A) == 1)
    for i in P + D:
        for k in K:
            m.addConstr(y[i, k] == gp.quicksum(x[i, l, k]
                                               for l in V if (i, l) in A))
    for i in P:
        for k in K:
            if z_UB[i, k] == 0:
                m.addConstr(y[i, k] == 0)

    svMTZ = {(i, j): min(-si[i] - dij[j, i], ei[j] - ei[i]) for (i, j) in A2}
    for (i, j) in A2:
        for k in K:
            if (j, i) in A2:
                m.addConstr(
                    T[i, k] - T[j, k] + (tij[i, j]+si[j]) * x[i, j, k] +
                    svMTZ[i, j] * x[j, i, k]
                    <= li[i]*y[i,k] - ei[j]*y[j,k]
                    - (li[i]-ei[j])*(x[i,j,k]+x[j,i,k]))
            else:
                m.addConstr(
                    T[i, k] - T[j, k] + (tij[i, j]+si[j]) * x[i, j, k]
                    <= li[i]*y[i,k] - ei[j]*y[j,k] - (li[i]-ei[j])*x[i,j,k])
    for (i, j) in A3:
        if i != 0:
            for k in K:
                m.addConstr(
                    T[i, k] - T[j, k] + (tij[i, j]+si[j]) * x[i, j, k]
                    <= li[i]*y[i,k]
                    - Ek[k]*gp.quicksum(x[0, ii, k] for ii in P if (ii,k) in PK)
                    - (li[i]-Ek[k])*x[i,j,k])
    for (i, j) in A1:
        for k in K:
            m.addConstr(
                T[0, k] - T[j, k] + (tijk[0,j,k]+si[j]) * x[0, j, k]
                <= Lk[k]*gp.quicksum(x[0,ii,k] for ii in P if (ii,k) in PK)
                - ei[j]*y[j,k] - (Lk[k]-ei[j])*x[0,j,k])
    for i in P + D:
        for k in K:
            m.addConstr(ei[i]*y[i,k] <= T[i,k])
            m.addConstr(T[i,k] <= li[i]*y[i,k])
    for i in P:
        for k in K:
            m.addConstr(T[i,k] + (tij[i, i+n]+si[i+n])*y[i,k] <= T[i+n, k])
    for k in K:
        m.addConstr(Ek[k]*gp.quicksum(x[0,i,k] for i in P if (i,k) in PK)
                    <= T[0, k])
        m.addConstr(T[2*n+1, k] <=
                    Lk[k]*gp.quicksum(x[0,i,k] for i in P if (i,k) in PK))
    for k in K:
        for i in P + D:
            m.addConstr(
                gp.quicksum(Lvar[i, j, k] for j in P+D if (i, j) in A2) -
                gp.quicksum(Lvar[j, i, k] for j in P+D if (j, i) in A2)
                == dem[i] * gp.quicksum(x[i, j, k] for j in V if (i, j) in A))
        for (i, j) in A2:
            m.addConstr(Lvar[i, j, k] >= max(dem[i], 0) * x[i, j, k])
            m.addConstr(Lvar[i, j, k] <= (Qk[k]+min(dem[i], 0)) * x[i, j, k])

    return m, x, y


# ── Build LNS repair MIP (same rule as base file) ────────────────────────────

def build_lns_repair_mip(served_by, time_limit):
    """Base-file fixing rule + claimant extension: singletons on claimant couriers freed."""
    m, x, y = build_full_mip(time_limit, label="lns_repair")

    duplicates = [i for i in P
                  if sum(1 for k in K if served_by[i][k] > 0.5) > 1]
    claimant_couriers = {k for i in duplicates for k in K
                         if served_by[i][k] > 0.5}

    n_fixed = n_free = n_claimant_freed = 0
    for i in P:
        assigned = [k for k in K if served_by[i][k] > 0.5]
        if len(assigned) == 1:
            k_star = assigned[0]
            if k_star in claimant_couriers:
                n_claimant_freed += 1
                n_free += 1
            else:
                m.addConstr(y[i, k_star] == 1)
                for k in K:
                    if k != k_star:
                        m.addConstr(y[i, k] == 0)
                n_fixed += 1
        else:
            n_free += 1

    print(f"  LNS MIP: fixed={n_fixed}  free={n_free}"
          f"  claimant_couriers={sorted(claimant_couriers)}"
          f"  claimant_freed={n_claimant_freed}")
    return m, x, y


def extract_routes(m, x_vars, K, A, n):
    """Extract {k: {(i,j): val}} from solved model."""
    arcs = {}
    for k in K:
        arcs[k] = {(i, j): x_vars[i, j, k].X for (i, j) in A}
    return arcs


def get_assignment(arcs_by_k, K, n):
    """Return {req: courier} for req in 1..n from arc solution."""
    P_set = set(range(1, n + 1))
    result = {}
    for k in K:
        route = reconstruct_route(k, arcs_by_k.get(k, {}), A, n)
        for nd in route:
            if nd in P_set:
                result[nd] = k
    return result


# ── LP relaxation (matches base file's surrogate primal) ─────────────────────

print_section("STEP 1a — LP relaxation")

_lp, _lp_x, _lp_y = build_full_mip(300, label="lp_relax")
# relax all binary variables to continuous
for v in _lp.getVars():
    v.vtype = GRB.CONTINUOUS
_lp.update()
_lp.optimize()
UB_LP = float(_lp.objVal)
print(f"  UB_LP = {UB_LP:.6f}")
del _lp, _lp_x, _lp_y


# ── Step 1b: Lagrangian subgradient ──────────────────────────────────────────

print_section("STEP 1b — Lagrangian subgradient (GraphSearch subproblems)")

u               = {i: 0.0 for i in P}
lambda_r        = LAG_LAMBDA0
no_improve_count = 0
best_L          = float("inf")
served_by       = {i: {k: 0.0 for k in K} for i in P}

print("Iter | L(u)           | best_L         | ||g||^2       | lambda   | noimp")

for it in range(1, LAG_MAX_ITERS + 1):
    sum_sub_obj     = 0.0
    served_by       = {i: {k: 0.0 for k in K} for i in P}
    served_sum_by_i = {i: 0.0 for i in P}

    for k in K:
        gs = GraphSearchSolver(instance, k, z_UB, x_UB, x_ind, u, verbose=False)
        res = gs.solve()
        sum_sub_obj += res["objVal"]
        for pickup in res["served_pickups"]:
            served_sum_by_i[pickup] += 1.0
            served_by[pickup][k]     = 1.0

    lagrangian_value = sum_sub_obj + sum(u.values())

    g = {i: 1.0 - served_sum_by_i[i] for i in P}
    norm_g_sq = sum(v * v for v in g.values())

    improved = lagrangian_value < best_L - 1e-9
    if improved:
        best_L = lagrangian_value
        no_improve_count = 0
    else:
        no_improve_count += 1

    print(f"{it:4d} | {lagrangian_value:14.6f} | {best_L:14.6f} | "
          f"{norm_g_sq:12.6f} | {lambda_r:7.4f} | {no_improve_count:5d}")

    if norm_g_sq <= 1e-12:
        print("  Zero subgradient — stopping.")
        break

    surrogate = -abs(UB_LP)
    step      = lambda_r * (lagrangian_value - surrogate) / norm_g_sq
    for i in P:
        u[i] = max(0.0, u[i] - step * g[i])

    if not improved and no_improve_count >= LAG_MAX_NOIMPV:
        lambda_r        *= 0.5
        no_improve_count = 0
        print(f"  → lambda reduced to {lambda_r:.6f}")
        if lambda_r < LAG_LAMBDA_MIN:
            print("  → lambda too small — stopping.")
            break

print(f"\n  best_L = {best_L:.6f}")


# ── Step 2: Classify requests from final Lagrangian iterate ──────────────────

print_section("STEP 2 — Lagrangian iterate: request classification")

P_set      = set(P)
duplicates = [i for i in P if sum(1 for k in K if served_by[i][k] > 0.5) > 1]
unserved   = [i for i in P if sum(1 for k in K if served_by[i][k] > 0.5) == 0]
singletons = [i for i in P if sum(1 for k in K if served_by[i][k] > 0.5) == 1]

print(f"\n  Total requests : {n}")
print(f"  Singletons     : {len(singletons)}")
print(f"  Duplicated     : {len(duplicates)}")
print(f"  Unserved       : {len(unserved)}")

if duplicates:
    print(f"\n  Duplicated requests (req → couriers claiming it):")
    for i in sorted(duplicates):
        claimants = sorted([k for k in K if served_by[i][k] > 0.5])
        print(f"    req {i:3d}  TW=[{ei[i]:.0f},{li[i]:.0f}]  → couriers {claimants}")

if unserved:
    print(f"\n  Unserved requests: {sorted(unserved)}")


# ── Step 3: LNS repair MIP ───────────────────────────────────────────────────

print_section("STEP 3 — LNS repair MIP (base-file fixing rule)")

t0 = time.time()
lns_m, lns_x, lns_y = build_lns_repair_mip(served_by, LNS_TIME_LIMIT)
lns_m.Params.OutputFlag = 0
lns_m.optimize()
lns_time = time.time() - t0

if lns_m.SolCount > 0:
    lns_obj   = lns_m.objVal
    lns_gap   = lns_m.MIPGap * 100 if lns_m.SolCount > 0 else None
    lns_arcs  = extract_routes(lns_m, lns_x, K, A, n)
    lns_assign = get_assignment(lns_arcs, K, n)
    lns_served = set(lns_assign.keys())
    print(f"\n  LNS objective = {lns_obj:.4f}   MIPgap={lns_gap:.2f}%   time={lns_time:.1f}s")
    print(f"  Requests served: {len(lns_served)} / {n}")
    print_routes("LNS repair — routes", K, KC, n, A, Ek, Lk, ei, li, si, lns_arcs)
else:
    lns_obj  = None
    lns_arcs = {}
    lns_assign = {}
    lns_served = set()
    print(f"\n  LNS repair found NO feasible solution (time={lns_time:.1f}s)")


# ── Step 4: Full optimal MIP ──────────────────────────────────────────────────

print_section("STEP 4 — Full optimal MIP (no fixing)")

t0 = time.time()
opt_m, opt_x, opt_y = build_full_mip(OPT_TIME_LIMIT, label="optimal")
opt_m.Params.OutputFlag = 0
opt_m.optimize()
opt_time = time.time() - t0

if opt_m.SolCount > 0:
    opt_obj    = opt_m.objVal
    opt_gap    = opt_m.MIPGap * 100
    opt_status = "OPTIMAL" if opt_m.status == GRB.OPTIMAL else f"TIME_LIMIT (gap={opt_gap:.2f}%)"
    opt_arcs   = extract_routes(opt_m, opt_x, K, A, n)
    opt_assign  = get_assignment(opt_arcs, K, n)
    opt_served  = set(opt_assign.keys())
    print(f"\n  Optimal objective = {opt_obj:.4f}   status={opt_status}   time={opt_time:.1f}s")
    print(f"  Requests served: {len(opt_served)} / {n}")
    print_routes("Optimal MIP — routes", K, KC, n, A, Ek, Lk, ei, li, si, opt_arcs)
else:
    opt_obj    = None
    opt_arcs   = {}
    opt_assign  = {}
    opt_served  = set()
    print(f"\n  Optimal MIP found NO feasible solution (time={opt_time:.1f}s)")


# ── Step 5: Comparison ────────────────────────────────────────────────────────

print_section("STEP 5 — Comparison: high-level summary")

lns_obj_str = f"{lns_obj:.4f}" if lns_obj is not None else "N/A"
opt_obj_str = f"{opt_obj:.4f}" if opt_obj is not None else "N/A"

print(f"\n  {'Metric':<40} {'LNS':>10} {'Optimal':>10}")
print(f"  {'-'*40} {'-'*10} {'-'*10}")
print(f"  {'Objective':<40} {lns_obj_str:>10} {opt_obj_str:>10}")
print(f"  {'Lagrangian bound (best_L)':<40} {best_L:>10.4f} {best_L:>10.4f}")
print(f"  {'Requests served':<40} {len(lns_served):>10} {len(opt_served):>10}")
print(f"  {'Requests skipped (not served)':<40} {n-len(lns_served):>10} {n-len(opt_served):>10}")

# ── 5a: Full request-level table ─────────────────────────────────────────────

print_section("STEP 5a — Full request table (every request)")

# Lagrangian status for each request
def lag_status(i):
    count = sum(1 for k in K if served_by[i][k] > 0.5)
    if count == 0:
        return "unserved"
    if count == 1:
        return f"singleton k={next(k for k in K if served_by[i][k] > 0.5)}"
    return f"DUPLICATE {sorted(k for k in K if served_by[i][k] > 0.5)}"

hdr = (f"  {'Req':>4}  {'TW_pickup':^14}  {'Lag status':<30}"
       f"  {'LNS':>8}  {'Opt':>8}  {'Agree?':>8}  Notes")
print(f"\n{hdr}")
print(f"  {'-'*100}")

req_notes = []
for i in sorted(P):
    lag_s  = lag_status(i)
    lns_c  = lns_assign.get(i)
    opt_c  = opt_assign.get(i)
    lns_str = f"k={lns_c}" if lns_c is not None else "skip"
    opt_str = f"k={opt_c}" if opt_c is not None else "skip"
    tw      = f"[{ei[i]:.0f},{li[i]:.0f}]"

    agree = (lns_c == opt_c)
    agree_str = "YES" if agree else "NO"

    notes = []
    if i in duplicates:
        notes.append("LAG_DUP")
    if i in unserved:
        notes.append("LAG_UNSERVED")
    if lns_c is None and opt_c is not None:
        notes.append("OPT_recovers")
    if lns_c is not None and opt_c is None:
        notes.append("LNS_only")
    if lns_c is None and opt_c is None and i in singletons:
        notes.append("BOTH_drop_singleton")
    if lns_c != opt_c and lns_c is not None and opt_c is not None:
        notes.append(f"reassigned(LNS:k{lns_c}→Opt:k{opt_c})")

    note_str = ", ".join(notes)
    print(f"  {i:>4}  {tw:^14}  {lag_s:<30}  {lns_str:>8}  {opt_str:>8}  {agree_str:>8}  {note_str}")

# ── 5b: Duplicate deep-dive ───────────────────────────────────────────────────

print_section("STEP 5b — Duplicated Lagrangian requests: deep dive")

if not duplicates:
    print("\n  No duplicated requests in final Lagrangian iterate.")
else:
    print(f"\n  {'Req':>4}  {'TW_pickup':^14}  {'Lag claimants & their windows':<45}"
          f"  {'LNS':>8}  {'Opt':>8}  Outcome")
    print(f"  {'-'*110}")
    for i in sorted(duplicates):
        claimants = sorted(k for k in K if served_by[i][k] > 0.5)
        tw        = f"[{ei[i]:.0f},{li[i]:.0f}]"
        cou_detail = "  ".join(
            f"k{k}[{Ek[k]:.0f},{Lk[k]:.0f}]" for k in claimants
        )
        lns_c = lns_assign.get(i)
        opt_c = opt_assign.get(i)
        lns_str = f"k={lns_c}" if lns_c is not None else "skipped"
        opt_str = f"k={opt_c}" if opt_c is not None else "skipped"

        if lns_c is None and opt_c is None:
            outcome = "both skip"
        elif lns_c == opt_c:
            outcome = f"agree → k{lns_c}"
        elif lns_c is None:
            outcome = f"LNS skips, Opt keeps k{opt_c}"
        elif opt_c is None:
            outcome = f"LNS keeps k{lns_c}, Opt skips"
        else:
            outcome = f"DISAGREE (LNS k{lns_c} vs Opt k{opt_c})"

        print(f"  {i:>4}  {tw:^14}  {cou_detail:<45}  {lns_str:>8}  {opt_str:>8}  {outcome}")

        # For each claimant: show what else they serve in LNS vs Opt
        for k in claimants:
            lns_route_k = [r for r in sorted(lns_assign) if lns_assign[r] == k]
            opt_route_k = [r for r in sorted(opt_assign) if opt_assign[r] == k]
            print(f"         claimant k{k}: LNS serves {lns_route_k}  |  Opt serves {opt_route_k}")

# ── 5c: Unserved in Lagrangian ───────────────────────────────────────────────

print_section("STEP 5c — Unserved in Lagrangian iterate")

if not unserved:
    print("\n  No unserved requests in final Lagrangian iterate.")
else:
    print(f"\n  {'Req':>4}  {'TW_pickup':^14}  {'z_UB=1 couriers':<30}"
          f"  {'LNS':>8}  {'Opt':>8}  Outcome")
    print(f"  {'-'*90}")
    for i in sorted(unserved):
        feasible_k = sorted(k for k in K if z_UB.get((i, k), 0) == 1)
        tw         = f"[{ei[i]:.0f},{li[i]:.0f}]"
        lns_c      = lns_assign.get(i)
        opt_c      = opt_assign.get(i)
        lns_str    = f"k={lns_c}" if lns_c is not None else "skipped"
        opt_str    = f"k={opt_c}" if opt_c is not None else "skipped"
        if lns_c is None and opt_c is None:
            outcome = "both skip — probably unprofitable"
        elif lns_c == opt_c:
            outcome = f"both recover → k{lns_c}"
        elif lns_c is None:
            outcome = f"LNS skips, Opt recovers k{opt_c}"
        elif opt_c is None:
            outcome = f"LNS recovers k{lns_c}, Opt skips"
        else:
            outcome = f"both recover but differ (LNS k{lns_c} vs Opt k{opt_c})"
        print(f"  {i:>4}  {tw:^14}  {str(feasible_k):<30}  {lns_str:>8}  {opt_str:>8}  {outcome}")

# ── 5d: Courier-level comparison ─────────────────────────────────────────────

print_section("STEP 5d — Courier-level comparison")

print(f"\n  {'Cour':>5}  {'Type':>4}  {'Window':^14}"
      f"  {'Lag assigned':^25}  {'LNS serves':^25}  {'Opt serves':^25}  Changes")
print(f"  {'-'*120}")

for k in K:
    tag      = "CDR" if k in KC else "ODR"
    win      = f"[{Ek[k]:.0f},{Lk[k]:.0f}]"
    lag_reqs = sorted(i for i in P if served_by[i][k] > 0.5)
    lns_reqs = sorted(i for i in P if lns_assign.get(i) == k)
    opt_reqs = sorted(i for i in P if opt_assign.get(i) == k)

    changes = []
    gained_lns = set(lns_reqs) - set(lag_reqs)
    lost_lns   = set(lag_reqs) - set(lns_reqs)
    gained_opt = set(opt_reqs) - set(lag_reqs)
    lost_opt   = set(lag_reqs) - set(opt_reqs)
    lns_vs_opt_gained = set(lns_reqs) - set(opt_reqs)
    lns_vs_opt_lost   = set(opt_reqs) - set(lns_reqs)

    if gained_lns:
        changes.append(f"LNS+{sorted(gained_lns)}")
    if lost_lns:
        changes.append(f"LNS-{sorted(lost_lns)}")
    if lns_vs_opt_gained:
        changes.append(f"Opt_drops{sorted(lns_vs_opt_gained)}")
    if lns_vs_opt_lost:
        changes.append(f"Opt_adds{sorted(lns_vs_opt_lost)}")

    # Only print couriers with something interesting (non-empty in at least one)
    if not lag_reqs and not lns_reqs and not opt_reqs:
        continue

    change_str = "  |  ".join(changes) if changes else "unchanged"
    print(f"  {k:>5}  {tag:>4}  {win:^14}"
          f"  {str(lag_reqs):^25}  {str(lns_reqs):^25}  {str(opt_reqs):^25}  {change_str}")

# ── 5e: LNS vs Optimal disagreements ─────────────────────────────────────────

print_section("STEP 5e — Where LNS and Optimal disagree")

only_opt  = opt_served - lns_served
only_lns  = lns_served - opt_served
both_serv = opt_served & lns_served
reassigned = {i for i in both_serv if lns_assign[i] != opt_assign[i]}

print(f"\n  Served by LNS only (Opt skips) : {sorted(only_lns)}")
print(f"  Served by Opt only (LNS skips) : {sorted(only_opt)}")
print(f"  Served by both, same courier   : {len(both_serv) - len(reassigned)}")
print(f"  Served by both, diff courier   : {len(reassigned)}")

if reassigned:
    print(f"\n  Requests reassigned between LNS and Optimal:")
    print(f"  {'Req':>4}  {'TW_pickup':^14}  {'Lag status':<25}  {'LNS':>8}  {'Opt':>8}")
    print(f"  {'-'*75}")
    for i in sorted(reassigned):
        tw    = f"[{ei[i]:.0f},{li[i]:.0f}]"
        lag_s = lag_status(i)
        print(f"  {i:>4}  {tw:^14}  {lag_s:<25}  {f'k={lns_assign[i]}':>8}  {f'k={opt_assign[i]}':>8}")

print()

