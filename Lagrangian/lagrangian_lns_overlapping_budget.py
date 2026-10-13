# lagrangian_lns_overlapping_budget.py
# Extends lagrangian_lns_overlapping.py with a budget cap on freed (i,k) pairs.
# Search "# LNS-BUD" for every new/changed line vs the overlapping file.
#
# CHANGES vs lagrangian_lns_overlapping.py:
#   1.  Two-stage fixing:
#       Stage 1 (overlap filter): same as overlapping file — builds
#       free_requests and free_couriers from TW overlap with conflict set.
#       Stage 2 (budget): scores each candidate (i,k) pair by
#         score(i,k) = s_req(i) * s_cou(k)
#       where s_req(i) is the max normalised 4-combo TW overlap of request i
#       with any conflict request, and s_cou(k) is the max normalised
#       courier-window overlap with any conflict request's pickup TW
#       (gated by z_UB=1).  Only the top-N pairs (by score) are freed, where
#         N = max(1, int(free_budget_frac * z_ub_ones)).
#   2.  New parameter lns_free_budget_frac (default 1.0 = no cap, identical
#       to overlapping file behaviour).
#   Everything else is UNCHANGED.

import os
import math
import time
import gurobipy as gp
from gurobipy import GRB
from openpyxl import Workbook
from openpyxl.utils import get_column_letter
from openpyxl.styles import Font, PatternFill

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root: instance_loader
from instance_loader import load_instance_from_txt
from graph_search import GraphSearchSolver


# ──────────────────────────────────────────────────────────────────────────────
# Route printing helper  (set print_routes=True in solve_instance_comparison)
# ──────────────────────────────────────────────────────────────────────────────

def _reconstruct_route(k, arcs_k, A, n):
    """Walk arc dict {(i,j): val} and return ordered node list, or [] if empty."""
    used = {(i, j) for (i, j) in A if arcs_k.get((i, j), 0.0) > 0.5}
    if not used:
        return []
    succ = {i: j for (i, j) in used}
    route = []
    cur = succ.get(0)
    while cur is not None and cur != 2 * n + 1:
        route.append(cur)
        cur = succ.get(cur)
    return route


def _walk_timing(route, k, Ek, d_0j_k, dij, ei, li, si):
    """Return list of (node, arrival, start_service, end_service) for each node."""
    if not route:
        return []
    records = []
    cur_time = Ek
    for idx, node in enumerate(route):
        if idx == 0:
            travel = d_0j_k.get(node, 0.0)
        else:
            travel = dij.get((route[idx - 1], node), 0.0)
        arrival = cur_time + travel
        start   = max(arrival, ei[node] - si[node])
        end     = start + si[node]
        records.append((node, arrival, start, end))
        cur_time = end
    return records


def print_routes_from_arcs(label, K, KC, A, n, Ek, Lk,
                            all_drivers_list, ei, li, si, dij, dijk_dict,
                            arcs_by_k, timing_by_k=None):
    """
    Print a human-readable route summary.

    arcs_by_k  : dict  k -> {(i,j): float}   (arc values; >0.5 = used)
    timing_by_k: dict  k -> list of (node, arrival, start, end)
                       If None, timing is re-simulated from arcs.
    """
    P_set = set(range(1, n + 1))
    D_set = set(range(n + 1, 2 * n + 1))

    print(f"\n{'─'*60}")
    print(f"  {label}")
    print(f"{'─'*60}")

    for k in K:
        drv = all_drivers_list[k]
        route = _reconstruct_route(k, arcs_by_k.get(k, {}), A, n)

        if timing_by_k is not None and k in timing_by_k:
            timing = timing_by_k[k]
        else:
            # simulate from arc data
            d_0j_k = {j: dijk_dict.get((0, j, k), 0.0) for j in P_set}
            timing = _walk_timing(route, k, Ek[k], d_0j_k, dij, ei, li, si)

        # Timing summary from simulation/MIP
        if timing:
            t_start = timing[0][1]          # arrival at first node
            t_end   = timing[-1][3]         # end of service at last node
        else:
            t_start = Ek[k]
            t_end   = Ek[k]

        print(f"\nCourier {k} ({'CDR' if k in KC else 'ODR'}):")
        print(f"  Start time: {t_start:7.2f}  (Earliest allowed {Ek[k]:.2f})")
        print(f"  End time:   {t_end:7.2f}  (Latest allowed   {Lk[k]:.2f})")

        served_reqs = [node for node in route if node in P_set]
        if served_reqs:
            print(f"  Served requests:")
            timing_map = {rec[0]: rec for rec in timing}
            for req in served_reqs:
                dlv = req + n
                p_rec = timing_map.get(req)
                d_rec = timing_map.get(dlv)
                p_t   = f"{p_rec[3]:.2f}" if p_rec else "?"
                d_t   = f"{d_rec[3]:.2f}" if d_rec else "?"
                print(f"    Request {req:3d}: "
                      f"Pickup at {p_t} (TW [{ei[req]:.2f},{li[req]:.2f}]),  "
                      f"Delivery at {d_t} (TW [{ei[dlv]:.2f},{li[dlv]:.2f}])")
        else:
            print(f"  Served requests: (none)")

        # Route with load annotation
        arcs_k = arcs_by_k.get(k, {})
        route_str = "0"
        cur = 0
        visited = {0}
        while True:
            nxt = None
            for (i, j) in A:
                if i == cur and arcs_k.get((i, j), 0.0) > 0.5 and j not in visited:
                    nxt = j
                    break
            if nxt is None:
                break
            visited.add(nxt)
            route_str += f" -> {nxt}"
            cur = nxt
            if cur == 2 * n + 1:
                break
        print(f"  Route: {route_str}")

    print(f"{'─'*60}\n")


# ──────────────────────────────────────────────────────────────────────────────


def solve_instance_comparison(instance_path: str,
                               max_iters: int = 100,
                               sub_time_limit: int = 60,
                               lp_time_limit: int = 300,
                               lambda0: float = 2.0,
                               max_noimpv: int = 7,
                               lambda_min: float = 1e-4,
                               use_graphsearch: bool = False,
                               lns_time_limit: int = 300,            # LNS
                               fix_singleton_routes: bool = False,    # LNS
                               lns_every_iter: bool = False,          # LNS
                               lns_overlap_threshold: float = 0.2,   # LNS-ST
                               lns_free_budget_frac: float = 1.0,    # LNS-BUD
                               print_routes: bool = False,            # DBG
                               verbose: bool = True):
    """
    Solve ONE instance using either Gurobi or GraphSearch for subproblems.
    Returns detailed comparison data.

    Args:
        use_graphsearch: If True, use GraphSearch for the subproblems; if False, use Gurobi
    New params (LNS):
        lns_time_limit:        Gurobi time limit for the LNS repair MIP.
        fix_singleton_routes:  If True, also fix x arcs for couriers whose served requests
                               are all singly assigned (requires Gurobi subproblem solver).
        lns_every_iter:        If True, also run LNS after each subgradient iteration.
    New return-dict keys (LNS):
        "best_primal"  – best feasible objective value found by LNS repair
        "best_gap"     – duality gap (%) = (best_L - best_primal)/|best_primal|*100
    """

    solver_name = "GraphSearch" if use_graphsearch else "Gurobi"
    t0 = time.time()

    # -----------------------
    # Load instance
    # -----------------------
    if verbose:
        print(f"\n=== Loading instance: {instance_path} ===")
        print(f"Solver: {solver_name}")

    instance = load_instance_from_txt(instance_path)

    b = int(instance["b"])
    H = float(instance["H"])
    num_comm_drivers = len(instance["committed"])
    num_occ_drivers = len(instance["occasional"])
    num_requests = len(instance["requests"])

    # Node indexing
    P = list(range(1, num_requests + 1))
    D = [i + num_requests for i in P]
    KC = list(range(0, num_comm_drivers))
    KO = list(range(num_comm_drivers, num_comm_drivers + num_occ_drivers))
    n = len(P)
    V = [0] + P + D + [2 * n + 1]
    K = KC + KO

    # Arcs
    A1 = [(0, j) for j in P]
    A2 = [(i, j) for i in P + D for j in P + D if i != j]
    A3 = [(i, 2 * n + 1) for i in D] + [(0, 2 * n + 1)]

    # Build coords & driver initial locations
    coords = {}
    for req in instance["requests"]:
        pid = req["id"]
        coords[pid] = req["pickup"]
        coords[pid + num_requests] = req["dest"]

    ok = [d["location"] for d in instance["committed"]] + [d["location"] for d in instance["occasional"]]

    # Distances / times
    dij_A2 = {(i, j): math.dist(coords[i], coords[j]) for (i, j) in A2}
    dij_A3 = {(i, j): 0 for (i, j) in A3}
    dij = {**dij_A2, **dij_A3}
    dijk = {(i, j, k): math.dist(ok[k], coords[j]) for (i, j) in A1 for k in K}
    tij = dij.copy()
    tijk = dijk.copy()

    # Time windows, service times, demand
    si = {0: 0, 2 * num_requests + 1: 0}
    ei, li, dem = {}, {}, {}
    for req in instance["requests"]:
        pid = req["id"]
        ei[pid], li[pid] = req["e_pickup"], req["l_pickup"]
        ei[pid + num_requests], li[pid + num_requests] = req["e_dest"], req["l_dest"]
        si[pid] = req["delay_p"]
        si[pid + num_requests] = req["delay_d"]
        dem[pid], dem[pid + num_requests] = req["l_dem"], -req["l_dem"]
    dem[0] = 0
    dem[2 * n + 1] = 0

    Ek = {d["id"]: d["e"] for d in instance["committed"]} | {d["id"]: d["e"] for d in instance["occasional"]}
    Lk = {d["id"]: d["l"] for d in instance["committed"]} | {d["id"]: d["l"] for d in instance["occasional"]}
    Qk = {d["id"]: d["cap"] for d in instance["committed"]} | {d["id"]: d["cap"] for d in instance["occasional"]}

    p = {i: instance["rev_order"] for i in P}

    wC = instance["c_com_h"] / b
    cO = instance["c_occ_o"]
    deltaC = instance["c_com_d"]
    deltaO = instance["c_occ_d"]

    # Pruning unfeasible links and assignments
    # This takes care of courier-request combos that are infeasible
    z_UB = {
        (pickup, driver): (
            1 if (li[pickup] - si[pickup] >= max(dijk[0, pickup, driver] + Ek[driver], ei[pickup])
                  and Lk[driver] >= ei[pickup + num_requests] + si[pickup + num_requests]) else 0
        )
        for driver in K for pickup in P
    }

    # This takes care of cases where j cannot be after i due to travel time restrictions or going from a delivery node back to its pickup
    # x_UB indicates if visiting j from i is ever feasible based on (1) time-windows (infeasible in the best-case scenario) and (2) going from a delivery node back to its own pickup
    x_UB = {
        (i, j): (0 if ((ei[i] + dij[i, j] + si[j] > li[j]) or (i in D and j == i - num_requests)) else 1)
        for (i, j) in A2
    }
    # UBxIPC1 indicates that if a node d2 is visited from a node n2, then if it is not feasible to go from d2's pickup to n2 directly, then n2 -> d2 cannot be part of a feasible path
    UBxIPC1 = {(n2, d2): 0 if (d2 in D and d2 - num_requests != n2 and x_UB[d2 - num_requests, n2] == 0) else 1 for (n2, d2) in A2}
    # UBxIPC2 indicates that if you leave p2 (pickup) and go to n2, then if it is not feasible to go from n2 to p2's corresponding delivery node directly, then p2 -> n2 cannot be part of a feasible path
    UBxIPC2 = {(p2, n2): 0 if (p2 in P and p2 + num_requests != n2 and x_UB[n2, p2 + num_requests] == 0) else 1 for (p2, n2) in A2}
    x_ind = {(i, j): x_UB[i, j] * UBxIPC1[i, j] * UBxIPC2[i, j] for (i, j) in A2}

    A2 = [(i, j) for (i, j) in A2 if x_ind[i, j] == 1]
    A = A1 + A2 + A3
    PK = [(pp, kk) for pp in P for kk in K if z_UB[pp, kk] == 1]
    Lk_org = Lk.copy()

    if verbose:
        print(f"Requests={num_requests}, Committed={num_comm_drivers}, Occasional={num_occ_drivers}, Vehicles={len(K)}")
        print(f"Arcs: |A1|={len(A1)}, |A2| pruned={len(A2)}, |A3|={len(A3)}, |A| total={len(A)}")
        print("Solving LP relaxation ...")

    # -----------------------
    # LP Relaxation (always using Gurobi)
    # -----------------------
    def build_lp_model(timeLimit=300):
        m = gp.Model("LP_relax")
        m.Params.OutputFlag = 0
        if timeLimit is not None:
            m.Params.TimeLimit = timeLimit

        x = m.addVars(A, K, vtype=GRB.CONTINUOUS, name="x")
        Lvar = m.addVars(A2, K, vtype=GRB.CONTINUOUS, name="L")
        y = m.addVars(P + D, K, vtype=GRB.CONTINUOUS, name="y")
        T = m.addVars(P + D + [0, 2 * n + 1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")

        obj_expr = gp.quicksum(p[i] * gp.quicksum(y[i, k] for k in K) for i in P)

        for k in KC:
            term = wC * (Lk_org[k] - Ek[k]) + deltaC * (
                gp.quicksum(dijk[0, j, k] * x[0, j, k] for (i, j) in A1) +
                gp.quicksum(dij[i, j] * x[i, j, k] for (i, j) in A2)
            )
            obj_expr -= term

        for k in KO:
            term = cO * gp.quicksum(y[i, k] for i in P) + deltaO * gp.quicksum(
                dij[i, j] * x[i, j, k] for (i, j) in A2
            )
            obj_expr -= term

        m.setObjective(obj_expr, GRB.MAXIMIZE)

        # Constraints (same as original)
        for i in P:
            m.addConstr(gp.quicksum(x[i, j, k] for k in K for j in V if (i, j) in A) <= 1)

        for i in P + D:
            for k in K:
                m.addConstr(
                    gp.quicksum(x[j, i, k] for j in V if (j, i) in A) -
                    gp.quicksum(x[i, j, k] for j in V if (i, j) in A) == 0
                )

        for i in P:
            for k in K:
                m.addConstr(
                    gp.quicksum(x[i, j, k] for j in V if (i, j) in A) -
                    gp.quicksum(x[j, i + num_requests, k] for j in V if (j, i + num_requests) in A) == 0
                )

        for k in K:
            m.addConstr(gp.quicksum(x[0, j, k] for j in V if (0, j) in A) == 1)
            m.addConstr(gp.quicksum(x[j, 2 * n + 1, k] for j in V if (j, 2 * n + 1) in A) == 1)
        for i in P + D:
            for k in K:
                m.addConstr(y[i, k] == gp.quicksum(x[i, l, k] for l in V if (i, l) in A))
        for i in P:
            for k in K:
                if z_UB[i, k] == 0:
                    m.addConstr(y[i, k] == 0)

        svMTZ = {(i, j): min(-si[i] - dij[j, i], ei[j] - ei[i]) for (i, j) in A2}

        for (i, j) in A2:
            for k in K:
                if (j, i) in A2:
                    m.addConstr(
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k] + svMTZ[i, j] * x[j, i, k]
                        <= li[i] * y[i, k] - ei[j] * y[j, k] - (li[i] - ei[j]) * (x[i, j, k] + x[j, i, k])
                    )
                else:
                    m.addConstr(
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k]
                        <= li[i] * y[i, k] - ei[j] * y[j, k] - (li[i] - ei[j]) * x[i, j, k]
                    )

        for (i, j) in A3:
            if i != 0:
                for k in K:
                    m.addConstr(
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k]
                        <= li[i] * y[i, k]
                        - Ek[k] * gp.quicksum(x[0, ii, k] for ii in P if (ii, k) in PK)
                        - (li[i] - Ek[k]) * x[i, j, k]
                    )

        for (i, j) in A1:
            for k in K:
                m.addConstr(
                    T[0, k] - T[j, k] + (tijk[0, j, k] + si[j]) * x[0, j, k]
                    <= Lk[k] * gp.quicksum(x[0, ii, k] for ii in P if (ii, k) in PK)
                    - ei[j] * y[j, k]
                    - (Lk[k] - ei[j]) * x[0, j, k]
                )

        for i in P + D:
            for k in K:
                m.addConstr(ei[i] * y[i, k] <= T[i, k])
                m.addConstr(T[i, k] <= li[i] * y[i, k])

        for i in P:
            for k in K:
                m.addConstr(T[i, k] + (tij[i, i + num_requests] + si[i + num_requests]) * y[i, k] <= T[i + num_requests, k])

        for k in K:
            m.addConstr(Ek[k] * gp.quicksum(x[0, i, k] for i in P if (i, k) in PK) <= T[0, k])
            m.addConstr(T[2 * n + 1, k] <= Lk[k] * gp.quicksum(x[0, i, k] for i in P if (i, k) in PK))

        for k in K:
            for i in P + D:
                m.addConstr(
                    gp.quicksum(Lvar[i, j, k] for j in P + D if (i, j) in A2) -
                    gp.quicksum(Lvar[j, i, k] for j in P + D if (j, i) in A2)
                    == dem[i] * gp.quicksum(x[i, j, k] for j in V if (i, j) in A)
                )
            for (i, j) in A2:
                m.addConstr(Lvar[i, j, k] >= max(dem[i], 0) * x[i, j, k])
                m.addConstr(Lvar[i, j, k] <= (Qk[k] + min(dem[i], 0)) * x[i, j, k])

        return m

    m_lp = build_lp_model(timeLimit=lp_time_limit)
    m_lp.optimize()

    if m_lp.status not in (GRB.OPTIMAL, GRB.TIME_LIMIT):
        if verbose:
            print(f"[LP] FAILED status={m_lp.status}")
        return {
            "instance": instance_path,
            "solver": solver_name,
            "UB_LP": None,
            "best_L": None,
            "best_primal": None,                                           # LNS
            "best_gap": None,                                              # LNS
            "iters_done": 0,
            "lambda_final": None,
            "total_time": time.time() - t0,
            "avg_subproblem_time": None,
            "iteration_log": [],                                           # LNS
            "status_msg": f"LP failed status={m_lp.status}"
        }

    UB_LP = float(m_lp.objVal)
    if verbose:
        print(f"[LP] UB_LP = {UB_LP:.6f}")

    # -----------------------
    # Gurobi Subproblem Builder (original)
    # -----------------------
    def build_vehicle_subproblem_gurobi(k, multipliers, timeLimit=60):
        m = gp.Model(f"veh_{k}")
        m.Params.OutputFlag = 0
        m.Params.TimeLimit = timeLimit

        x = m.addVars(A, vtype=GRB.BINARY, name="x")
        y = m.addVars(P + D, vtype=GRB.BINARY, name="y")
        T = m.addVars(P + D + [0, 2 * n + 1], vtype=GRB.CONTINUOUS, lb=0, name="T")
        Lvar = m.addVars(A2, vtype=GRB.CONTINUOUS, name="L")

        obj = gp.quicksum((p[i] - multipliers.get(i, 0.0)) * y[i] for i in P)

        if k in KC:
            obj -= wC * (Lk_org[k] - Ek[k])
            obj -= deltaC * (
                gp.quicksum(dijk[0, j, k] * x[0, j] for (i, j) in A1) +
                gp.quicksum(dij[i, j] * x[i, j] for (i, j) in A2)
            )
        else:
            obj -= cO * gp.quicksum(y[i] for i in P)
            obj -= deltaO * gp.quicksum(dij[i, j] * x[i, j] for (i, j) in A2)

        m.setObjective(obj, GRB.MAXIMIZE)

        for i in P + D:
            m.addConstr(
                gp.quicksum(x[j, i] for j in V if (j, i) in A) -
                gp.quicksum(x[i, j] for j in V if (i, j) in A) == 0
            )

        for i in P:
            m.addConstr(
                gp.quicksum(x[i, j] for j in V if (i, j) in A) -
                gp.quicksum(x[j, i + num_requests] for j in V if (j, i + num_requests) in A) == 0
            )

        m.addConstr(gp.quicksum(x[0, j] for j in V if (0, j) in A) == 1)
        m.addConstr(gp.quicksum(x[j, 2 * n + 1] for j in V if (j, 2 * n + 1) in A) == 1)

        for i in P + D:
            m.addConstr(y[i] == gp.quicksum(x[i, l] for l in V if (i, l) in A))
        for i in P:
            if z_UB[i, k] == 0:
                m.addConstr(y[i] == 0)

        svMTZ = {(i, j): min(-si[i] - dij[j, i], ei[j] - ei[i]) for (i, j) in A2}

        for (i, j) in A2:
            if (j, i) in A2:
                m.addConstr(
                    T[i] - T[j] + (tij[i, j] + si[j]) * x[i, j] + svMTZ[i, j] * x[j, i]
                    <= li[i] * y[i] - ei[j] * y[j] - (li[i] - ei[j]) * (x[i, j] + x[j, i])
                )
            else:
                m.addConstr(
                    T[i] - T[j] + (tij[i, j] + si[j]) * x[i, j]
                    <= li[i] * y[i] - ei[j] * y[j] - (li[i] - ei[j]) * x[i, j]
                )

        for (i, j) in A3:
            if i != 0:
                m.addConstr(
                    T[i] - T[j] + (tij[i, j] + si[j]) * x[i, j]
                    <= li[i] * y[i]
                    - Ek[k] * gp.quicksum(x[0, ii] for ii in P if (ii, k) in PK)
                    - (li[i] - Ek[k]) * x[i, j]
                )

        for (i, j) in A1:
            m.addConstr(
                T[0] - T[j] + (tijk[0, j, k] + si[j]) * x[0, j]
                <= Lk[k] * gp.quicksum(x[0, ii] for ii in P if (ii, k) in PK)
                - ei[j] * y[j]
                - (Lk[k] - ei[j]) * x[0, j]
            )

        for i in P + D:
            m.addConstr(ei[i] * y[i] <= T[i])
            m.addConstr(T[i] <= li[i] * y[i])

        for i in P:
            m.addConstr(T[i] + (tij[i, i + num_requests] + si[i + num_requests]) * y[i] <= T[i + num_requests])

        m.addConstr(Ek[k] * gp.quicksum(x[0, i] for i in P if (i, k) in PK) <= T[0])
        m.addConstr(T[2 * n + 1] <= Lk[k] * gp.quicksum(x[0, i] for i in P if (i, k) in PK))

        for i in P + D:
            m.addConstr(
                gp.quicksum(Lvar[i, j] for j in P + D if (i, j) in A2) -
                gp.quicksum(Lvar[j, i] for j in P + D if (j, i) in A2)
                == dem[i] * gp.quicksum(x[i, j] for j in V if (i, j) in A)
            )
        for (i, j) in A2:
            m.addConstr(Lvar[i, j] >= max(dem[i], 0) * x[i, j])
            m.addConstr(Lvar[i, j] <= (Qk[k] + min(dem[i], 0)) * x[i, j])

        return m, {"y": y, "x": x}                                        # LNS: also return x

    # -----------------------                                              # LNS
    # LNS: feasibility repair MIP                                         # LNS
    # -----------------------                                              # LNS
    def build_lns_repair_mip(served_by, x_sol=None,                       # LNS
                              fix_singleton_routes=False,                   # LNS
                              time_limit=300, verbose_lns=False,            # LNS
                              overlap_threshold=0.05,                        # LNS-ST
                              free_budget_frac=1.0,                          # LNS-BUD
                              _print_routes=False):
        """                                                                # LNS
        Build and solve a full PDPTW MIP (binary vars) for feasibility    # LNS
        repair after the Lagrangian subgradient.                           # LNS
                                                                           # LNS
        Conflict-driven fixing rule                                        # LNS-ST
        ----------------------------                                       # LNS-ST
        conflict_set = duplicated requests (served by 2+ couriers)        # LNS-ST
                     + unserved requests.                                  # LNS-ST
        culprit_couriers = couriers with duplicately-served requests.      # LNS-ST
        - y[i,k]: if i in conflict_set → free for k where z_UB[(i,k)]=1, # LNS-ST
                                          fixed to 0 otherwise.            # LNS-ST
                  if i singleton        → fix non-culprit k to Lagrangian  # LNS-ST
                                          value; leave culprit k free.     # LNS-ST
        - if fix_singleton_routes: fix ALL arcs for non-culprit couriers; # LNS-ST
          leave ALL arcs free for culprit couriers.                        # LNS-ST
                                                                           # LNS
        Returns float | None                                               # LNS
        """                                                                # LNS
        m = gp.Model("lns_repair")                                        # LNS
        m.Params.OutputFlag = 1 if verbose_lns else 0                     # LNS
        m.Params.TimeLimit = time_limit                                    # LNS

        x    = m.addVars(A, K, vtype=GRB.BINARY,     name="x")           # LNS
        Lvar = m.addVars(A2, K, vtype=GRB.CONTINUOUS, name="L")          # LNS
        y    = m.addVars(P + D, K, vtype=GRB.BINARY,  name="y")          # LNS
        T    = m.addVars(P + D + [0, 2 * n + 1], K,                      # LNS
                         vtype=GRB.CONTINUOUS, lb=0,   name="T")          # LNS

        # Objective (same structure as build_lp_model but binary)         # LNS
        obj_expr = gp.quicksum(p[i] * gp.quicksum(y[i, k] for k in K)   # LNS
                               for i in P)                                 # LNS
        for k in KC:                                                       # LNS
            term = wC * (Lk_org[k] - Ek[k]) + deltaC * (                 # LNS
                gp.quicksum(dijk[0, j, k] * x[0, j, k] for (_, j) in A1) +  # LNS
                gp.quicksum(dij[i, j]     * x[i, j, k] for (i, j) in A2)    # LNS
            )                                                              # LNS
            obj_expr -= term                                               # LNS
        for k in KO:                                                       # LNS
            term = cO * gp.quicksum(y[i, k] for i in P) + deltaO * (     # LNS
                gp.quicksum(dij[i, j] * x[i, j, k]                       # LNS
                            for (i, j) in A2)                             # LNS
            )                                                              # LNS
            obj_expr -= term                                               # LNS
        m.setObjective(obj_expr, GRB.MAXIMIZE)                            # LNS

        # Standard routing constraints                                     # LNS
        for i in P:                                                        # LNS
            m.addConstr(gp.quicksum(x[i, j, k] for k in K                # LNS
                                    for j in V if (i, j) in A) <= 1)     # LNS
        for i in P + D:                                                    # LNS
            for k in K:                                                    # LNS
                m.addConstr(                                               # LNS
                    gp.quicksum(x[j, i, k] for j in V if (j, i) in A) -  # LNS
                    gp.quicksum(x[i, j, k] for j in V if (i, j) in A)   # LNS
                    == 0)                                                  # LNS
        for i in P:                                                        # LNS
            for k in K:                                                    # LNS
                m.addConstr(                                               # LNS
                    gp.quicksum(x[i, j, k] for j in V if (i, j) in A) -  # LNS
                    gp.quicksum(x[j, i + num_requests, k]                 # LNS
                                for j in V if (j, i + num_requests) in A) # LNS
                    == 0)                                                  # LNS
        for k in K:                                                        # LNS
            m.addConstr(gp.quicksum(x[0, j, k]                           # LNS
                                    for j in V if (0, j) in A) == 1)     # LNS
            m.addConstr(gp.quicksum(x[j, 2 * n + 1, k]                   # LNS
                                    for j in V if (j, 2 * n + 1) in A)   # LNS
                        == 1)                                              # LNS
        for i in P + D:                                                    # LNS
            for k in K:                                                    # LNS
                m.addConstr(y[i, k] ==                                    # LNS
                            gp.quicksum(x[i, l, k] for l in V            # LNS
                                        if (i, l) in A))                  # LNS
        for i in P:                                                        # LNS
            for k in K:                                                    # LNS
                if z_UB[i, k] == 0:                                       # LNS
                    m.addConstr(y[i, k] == 0)                             # LNS

        svMTZ = {(i, j): min(-si[i] - dij[j, i], ei[j] - ei[i])         # LNS
                 for (i, j) in A2}                                        # LNS
        for (i, j) in A2:                                                  # LNS
            for k in K:                                                    # LNS
                if (j, i) in A2:                                           # LNS
                    m.addConstr(                                           # LNS
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k] +  # LNS
                        svMTZ[i, j] * x[j, i, k]                         # LNS
                        <= li[i] * y[i, k] - ei[j] * y[j, k]            # LNS
                        - (li[i] - ei[j]) * (x[i, j, k] + x[j, i, k])) # LNS
                else:                                                      # LNS
                    m.addConstr(                                           # LNS
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k] # LNS
                        <= li[i] * y[i, k] - ei[j] * y[j, k]            # LNS
                        - (li[i] - ei[j]) * x[i, j, k])                  # LNS
        for (i, j) in A3:                                                  # LNS
            if i != 0:                                                     # LNS
                for k in K:                                                # LNS
                    m.addConstr(                                           # LNS
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k] # LNS
                        <= li[i] * y[i, k]                                # LNS
                        - Ek[k] * gp.quicksum(x[0, ii, k] for ii in P   # LNS
                                              if (ii, k) in PK)           # LNS
                        - (li[i] - Ek[k]) * x[i, j, k])                  # LNS
        for (i, j) in A1:                                                  # LNS
            for k in K:                                                    # LNS
                m.addConstr(                                               # LNS
                    T[0, k] - T[j, k] + (tijk[0, j, k] + si[j]) * x[0, j, k] # LNS
                    <= Lk[k] * gp.quicksum(x[0, ii, k] for ii in P      # LNS
                                           if (ii, k) in PK)              # LNS
                    - ei[j] * y[j, k]                                     # LNS
                    - (Lk[k] - ei[j]) * x[0, j, k])                      # LNS
        for i in P + D:                                                    # LNS
            for k in K:                                                    # LNS
                m.addConstr(ei[i] * y[i, k] <= T[i, k])                  # LNS
                m.addConstr(T[i, k] <= li[i] * y[i, k])                  # LNS
        for i in P:                                                        # LNS
            for k in K:                                                    # LNS
                m.addConstr(T[i, k] +                                     # LNS
                            (tij[i, i + num_requests] +                   # LNS
                             si[i + num_requests]) * y[i, k]              # LNS
                            <= T[i + num_requests, k])                    # LNS
        for k in K:                                                        # LNS
            m.addConstr(Ek[k] * gp.quicksum(x[0, i, k] for i in P       # LNS
                                            if (i, k) in PK) <= T[0, k]) # LNS
            m.addConstr(T[2 * n + 1, k] <=                                # LNS
                        Lk[k] * gp.quicksum(x[0, i, k] for i in P       # LNS
                                            if (i, k) in PK))             # LNS
        for k in K:                                                        # LNS
            for i in P + D:                                                # LNS
                m.addConstr(                                               # LNS
                    gp.quicksum(Lvar[i, j, k] for j in P + D             # LNS
                                if (i, j) in A2) -                        # LNS
                    gp.quicksum(Lvar[j, i, k] for j in P + D             # LNS
                                if (j, i) in A2)                          # LNS
                    == dem[i] * gp.quicksum(x[i, j, k] for j in V        # LNS
                                            if (i, j) in A))              # LNS
            for (i, j) in A2:                                             # LNS
                m.addConstr(Lvar[i, j, k] >= max(dem[i], 0) * x[i, j, k])  # LNS
                m.addConstr(Lvar[i, j, k] <=                              # LNS
                            (Qk[k] + min(dem[i], 0)) * x[i, j, k])       # LNS

        # ── Stage 1: overlap-based candidate selection ──────────────── # LNS-BUD
        duplicates = [i for i in P                                        # LNS-BUD
                      if sum(1 for k in K if served_by[i][k] > 0.5) > 1] # LNS-BUD
        unserved   = [i for i in P                                        # LNS-BUD
                      if sum(1 for k in K if served_by[i][k] > 0.5) == 0]# LNS-BUD
        conflict_set = set(duplicates) | set(unserved)                    # LNS-BUD

        def _tw_overlap_amount(e1, l1, e2, l2):                           # LNS-BUD
            return max(0.0, min(l1, l2) - max(e1, e2))                   # LNS-BUD

        def _tw_overlap_sufficient(e1, l1, e2, l2, ref_e, ref_l):        # LNS-BUD
            if overlap_threshold <= 0.0:                                  # LNS-BUD
                return _tw_overlap_amount(e1, l1, e2, l2) > 0            # LNS-BUD
            ref_width = ref_l - ref_e                                     # LNS-BUD
            if ref_width <= 0:                                            # LNS-BUD
                return False                                              # LNS-BUD
            return _tw_overlap_amount(e1, l1, e2, l2) >= overlap_threshold * ref_width  # LNS-BUD

        free_requests = set(conflict_set)                                 # LNS-BUD
        for i in P:                                                       # LNS-BUD
            if i not in conflict_set:                                     # LNS-BUD
                for d in duplicates:                                      # LNS-BUD
                    if any(                                               # LNS-BUD
                        _tw_overlap_sufficient(e1, l1, e2, l2, e2, l2)  # LNS-BUD
                        for e1, l1 in [(ei[i],   li[i]),                 # LNS-BUD
                                       (ei[i+n], li[i+n])]               # LNS-BUD
                        for e2, l2 in [(ei[d],   li[d]),                 # LNS-BUD
                                       (ei[d+n], li[d+n])]               # LNS-BUD
                    ):                                                    # LNS-BUD
                        free_requests.add(i)                              # LNS-BUD
                        break                                             # LNS-BUD

        free_couriers = {                                                  # LNS-BUD
            k for k in K for d in conflict_set                            # LNS-BUD
            if z_UB.get((d, k), 0) == 1                                  # LNS-BUD
            and _tw_overlap_sufficient(                                   # LNS-BUD
                Ek[k], Lk[k], ei[d], li[d], ei[d], li[d])               # LNS-BUD
        }                                                                  # LNS-BUD

        # ── Stage 2: score candidates and apply budget ───────────── # LNS-BUD
        # s_req(i): max normalised 4-combo overlap with any conflict   # LNS-BUD
        # request; 1.0 if i is itself in conflict_set.                # LNS-BUD
        def _score_request(i):                                         # LNS-BUD
            if i in conflict_set:                                      # LNS-BUD
                return 1.0                                             # LNS-BUD
            best = 0.0                                                 # LNS-BUD
            for d in duplicates:                                     # LNS-BUD
                w_d = li[d] - ei[d]                                   # LNS-BUD
                if w_d <= 0:                                           # LNS-BUD
                    continue                                           # LNS-BUD
                for e1, l1 in [(ei[i], li[i]), (ei[i+n], li[i+n])]:  # LNS-BUD
                    for e2, l2 in [(ei[d], li[d]), (ei[d+n], li[d+n])]:  # LNS-BUD
                        ov = _tw_overlap_amount(e1, l1, e2, l2) / w_d  # LNS-BUD
                        if ov > best:                                  # LNS-BUD
                            best = ov                                  # LNS-BUD
            return best                                                # LNS-BUD

        # s_cou(k): max normalised overlap of courier window with any  # LNS-BUD
        # conflict pickup TW, gated by z_UB=1.                        # LNS-BUD
        def _score_courier(k):                                         # LNS-BUD
            best = 0.0                                                 # LNS-BUD
            for d in duplicates:                                     # LNS-BUD
                if z_UB.get((d, k), 0) != 1:                          # LNS-BUD
                    continue                                           # LNS-BUD
                w_d = li[d] - ei[d]                                   # LNS-BUD
                if w_d <= 0:                                           # LNS-BUD
                    continue                                           # LNS-BUD
                ov = _tw_overlap_amount(Ek[k], Lk[k], ei[d], li[d]) / w_d  # LNS-BUD
                if ov > best:                                          # LNS-BUD
                    best = ov                                          # LNS-BUD
            return best                                                # LNS-BUD

        total_y   = len(P) * len(K)                                    # LNS-BUD
        feasible_pairs = [(i, k) for i in P for k in K                # LNS-BUD
                          if z_UB.get((i, k), 0) == 1]                # LNS-BUD
        z_ub_ones = len(feasible_pairs)                                # LNS-BUD

        # Candidate pairs: pass overlap filter                         # LNS-BUD
        candidate_pairs = [(i, k) for (i, k) in feasible_pairs        # LNS-BUD
                           if i in free_requests or k in free_couriers]# LNS-BUD

        # Score and sort descending                                    # LNS-BUD
        req_scores = {i: _score_request(i) for i in P}                # LNS-BUD
        cou_scores = {k: _score_courier(k) for k in K}                # LNS-BUD
        candidate_pairs.sort(                                          # LNS-BUD
            key=lambda ik: req_scores[ik[0]] * cou_scores[ik[1]],    # LNS-BUD
            reverse=True,                                              # LNS-BUD
        )                                                              # LNS-BUD

        # Budget: free at most free_budget_frac * z_ub_ones pairs     # LNS-BUD
        budget   = max(1, int(free_budget_frac * z_ub_ones))          # LNS-BUD
        free_set = set(candidate_pairs[:budget])                      # LNS-BUD

        # Apply fix / free to y variables                              # LNS-BUD
        n_fixed_y = 0                                                  # LNS-BUD
        n_free_y  = 0                                                  # LNS-BUD
        for i in P:                                                    # LNS-BUD
            singleton_k = None                                         # LNS-BUD
            if i not in conflict_set:                                  # LNS-BUD
                singleton_k = next(                                    # LNS-BUD
                    (kk for kk in K if served_by[i][kk] > 0.5), None)# LNS-BUD
            for k in K:                                                # LNS-BUD
                if z_UB.get((i, k), 0) == 0:                          # LNS-BUD
                    m.addConstr(y[i, k] == 0)                          # LNS-BUD
                elif (i, k) in free_set:                               # LNS-BUD
                    n_free_y += 1                                      # LNS-BUD
                else:                                                  # LNS-BUD
                    if singleton_k is not None:                        # LNS-BUD
                        m.addConstr(y[i, k] == (1 if k == singleton_k else 0))  # LNS-BUD
                    else:                                              # LNS-BUD
                        m.addConstr(y[i, k] == 0)                     # LNS-BUD
                    n_fixed_y += 1                                     # LNS-BUD

        # Arc fixing: fix arcs for couriers with no pair in free_set  # LNS-BUD
        n_fixed_arcs = 0                                               # LNS-BUD
        if fix_singleton_routes and x_sol is not None:                # LNS-BUD
            free_courier_set = {k for (_, k) in free_set}             # LNS-BUD
            for k in K:                                                # LNS-BUD
                if k in free_courier_set:                              # LNS-BUD
                    continue                                           # LNS-BUD
                arcs_k = x_sol.get(k, {})                             # LNS-BUD
                if not arcs_k:                                         # LNS-BUD
                    continue                                           # LNS-BUD
                for (i, j) in A:                                       # LNS-BUD
                    val = 1 if arcs_k.get((i, j), 0.0) > 0.5 else 0  # LNS-BUD
                    m.addConstr(x[i, j, k] == val)                    # LNS-BUD
                    n_fixed_arcs += 1                                  # LNS-BUD

        if verbose_lns:                                                # LNS-BUD
            print(f"  [LNS-BUD] duplicates={len(duplicates)}"         # LNS-BUD
                  f" unserved={len(unserved)}"                         # LNS-BUD
                  f" z_ub_ones={z_ub_ones}"                            # LNS-BUD
                  f" candidates={len(candidate_pairs)}"                # LNS-BUD
                  f" budget={budget}"                                  # LNS-BUD
                  f" free_y={n_free_y} fixed_y={n_fixed_y}"           # LNS-BUD
                  f" fixed_arcs={n_fixed_arcs}")                       # LNS-BUD

        n_fixed_arcs_stat = n_fixed_arcs                               # LNS

        m.optimize()                                                       # LNS
        if m.SolCount > 0:                                                 # LNS
            if _print_routes:                                              # DBG
                # Build arc-value dicts from MIP solution                  # DBG
                arcs_mip = {                                               # DBG
                    k: {(i, j): x[i, j, k].X for (i, j) in A}            # DBG
                    for k in K                                             # DBG
                }                                                          # DBG
                # Build timing from T variables                            # DBG
                timing_mip = {}                                            # DBG
                for k in K:                                                # DBG
                    route = _reconstruct_route(k, arcs_mip[k], A, n)     # DBG
                    timing_mip[k] = [                                      # DBG
                        (nd, T[nd, k].X, T[nd, k].X,                     # DBG
                         T[nd, k].X + si.get(nd, 0))                     # DBG
                        for nd in route                                    # DBG
                    ]                                                      # DBG
                all_drivers = instance["committed"] + instance["occasional"]  # DBG
                print_routes_from_arcs(                                    # DBG
                    "After LNS repair", K, KC, A, n, Ek, Lk,             # DBG
                    all_drivers, ei, li, si, dij,                         # DBG
                    {(0, j, k): dijk[0, j, k] for (_, j) in A1 for k in K},  # DBG
                    arcs_mip, timing_mip,                                  # DBG
                )                                                          # DBG
            return {                                                        # LNS
                "objVal": float(m.objVal),                                 # LNS
                "total_y": total_y,                                        # LNS
                "z_ub_ones": z_ub_ones,                                    # LNS
                "n_fixed_y": n_fixed_y,                                    # LNS
                "n_free_y": n_free_y,                                      # LNS
                "n_fixed_arcs": n_fixed_arcs_stat,                         # LNS
            }                                                              # LNS
        return None                                                        # LNS

    # -----------------------
    # Lagrangian Subgradient
    # -----------------------
    u = {i: 0.0 for i in P}
    lambda_r = float(lambda0)
    no_improve_count = 0
    best_L = float("inf")
    best_primal = None                                                     # LNS
    it_done = 0
    subproblem_times = []
    served_by = {i: {k: 0.0 for k in K} for i in P}                      # LNS: initialise before loop (fallback for end-LNS)
    x_sol = {k: {} for k in K} if fix_singleton_routes else None          # LNS
    iteration_log = []                                                     # LNS

    if verbose:
        print(f"Starting subgradient method with {solver_name}...")
        print("Iter | L(u)           | best_L         | ||g||^2       | lambda   | noimp | avg_sub_time")

    for it in range(1, max_iters + 1):
        it_done = it
        sum_sub_obj = 0.0
        served_by = {i: {k: 0.0 for k in K} for i in P}                  # LNS: reset each iteration
        served_sum_by_i = {i: 0.0 for i in P}

        iter_sub_times = []

        for k in K:
            t_sub_start = time.time()

            if use_graphsearch:
                # Use GraphSearch
                gs_solver = GraphSearchSolver(instance, k, z_UB, x_UB, x_ind, u, verbose=False)
                result = gs_solver.solve()
                sub_obj = result['objVal']

                for pickup in result['served_pickups']:
                    served_sum_by_i[pickup] += 1.0
                    served_by[pickup][k] = 1.0                             # LNS
                if fix_singleton_routes and x_sol is not None:             # LNS
                    route = result['route']                                 # LNS
                    arc_vals = {}                                           # LNS
                    if route:                                               # LNS
                        arc_vals[(0, route[0])] = 1.0                      # LNS
                        for _idx in range(len(route) - 1):                 # LNS
                            arc_vals[(route[_idx], route[_idx + 1])] = 1.0 # LNS
                        arc_vals[(route[-1], 2 * n + 1)] = 1.0             # LNS
                    else:                                                   # LNS
                        arc_vals[(0, 2 * n + 1)] = 1.0                     # LNS
                    x_sol[k] = arc_vals                                    # LNS

            else:
                # Use Gurobi
                mk, vk = build_vehicle_subproblem_gurobi(k, u, timeLimit=sub_time_limit)
                mk.optimize()

                if mk.status not in (GRB.OPTIMAL, GRB.TIME_LIMIT, GRB.SUBOPTIMAL):
                    if verbose:
                        print(f"[LAG] vehicle {k} failed status={mk.status}")
                    return {
                        "instance": instance_path,
                        "solver": solver_name,
                        "UB_LP": UB_LP,
                        "best_L": None,
                        "best_primal": best_primal,                        # LNS
                        "best_gap": None,                                  # LNS
                        "iters_done": it_done,
                        "lambda_final": lambda_r,
                        "total_time": time.time() - t0,
                        "avg_subproblem_time": None,
                        "iteration_log": iteration_log,                    # LNS
                        "status_msg": f"veh {k} failed status={mk.status}"
                    }

                sub_obj = mk.objVal
                for i in P:
                    served_val = vk["y"][i].X
                    served_sum_by_i[i] += served_val
                    served_by[i][k] = served_val                            # LNS
                if fix_singleton_routes:                                    # LNS
                    x_sol[k] = {(i, j): vk["x"][i, j].X for (i, j) in A}    # LNS

            t_sub_end = time.time()
            iter_sub_times.append(t_sub_end - t_sub_start)
            sum_sub_obj += sub_obj

        avg_sub_time_this_iter = sum(iter_sub_times) / len(iter_sub_times)
        subproblem_times.extend(iter_sub_times)

        lagrangian_value = sum_sub_obj + sum(u.values())

        # Subgradient
        norm_g_sq = 0.0
        g = {}
        for i in P:
            g_i = 1.0 - served_sum_by_i[i]
            g[i] = g_i
            norm_g_sq += g_i * g_i

        improved = False
        if lagrangian_value < best_L - 1e-9:
            best_L = lagrangian_value
            improved = True
            no_improve_count = 0
        else:
            no_improve_count += 1

        if verbose:
            print(f"{it:4d} | {lagrangian_value:14.6f} | {best_L:14.6f} | {norm_g_sq:12.6f} | {lambda_r:7.4f} | {no_improve_count:5d} | {avg_sub_time_this_iter:.3f}s")

        lns_iter_time = 0.0                                                # LNS

        if norm_g_sq <= 1e-12:
            if verbose:
                print("Zero (or very small) subgradient -> stopping.")
            iteration_log.append({                                         # LNS
                "iter": it, "L_u": lagrangian_value, "best_L": best_L,   # LNS
                "best_primal": best_primal,                                # LNS
                "gap_pct": 100.0 * (best_L - best_primal) / abs(best_primal)  # LNS
                           if best_primal is not None and abs(best_primal) > 1e-9 else None,  # LNS
                "norm_g_sq": norm_g_sq, "lambda": lambda_r,               # LNS
                "no_improve": no_improve_count,                            # LNS
                "lns_iter_time": lns_iter_time,                            # LNS
                "avg_sub_time": avg_sub_time_this_iter,                   # LNS
                "lns_total_y": None, "lns_z_ub_ones": None,               # LNS
                "lns_z_ub_ones_pct": None, "lns_fixed_y": None,           # LNS
                "lns_fixed_y_pct": None, "lns_free_y": None,              # LNS
                "lns_free_y_pct": None, "lns_fixed_arcs": None,           # LNS
            })                                                             # LNS
            break

        # LNS within iterations (optional, controlled by lns_every_iter)  # LNS
        lns_stats = None                                                    # LNS
        if lns_every_iter:                                                 # LNS
            t_lns = time.time()                                            # LNS
            lns_result = build_lns_repair_mip(                             # LNS
                served_by=served_by,                                       # LNS
                x_sol=x_sol,                                               # LNS
                fix_singleton_routes=fix_singleton_routes,                 # LNS
                time_limit=lns_time_limit,                                 # LNS
                verbose_lns=False,                                         # LNS
                overlap_threshold=lns_overlap_threshold,                  # LNS-ST
                free_budget_frac=lns_free_budget_frac,                    # LNS-BUD
                _print_routes=print_routes,                                # DBG
            )                                                              # LNS
            pv = lns_result["objVal"] if lns_result is not None else None  # LNS
            lns_stats = lns_result                                         # LNS
            lns_iter_time = time.time() - t_lns                            # LNS
            if pv is not None and (best_primal is None or pv > best_primal):  # LNS
                best_primal = pv                                            # LNS
                if verbose:                                                # LNS
                    print(f"  [LNS iter {it}] New best primal: {best_primal:.6f}"  # LNS
                          f" ({lns_iter_time:.1f}s)")                      # LNS

        _zub = lns_stats["z_ub_ones"] if lns_stats is not None else None    # LNS
        iteration_log.append({                                             # LNS
            "iter": it, "L_u": lagrangian_value, "best_L": best_L,       # LNS
            "best_primal": best_primal,                                    # LNS
            "gap_pct": 100.0 * (best_L - best_primal) / abs(best_primal) # LNS
                       if best_primal is not None and abs(best_primal) > 1e-9 else None,  # LNS
            "norm_g_sq": norm_g_sq, "lambda": lambda_r,                   # LNS
            "no_improve": no_improve_count,                                # LNS
            "lns_iter_time": lns_iter_time,                                # LNS
            "avg_sub_time": avg_sub_time_this_iter,                       # LNS
            "lns_total_y": lns_stats["total_y"] if lns_stats is not None else None,  # LNS
            "lns_z_ub_ones": _zub,                                        # LNS
            "lns_z_ub_ones_pct": 100.0 * _zub / lns_stats["total_y"] if _zub is not None and lns_stats["total_y"] else None,  # LNS
            "lns_fixed_y": lns_stats["n_fixed_y"] if lns_stats is not None else None,  # LNS
            "lns_fixed_y_pct": 100.0 * lns_stats["n_fixed_y"] / _zub if lns_stats is not None and _zub else None,  # LNS
            "lns_free_y": lns_stats["n_free_y"] if lns_stats is not None else None,  # LNS
            "lns_free_y_pct": 100.0 * lns_stats["n_free_y"] / _zub if lns_stats is not None and _zub else None,  # LNS
            "lns_fixed_arcs": lns_stats["n_fixed_arcs"] if lns_stats is not None else None,  # LNS
        })                                                                 # LNS

        # Convergence: primal matches dual bound
        if best_primal is not None and best_L - best_primal <= 1e-6 * max(1.0, abs(best_primal)):   # LNS
            if verbose:                                                                             # LNS
                print(f"  --> Gap closed. Stopping.")                                               # LNS
            break                                                                                   # LNS

        # Subgradient step
        if lns_every_iter and best_primal is not None:                    # LNS
            surrogate_primal = best_primal                                 # LNS
        else:                                                              # LNS
            surrogate_primal = -abs(UB_LP)
        numerator = lagrangian_value - surrogate_primal
        step = lambda_r * numerator / norm_g_sq

        for i in P:
            u[i] = max(0.0, u[i] - step * g[i])

        # Reduce lambda if no improvement
        if (not improved) and (no_improve_count >= max_noimpv):
            lambda_r *= 0.5
            no_improve_count = 0
            if verbose:
                print(f"  --> No improvement: reducing lambda to {lambda_r:.6f}")
            if lambda_r < lambda_min:
                if verbose:
                    print("  --> lambda too small. Stopping.")
                break

    # ----------------------------------------------------------------    # LNS
    # LNS – feasibility repair using the final subgradient iterate        # LNS
    # ----------------------------------------------------------------    # LNS
    if print_routes:                                                       # DBG
        _lag_arcs = x_sol if x_sol else {                                 # DBG
            k: {(i, j): (1.0 if served_by.get(i, {}).get(k, 0) > 0.5    # DBG
                          and i in range(1, n + 1) else 0.0)             # DBG
                for (i, j) in A}                                          # DBG
            for k in K                                                     # DBG
        }                                                                  # DBG
        print_routes_from_arcs(                                            # DBG
            "Before LNS repair (Lagrangian iterate)",                     # DBG
            K, KC, A, n, Ek, Lk,                                         # DBG
            instance["committed"] + instance["occasional"],               # DBG
            ei, li, si, dij,                                              # DBG
            {(0, j, k): dijk[0, j, k] for (_, j) in A1 for k in K},     # DBG
            _lag_arcs,                                                     # DBG
        )                                                                  # DBG
    t_lns_start = time.time()                                             # LNS
    lns_result_end = build_lns_repair_mip(                                # LNS
        served_by=served_by,                                              # LNS
        x_sol=x_sol,                                                      # LNS
        fix_singleton_routes=fix_singleton_routes,                        # LNS
        time_limit=lns_time_limit,                                        # LNS
        verbose_lns=verbose,                                              # LNS
        overlap_threshold=lns_overlap_threshold,                         # LNS-ST
        free_budget_frac=lns_free_budget_frac,                           # LNS-BUD
        _print_routes=print_routes,                                       # DBG
    )                                                                     # LNS
    primal_val = lns_result_end["objVal"] if lns_result_end is not None else None  # LNS
    lns_stats_end = lns_result_end                                        # LNS
    t_lns_end = time.time()                                               # LNS
    if primal_val is not None:                                            # LNS
        if best_primal is None or primal_val > best_primal:               # LNS
            best_primal = primal_val                                       # LNS
    if verbose:                                                           # LNS
        print(f"  [LNS end] best_primal={best_primal} "                   # LNS
              f"(took {t_lns_end - t_lns_start:.1f}s)")                   # LNS
    _zub_end = lns_stats_end["z_ub_ones"] if lns_stats_end is not None else None  # LNS
    iteration_log.append({                                                 # LNS
        "iter": "END_LNS", "L_u": best_L, "best_L": best_L,             # LNS
        "best_primal": best_primal,                                        # LNS
        "gap_pct": 100.0 * (best_L - best_primal) / abs(best_primal)     # LNS
                   if best_primal is not None and abs(best_primal) > 1e-9 else None,  # LNS
        "norm_g_sq": None, "lambda": lambda_r, "no_improve": None,       # LNS
        "lns_iter_time": t_lns_end - t_lns_start, "avg_sub_time": None,  # LNS
        "lns_total_y": lns_stats_end["total_y"] if lns_stats_end is not None else None,  # LNS
        "lns_z_ub_ones": _zub_end,                                        # LNS
        "lns_z_ub_ones_pct": 100.0 * _zub_end / lns_stats_end["total_y"] if _zub_end is not None and lns_stats_end["total_y"] else None,  # LNS
        "lns_fixed_y": lns_stats_end["n_fixed_y"] if lns_stats_end is not None else None,  # LNS
        "lns_fixed_y_pct": 100.0 * lns_stats_end["n_fixed_y"] / _zub_end if lns_stats_end is not None and _zub_end else None,  # LNS
        "lns_free_y": lns_stats_end["n_free_y"] if lns_stats_end is not None else None,  # LNS
        "lns_free_y_pct": 100.0 * lns_stats_end["n_free_y"] / _zub_end if lns_stats_end is not None and _zub_end else None,  # LNS
        "lns_fixed_arcs": lns_stats_end["n_fixed_arcs"] if lns_stats_end is not None else None,  # LNS
    })                                                                     # LNS
    # ----------------------------------------------------------------    # LNS

    elapsed = time.time() - t0
    avg_subproblem_time = sum(subproblem_times) / len(subproblem_times) if subproblem_times else None

    if best_primal is not None and abs(best_primal) > 1e-9:               # LNS
        best_gap = 100.0 * (best_L - best_primal) / abs(best_primal)      # LNS
    else:                                                                 # LNS
        best_gap = None                                                   # LNS

    if verbose:
        print(f"Done. best_L={best_L:.6f}, UB_LP={UB_LP:.6f}, iters={it_done}, lambda_final={lambda_r:.6f}")
        print(f"Total time={elapsed:.1f}s, Avg subproblem time={avg_subproblem_time:.3f}s")

    return {
        "instance": instance_path,
        "solver": solver_name,
        "UB_LP": UB_LP,
        "best_L": float(best_L),
        "best_primal": best_primal,                                        # LNS
        "best_gap": best_gap,                                              # LNS
        "iters_done": it_done,
        "lambda_final": float(lambda_r),
        "total_time": elapsed,
        "avg_subproblem_time": avg_subproblem_time,
        "iteration_log": iteration_log,                                    # LNS
        "status_msg": "OK"
    }


def autosize_columns(ws):
    for col in range(1, ws.max_column + 1):
        max_len = 0
        col_letter = get_column_letter(col)
        for row in range(1, ws.max_row + 1):
            v = ws.cell(row=row, column=col).value
            if v is None:
                continue
            max_len = max(max_len, len(str(v)))
        ws.column_dimensions[col_letter].width = min(max_len + 2, 60)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_xlsx = os.path.join(script_dir, "lagrangian_comparison_results_lns_budget.xlsx")

    wb = Workbook()
    header_fill = PatternFill(start_color="366092", end_color="366092", fill_type="solid")
    header_font = Font(bold=True, color="FFFFFF")

    def _style_header(ws):
        for cell in ws[1]:
            cell.fill = header_fill
            cell.font = header_font

    # ---------- Sheet: LNS_EndOnly ----------                              # LNS
    ws_end = wb.active                                                      # LNS
    ws_end.title = "LNS_EndOnly"                                           # LNS
    summary_header = [                                                      # LNS
        "instance_file", "nbr_req", "nbr_cdr", "nbr_odr",                 # LNS
        "UB_LP", "best_L", "best_primal", "best_gap (%)",                  # LNS
        "iters_done", "lambda_final",                                      # LNS
        "total_time (s)", "avg_subproblem_time (s)", "status",             # LNS
    ]                                                                      # LNS
    ws_end.append(summary_header)                                          # LNS
    _style_header(ws_end)                                                  # LNS

    # ---------- Sheet: LNS_EveryIter ----------                           # LNS
    ws_iter = wb.create_sheet("LNS_EveryIter")                             # LNS
    ws_iter.append(summary_header)                                         # LNS
    _style_header(ws_iter)                                                 # LNS

    # ---------- Sheet: LNS_Comparison ----------                          # LNS
    ws_comp = wb.create_sheet("LNS_Comparison")                            # LNS
    ws_comp.append([                                                        # LNS
        "instance_file", "nbr_req", "nbr_cdr", "nbr_odr",                 # LNS
        "UB_LP",                                                           # LNS
        "EndOnly_best_L", "EveryIter_best_L",                              # LNS
        "EndOnly_best_primal", "EveryIter_best_primal",                    # LNS
        "EndOnly_gap (%)", "EveryIter_gap (%)",                            # LNS
        "EndOnly_time (s)", "EveryIter_time (s)",                          # LNS
        "EndOnly_iters", "EveryIter_iters",                                # LNS
    ])                                                                     # LNS
    _style_header(ws_comp)                                                 # LNS

    # ---------- Sheet: LNS_Runtime ----------                             # LNS
    ws_rt = wb.create_sheet("LNS_Runtime")                                 # LNS
    runtime_header = [                                                      # LNS
        "instance_file", "run_mode",                                       # LNS
        "iter", "L_u", "best_L", "best_primal", "gap_pct (%)",            # LNS
        "norm_g_sq", "lambda", "no_improve",                               # LNS
        "lns_iter_time (s)", "avg_sub_time (s)",                           # LNS
        "lns_total_y", "lns_z_ub_ones", "lns_z_ub_ones_pct (%)",          # LNS
        "lns_fixed_y", "lns_fixed_y_pct (%)",                             # LNS
        "lns_free_y", "lns_free_y_pct (%)",                               # LNS
        "lns_fixed_arcs",                                                  # LNS
    ]                                                                      # LNS
    ws_rt.append(runtime_header)                                           # LNS
    _style_header(ws_rt)                                                   # LNS

    # ---------- Sheet: LNS_FixingStats ----------                         # LNS
    ws_fix = wb.create_sheet("LNS_FixingStats")                            # LNS
    fix_header = [                                                          # LNS
        "instance_file", "nbr_req", "nbr_cdr", "nbr_odr",                 # LNS
        "run_mode", "n_lns_calls",                                         # LNS
        "avg_total_y", "avg_z_ub_ones", "avg_z_ub_ones_pct (%)",          # LNS
        "avg_fixed_y", "avg_fixed_y_pct (%)",                             # LNS
        "avg_free_y", "avg_free_y_pct (%)",                               # LNS
        "avg_fixed_arcs",                                                  # LNS
    ]                                                                      # LNS
    ws_fix.append(fix_header)                                              # LNS
    _style_header(ws_fix)                                                  # LNS

    def _write_fixing_stats(res, mode_label, nbr_req, nbr_cdr, nbr_odr, instance_file):  # LNS
        entries = [e for e in res.get("iteration_log", [])                 # LNS
                   if e.get("lns_total_y") is not None]                    # LNS
        if not entries:                                                     # LNS
            return                                                          # LNS
        n = len(entries)                                                    # LNS
        def avg(key): return sum(e.get(key) or 0 for e in entries) / n    # LNS
        ws_fix.append([                                                     # LNS
            instance_file, nbr_req, nbr_cdr, nbr_odr, mode_label, n,      # LNS
            avg("lns_total_y"), avg("lns_z_ub_ones"), avg("lns_z_ub_ones_pct"),  # LNS
            avg("lns_fixed_y"), avg("lns_fixed_y_pct"),                    # LNS
            avg("lns_free_y"), avg("lns_free_y_pct"),                      # LNS
            avg("lns_fixed_arcs"),                                         # LNS
        ])                                                                 # LNS

    # ---------- Sheets: Gurobi / GraphSearch / Comparison (solver comparison) ----------
    solver_header = [
        "instance_file",
        "nbr_req", "nbr_cdr", "nbr_odr",
        "UB_LP",
        "best_L",
        "best_primal",
        "best_gap (%)",
        "iters_done",
        "lambda_final",
        "total_time (s)",
        "avg_subproblem_time (s)",
        "status"
    ]
    ws_gurobi = wb.create_sheet("Gurobi")
    ws_gurobi.append(solver_header)
    _style_header(ws_gurobi)

    ws_graphsearch = wb.create_sheet("GraphSearch")
    ws_graphsearch.append(solver_header)
    _style_header(ws_graphsearch)

    ws_comparison = wb.create_sheet("Comparison")
    ws_comparison.append([
        "instance_file",
        "nbr_req", "nbr_cdr", "nbr_odr",
        "UB_LP",
        "Gurobi_best_L", "GraphSearch_best_L", "L_diff (%)",
        "Gurobi_time (s)", "GraphSearch_time (s)", "time_ratio (G/GS)",
        "Gurobi_avg_sub_time", "GS_avg_sub_time", "sub_time_ratio (G/GS)"
    ])
    _style_header(ws_comparison)

    # ---------- Instance list ----------
    instances = []
    nbr_req = 10
    while nbr_req <= 60:
        for nbr_cdr in range(0, 12, 2):
            nbr_odr = 10 - nbr_cdr
            instance_file = f"InstanceswithDemand/Instance_REQ{nbr_req}_CDR{nbr_cdr}_ODR{nbr_odr}_FS0_SS0.txt"
            instances.append((nbr_req, nbr_cdr, nbr_odr, instance_file))
        nbr_req += 10

    print(f"Will solve {len(instances)} instances (Gurobi, EndOnly + EveryIter LNS)")
    print(f"Writing Excel to: {out_xlsx}")
    wb.save(out_xlsx)

    total = len(instances)

    def _summary_row(res, nbr_req, nbr_cdr, nbr_odr, instance_file):      # LNS
        """Build one summary row from a solve_instance_comparison result."""  # LNS
        return [                                                            # LNS
            instance_file, nbr_req, nbr_cdr, nbr_odr,                     # LNS
            res.get("UB_LP"), res.get("best_L"),                           # LNS
            res.get("best_primal"), res.get("best_gap"),                   # LNS
            res.get("iters_done"), res.get("lambda_final"),                # LNS
            res.get("total_time"), res.get("avg_subproblem_time"),         # LNS
            res.get("status_msg"),                                         # LNS
        ]                                                                  # LNS

    def _write_runtime(res, mode_label, instance_file):                    # LNS
        """Append all iteration log rows for this run to LNS_Runtime."""   # LNS
        for entry in res.get("iteration_log", []):                         # LNS
            ws_rt.append([                                                  # LNS
                instance_file, mode_label,                                 # LNS
                entry.get("iter"),                                         # LNS
                entry.get("L_u"), entry.get("best_L"),                    # LNS
                entry.get("best_primal"), entry.get("gap_pct"),            # LNS
                entry.get("norm_g_sq"), entry.get("lambda"),               # LNS
                entry.get("no_improve"),                                   # LNS
                entry.get("lns_iter_time"), entry.get("avg_sub_time"),     # LNS
                entry.get("lns_total_y"), entry.get("lns_z_ub_ones"),      # LNS
                entry.get("lns_z_ub_ones_pct"),                            # LNS
                entry.get("lns_fixed_y"), entry.get("lns_fixed_y_pct"),   # LNS
                entry.get("lns_free_y"), entry.get("lns_free_y_pct"),     # LNS
                entry.get("lns_fixed_arcs"),                               # LNS
            ])                                                             # LNS

    for idx, (nbr_req, nbr_cdr, nbr_odr, instance_file) in enumerate(instances, start=1):
        instance_path = instance_file

        print(f"\n{'='*70}")
        print(f"Instance {idx}/{total}: REQ={nbr_req}, CDR={nbr_cdr}, ODR={nbr_odr}")
        print(f"File: {instance_path}")
        print(f"{'='*70}")

        if not os.path.exists(instance_path):
            print("[SKIP] FILE NOT FOUND")
            skip_row = [instance_file, nbr_req, nbr_cdr, nbr_odr,
                        None, None, None, None, 0, None, None, None, "FILE NOT FOUND"]
            ws_end.append(skip_row)                                        # LNS
            ws_iter.append(skip_row)                                       # LNS
            ws_comp.append([instance_file, nbr_req, nbr_cdr, nbr_odr] + [None] * 11)  # LNS
            ws_gurobi.append(skip_row)
            ws_graphsearch.append(skip_row)
            ws_comparison.append([instance_file, nbr_req, nbr_cdr, nbr_odr] + [None] * 10)
            autosize_columns(ws_end)
            autosize_columns(ws_iter)                                      # LNS
            autosize_columns(ws_comp)                                      # LNS
            autosize_columns(ws_rt)                                        # LNS
            autosize_columns(ws_fix)                                       # LNS
            autosize_columns(ws_gurobi)
            autosize_columns(ws_graphsearch)
            autosize_columns(ws_comparison)
            wb.save(out_xlsx)
            continue

        # --- Gurobi solver (uncomment to enable) ---
        # print(f"\n--- Solving with GUROBI ---")
        # res_gurobi = solve_instance_comparison(instance_path, use_graphsearch=False, lns_every_iter=False, verbose=True)
        # ws_gurobi.append([
        #     instance_file, nbr_req, nbr_cdr, nbr_odr,
        #     res_gurobi["UB_LP"], res_gurobi["best_L"],
        #     res_gurobi["best_primal"], res_gurobi["best_gap"],
        #     res_gurobi["iters_done"], res_gurobi["lambda_final"],
        #     res_gurobi["total_time"], res_gurobi["avg_subproblem_time"],
        #     res_gurobi["status_msg"]
        # ])

        # --- GraphSearch solver ---
        print(f"\n--- Solving with GRAPHSEARCH ---")
        res_graphsearch = solve_instance_comparison(instance_path, use_graphsearch=True, fix_singleton_routes=False, lns_every_iter=False, verbose=True)
        ws_graphsearch.append([
            instance_file, nbr_req, nbr_cdr, nbr_odr,
            res_graphsearch["UB_LP"], res_graphsearch["best_L"],
            res_graphsearch["best_primal"], res_graphsearch["best_gap"],
            res_graphsearch["iters_done"], res_graphsearch["lambda_final"],
            res_graphsearch["total_time"], res_graphsearch["avg_subproblem_time"],
            res_graphsearch["status_msg"]
        ])

        # --- Gurobi vs GraphSearch comparison (uncomment both solvers above first) ---
        # L_diff_pct = None
        # time_ratio = None
        # sub_time_ratio = None
        # if res_gurobi["best_L"] is not None and res_graphsearch["best_L"] is not None:
        #     if abs(res_gurobi["best_L"]) > 1e-6:
        #         L_diff_pct = 100 * (res_graphsearch["best_L"] - res_gurobi["best_L"]) / abs(res_gurobi["best_L"])
        # if res_gurobi["total_time"] is not None and res_graphsearch["total_time"] is not None:
        #     if res_graphsearch["total_time"] > 1e-6:
        #         time_ratio = res_gurobi["total_time"] / res_graphsearch["total_time"]
        # if res_gurobi["avg_subproblem_time"] is not None and res_graphsearch["avg_subproblem_time"] is not None:
        #     if res_graphsearch["avg_subproblem_time"] > 1e-6:
        #         sub_time_ratio = res_gurobi["avg_subproblem_time"] / res_graphsearch["avg_subproblem_time"]
        # ws_comparison.append([
        #     instance_file, nbr_req, nbr_cdr, nbr_odr,
        #     res_gurobi["UB_LP"],
        #     res_gurobi["best_L"], res_graphsearch["best_L"], L_diff_pct,
        #     res_gurobi["total_time"], res_graphsearch["total_time"], time_ratio,
        #     res_gurobi["avg_subproblem_time"], res_graphsearch["avg_subproblem_time"], sub_time_ratio
        # ])

        # # --- Run 1: LNS every iteration (lns_every_iter=True) ---         # LNS
        # print(f"\n--- GraphSearch + LNS every iteration ---")              # LNS
        # res_iter = solve_instance_comparison(                              # LNS
        #     instance_path, use_graphsearch=True,                         # LNS
        #     fix_singleton_routes=False,                                     # LNS
        #     lns_every_iter=True, verbose=True,                            # LNS
        # )                                                                  # LNS

        # ws_iter.append(_summary_row(res_iter, nbr_req, nbr_cdr, nbr_odr, instance_file))  # LNS
        # _write_runtime(res_iter, "EveryIter", instance_file)               # LNS
        # _write_fixing_stats(res_iter, "EveryIter", nbr_req, nbr_cdr, nbr_odr, instance_file)  # LNS

        # --- Run 2: LNS end-only (lns_every_iter=False) ---               # LNS
        print(f"\n--- GraphSearch + LNS end-only ---")                     # LNS
        res_end = solve_instance_comparison(                               # LNS
            instance_path, use_graphsearch=True,                         # LNS
            fix_singleton_routes=False,                                    # LNS
            lns_every_iter=False, lns_overlap_threshold=0.0,             # LNS
            lns_free_budget_frac=1.00, verbose=True,                      # LNS-BUD
        )                                                                  # LNS

        ws_end.append(_summary_row(res_end, nbr_req, nbr_cdr, nbr_odr, instance_file))  # LNS
        _write_runtime(res_end, "EndOnly", instance_file)                  # LNS
        _write_fixing_stats(res_end, "EndOnly", nbr_req, nbr_cdr, nbr_odr, instance_file)  # LNS

        # --- Comparison row ---                                            # LNS
        ws_comp.append([                                                    # LNS
            instance_file, nbr_req, nbr_cdr, nbr_odr,                     # LNS
            res_end.get("UB_LP"),                                          # LNS
            res_end.get("best_L"),               # LNS
            res_end.get("best_primal"),      # LNS
            res_end.get("best_gap"),             # LNS
            res_end.get("total_time"),         # LNS
            res_end.get("iters_done"),         # LNS
        ])                                                                 # LNS

\
        autosize_columns(ws_end)                                           # LNS
        autosize_columns(ws_iter)                                          # LNS
        autosize_columns(ws_comp)                                          # LNS
        autosize_columns(ws_rt)                                            # LNS
        autosize_columns(ws_fix)                                           # LNS
        autosize_columns(ws_gurobi)
        autosize_columns(ws_graphsearch)
        autosize_columns(ws_comparison)
        wb.save(out_xlsx)
        print(f"[SAVED] {out_xlsx}")

    print(f"\n{'='*70}")
    print(f"All done! Results saved to: {out_xlsx}")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
