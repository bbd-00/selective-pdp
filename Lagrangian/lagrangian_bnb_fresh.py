# lagrangian_bnb_fresh.py  [u=0 variant]
#
# Branch-and-Bound with Lagrangian relaxation as the bounding function.
# At each B&B node, the Lagrangian subgradient (GraphSearch subproblems) is
# solved with z_UB modified to reflect the node's fixing decisions.
#
# Children restart multipliers from u=0 (fresh solve at every node).
#
# Branching rule: for the most-contested duplicate request i, pick the two
# couriers k1, k2 that are both claiming i.
#   Left  child: only k1 may serve i (all others excluded); restart u=0
#   Right child: only k2 may serve i (all others excluded); restart u=0
# Together these two children are exhaustive: any feasible solution must assign
# i to exactly one of {k1, k2} or to neither (which is captured by both children
# allowing the request to be dropped).
#
# Primal check: when the Lagrangian solution has no duplicates, re-run
# GraphSearch with u=0 and assignment-restricted z_UB to get the true primal value.

import os
import math
import time
import heapq
import gurobipy as gp
from gurobipy import GRB
from openpyxl import Workbook
from openpyxl.utils import get_column_letter
from openpyxl.styles import Font, PatternFill

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root: instance_loader
from instance_loader import load_instance_from_txt
from graph_search import GraphSearchSolver


def solve_instance_bnb(instance_path: str,
                       max_iters_root: int = 100,
                       max_iters_node: int = 30,
                       sub_time_limit: int = 60,
                       lp_time_limit: int = 300,
                       bnb_time_limit: int = 3600,
                       lambda0: float = 2.0,
                       max_noimpv: int = 7,
                       lambda_min: float = 1e-4,
                       verbose: bool = True):
    """
    Solve ONE instance using Branch-and-Bound with Lagrangian relaxation bounding.

    Args:
        max_iters_root:  Subgradient iterations at the root node.
        max_iters_node:  Subgradient iterations at all other nodes.
        bnb_time_limit:  Wall-clock limit (s) for the entire B&B.

    Return dict keys (compatible with solve_instance_comparison):
        instance, solver, UB_LP, best_L (root Lagrangian bound), best_primal,
        best_gap (% vs UB_LP), lag_gap (% vs root best_L), iters_done,
        total_time, n_nodes, n_pruned, n_feasible, max_depth, status_msg,
        node_log (list of per-node dicts).
    """
    t0 = time.time()

    if verbose:
        print(f"\n=== Loading instance: {instance_path} ===")

    instance = load_instance_from_txt(instance_path)

    b = int(instance["b"])
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
        coords[pid]              = req["pickup"]
        coords[pid + num_requests] = req["dest"]

    ok = ([d["location"] for d in instance["committed"]] +
          [d["location"] for d in instance["occasional"]])

    dij_A2 = {(i, j): math.dist(coords[i], coords[j]) for (i, j) in A2}
    dij_A3 = {(i, j): 0 for (i, j) in A3}
    dij    = {**dij_A2, **dij_A3}
    dijk   = {(i, j, k): math.dist(ok[k], coords[j]) for (i, j) in A1 for k in K}
    tij    = dij.copy()
    tijk   = dijk.copy()

    si = {0: 0, 2 * num_requests + 1: 0}
    ei, li, dem = {}, {}, {}
    for req in instance["requests"]:
        pid = req["id"]
        ei[pid], li[pid]                           = req["e_pickup"],  req["l_pickup"]
        ei[pid + num_requests], li[pid + num_requests] = req["e_dest"],    req["l_dest"]
        si[pid]               = req["delay_p"]
        si[pid + num_requests] = req["delay_d"]
        dem[pid], dem[pid + num_requests] = req["l_dem"], -req["l_dem"]
    dem[0] = 0
    dem[2 * n + 1] = 0

    Ek = ({d["id"]: d["e"] for d in instance["committed"]} |
          {d["id"]: d["e"] for d in instance["occasional"]})
    Lk = ({d["id"]: d["l"] for d in instance["committed"]} |
          {d["id"]: d["l"] for d in instance["occasional"]})
    Qk = ({d["id"]: d["cap"] for d in instance["committed"]} |
          {d["id"]: d["cap"] for d in instance["occasional"]})

    p      = {req["id"]: req["rev"] for req in instance["requests"]}   # per-request revenue
    wC     = instance["c_com_h"] / b
    cO     = instance["c_occ_o"]
    deltaC = instance["c_com_d"]
    deltaO = instance["c_occ_d"]

    z_UB_base = {
        (pickup, driver): (
            1 if (li[pickup] - si[pickup] >= max(dijk[0, pickup, driver] + Ek[driver], ei[pickup] - si[pickup])
                  and Lk[driver] >= ei[pickup + num_requests])
            else 0
        )
        for driver in K for pickup in P
    }

    x_UB = {
        (i, j): (0 if (ei[i] + dij[i, j] + si[j] > li[j]) or (i in D and j == i - num_requests) else 1)
        for (i, j) in A2
    }
    UBxIPC1 = {
        (n2, d2): 0 if (d2 in D and d2 - num_requests != n2 and x_UB.get((d2 - num_requests, n2), 0) == 0) else 1
        for (n2, d2) in A2
    }
    UBxIPC2 = {
        (p2, n2): 0 if (p2 in P and p2 + num_requests != n2 and x_UB.get((n2, p2 + num_requests), 0) == 0) else 1
        for (p2, n2) in A2
    }
    x_ind = {(i, j): x_UB[i, j] * UBxIPC1[i, j] * UBxIPC2[i, j] for (i, j) in A2}

    A2 = [(i, j) for (i, j) in A2 if x_ind[i, j] == 1]
    A  = A1 + A2 + A3
    PK = [(pp, kk) for pp in P for kk in K if z_UB_base[pp, kk] == 1]
    Lk_org = Lk.copy()

    if verbose:
        print(f"Requests={num_requests}, CDR={num_comm_drivers}, ODR={num_occ_drivers}, Vehicles={len(K)}")
        print(f"Arcs: |A1|={len(A1)}, |A2| pruned={len(A2)}, |A3|={len(A3)}")
        print("Solving LP relaxation ...")

    # ------------------------------------------------------------------
    # LP Relaxation (Gurobi, continuous)
    # ------------------------------------------------------------------
    def build_lp_model(timeLimit=300):
        m = gp.Model("LP_relax")
        m.Params.OutputFlag = 0
        if timeLimit:
            m.Params.TimeLimit = timeLimit

        x    = m.addVars(A, K, vtype=GRB.CONTINUOUS, name="x")
        Lvar = m.addVars(A2, vtype=GRB.CONTINUOUS, name="L")
        y    = m.addVars(P + D, K, vtype=GRB.CONTINUOUS, name="y")
        T    = m.addVars(P + D + [0, 2 * n + 1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")

        obj_expr = gp.quicksum(p[i] * gp.quicksum(y[i, k] for k in K) for i in P)
        for k in KC:
            obj_expr -= wC * (Lk_org[k] - Ek[k]) + deltaC * (
                gp.quicksum(dijk[0, j, k] * x[0, j, k] for (_, j) in A1) +
                gp.quicksum(dij[i, j]     * x[i, j, k] for (i, j) in A2)
            )
        for k in KO:
            obj_expr -= cO  * gp.quicksum(y[i, k] for i in P) + deltaO * gp.quicksum(
                dij[i, j] * x[i, j, k] for (i, j) in A2
            )
        m.setObjective(obj_expr, GRB.MAXIMIZE)

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
                if z_UB_base[i, k] == 0:
                    m.addConstr(y[i, k] == 0)

        svMTZ = {(i, j): min(-si[i] - dij[j, i], ei[j] - ei[i]) for (i, j) in A2}
        for (i, j) in A2:
            for k in K:
                if (j, i) in A2:
                    m.addConstr(
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k] + svMTZ[i, j] * x[j, i, k]
                        <= li[i]*y[i,k] - ei[j]*y[j,k] - (li[i]-ei[j])*(x[i,j,k]+x[j,i,k])
                    )
                else:
                    m.addConstr(
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k]
                        <= li[i]*y[i,k] - ei[j]*y[j,k] - (li[i]-ei[j])*x[i,j,k]
                    )
        for (i, j) in A3:
            if i != 0:
                for k in K:
                    m.addConstr(
                        T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k]
                        <= li[i]*y[i,k]
                        - Ek[k]*gp.quicksum(x[0, ii, k] for ii in P if (ii, k) in PK)
                        - (li[i]-Ek[k])*x[i,j,k]
                    )
        for (i, j) in A1:
            for k in K:
                m.addConstr(
                    T[0, k] - T[j, k] + (tijk[0, j, k] + si[j]) * x[0, j, k]
                    <= Lk[k]*gp.quicksum(x[0, ii, k] for ii in P if (ii, k) in PK)
                    - ei[j]*y[j,k] - (Lk[k]-ei[j])*x[0,j,k]
                )
        for i in P + D:
            for k in K:
                m.addConstr(ei[i] * y[i, k] <= T[i, k])
                m.addConstr(T[i, k] <= li[i] * y[i, k])
        for i in P:
            for k in K:
                m.addConstr(T[i, k] + (tij[i, i + num_requests] + si[i + num_requests]) * y[i, k]
                            <= T[i + num_requests, k])
        for k in K:
            m.addConstr(Ek[k]*gp.quicksum(x[0, i, k] for i in P if (i,k) in PK) <= T[0, k])
            m.addConstr(T[2*n+1, k] <= Lk[k]*gp.quicksum(x[0, i, k] for i in P if (i,k) in PK))
        # Load flow with one load variable per arc, shared by all couriers (each arc is used by at
        # most one courier, so the sums over k pick the capacity of whichever courier uses it).
        for i in P + D:
            m.addConstr(
                gp.quicksum(Lvar[i, j] for j in P + D if (i, j) in A2) -
                gp.quicksum(Lvar[j, i] for j in P + D if (j, i) in A2)
                == dem[i] * gp.quicksum(x[i, j, k] for k in K for j in V if (i, j) in A)
            )
        for (i, j) in A2:
            m.addConstr(Lvar[i, j] >= gp.quicksum(max(dem[i], 0) * x[i, j, k] for k in K))
            m.addConstr(Lvar[i, j] <= gp.quicksum((Qk[k] + min(dem[i], 0)) * x[i, j, k] for k in K))
        return m

    m_lp = build_lp_model(lp_time_limit)
    m_lp.optimize()

    if m_lp.status not in (GRB.OPTIMAL, GRB.TIME_LIMIT):
        if verbose:
            print(f"[LP] FAILED status={m_lp.status}")
        return {
            "instance": instance_path, "solver": "BnB_GraphSearch",
            "UB_LP": None, "best_L": None, "best_primal": None,
            "best_gap": None, "lag_gap": None, "iters_done": 0,
            "total_time": time.time() - t0, "n_nodes": 0, "n_pruned": 0,
            "n_feasible": 0, "max_depth": 0, "node_log": [],
            "status_msg": f"LP failed status={m_lp.status}"
        }

    UB_LP = float(m_lp.objVal)
    if verbose:
        print(f"[LP] UB_LP = {UB_LP:.6f}")

    # ------------------------------------------------------------------
    # Lagrangian subgradient solver (called at every B&B node)
    # ------------------------------------------------------------------
    def _solve_lagrangian(z_UB_node, u_init, max_iters_here):
        u            = dict(u_init)
        lambda_r     = float(lambda0)
        no_improve   = 0
        best_L       = float("inf")
        it_done      = 0
        served_by    = {i: {k: 0.0 for k in K} for i in P}
        sub_times    = []

        for it in range(1, max_iters_here + 1):
            it_done = it
            sum_sub_obj      = 0.0
            served_by        = {i: {k: 0.0 for k in K} for i in P}
            served_sum_by_i  = {i: 0.0 for i in P}
            iter_times       = []

            for k in K:
                ts  = time.time()
                gs  = GraphSearchSolver(instance, k, z_UB_node, x_UB, x_ind, u, verbose=False)
                res = gs.solve()
                for pickup in res["served_pickups"]:
                    served_sum_by_i[pickup]  += 1.0
                    served_by[pickup][k]      = 1.0
                iter_times.append(time.time() - ts)
                sum_sub_obj += res["objVal"]

            sub_times.extend(iter_times)
            lag_val = sum_sub_obj + sum(u.values())

            g        = {i: 1.0 - served_sum_by_i[i] for i in P}
            norm_g_sq = sum(g[i] ** 2 for i in P)

            improved = lag_val < best_L - 1e-9
            if improved:
                best_L     = lag_val
                no_improve = 0
            else:
                no_improve += 1

            if norm_g_sq <= 1e-12:
                break

            step = lambda_r * (lag_val - (-abs(UB_LP))) / norm_g_sq
            for i in P:
                u[i] = max(0.0, u[i] - step * g[i])

            if no_improve >= max_noimpv:
                lambda_r   *= 0.5
                no_improve  = 0
                if lambda_r < lambda_min:
                    break

        avg_sub = sum(sub_times) / len(sub_times) if sub_times else 0.0
        return best_L, u, served_by, it_done, avg_sub

    # ------------------------------------------------------------------
    # Compute true primal value at a duplicate-free node.
    # Re-run GraphSearch with u=0 restricted to each courier's assigned
    # requests to get the true route cost without multiplier distortion,
    # while guaranteeing no duplicate assignments.
    # ------------------------------------------------------------------
    def _compute_primal(served_by, z_UB_node):
        u_zero = {i: 0.0 for i in P}
        total  = 0.0
        for k in K:
            assigned = {i for i in P if served_by[i][k] > 0.5}
            z_leaf   = {
                (i2, k2): (z_UB_node.get((i2, k2), 0) if k2 == k and i2 in assigned else 0)
                for i2 in P for k2 in K
            }
            gs  = GraphSearchSolver(instance, k, z_leaf, x_UB, x_ind, u_zero, verbose=False)
            res = gs.solve()
            total += res["objVal"]
        return total

    # ------------------------------------------------------------------
    # Branch-and-Bound
    # Node in heap: (neg_bound, node_id, depth, excluded, u_init)
    # excluded  : {request_i: set_of_k}   — i must NOT be served by any k in set
    #
    # Branching on duplicate request i_branch claimed by couriers k1 and k2:
    #   Left  child: only k1 may serve i_branch (all others excluded)
    #   Right child: only k2 may serve i_branch (all others excluded)
    # ------------------------------------------------------------------
    best_primal   = None
    best_assign   = None
    n_nodes       = 0
    n_pruned      = 0
    n_feasible    = 0
    max_depth     = 0
    node_log      = []
    _id_counter   = [0]
    best_L_root   = None
    iters_root    = 0
    avg_sub_root  = None

    def _nid():
        _id_counter[0] += 1
        return _id_counter[0]

    u_zero = {i: 0.0 for i in P}
    # heap entry: (neg_bound, node_id, depth, excluded, u_init)
    heap = [(-UB_LP, _nid(), 0, {}, u_zero)]

    if verbose:
        print(f"\nStarting Branch-and-Bound (time limit={bnb_time_limit}s)...")
        print(f"{'Node':>6} | {'Depth':>5} | {'Bound':>12} | {'BestPrimal':>12} | {'Dups':>4} | Action")
        print("-" * 75)

    while heap and (time.time() - t0) < bnb_time_limit:
        neg_bnd, node_id, depth, excluded, u_init = heapq.heappop(heap)
        parent_bound = -neg_bnd
        max_depth    = max(max_depth, depth)
        n_nodes     += 1

        # Prune by inherited bound
        if best_primal is not None and parent_bound <= best_primal + 1e-6:
            n_pruned += 1
            continue

        # Build node-specific z_UB (exclusions only)
        z_UB_node = dict(z_UB_base)
        for i, excl_set in excluded.items():
            for k in excl_set:
                z_UB_node[i, k] = 0

        # Solve Lagrangian — same iteration budget at every node (fresh u=0 start)
        max_iters_here = max_iters_root if depth == 0 else max_iters_node
        bound, u_final, served_by, it_done, avg_sub = _solve_lagrangian(
            z_UB_node, u_init, max_iters_here
        )

        if depth == 0:
            best_L_root  = bound
            iters_root   = it_done
            avg_sub_root = avg_sub
            if verbose:
                print(f"  [Root] best_L = {best_L_root:.6f}  iters={iters_root}")

        # Prune by tightened bound
        if best_primal is not None and bound <= best_primal + 1e-6:
            n_pruned += 1
            node_log.append({"node_id": node_id, "depth": depth, "bound": bound,
                              "best_primal": best_primal, "n_dups": None,
                              "action": "PRUNED", "branch_req": None, "branch_k": None})
            continue

        # Classify duplicates
        duplicates = [i for i in P if sum(1 for k in K if served_by[i][k] > 0.5) > 1]
        n_dups     = len(duplicates)

        if n_dups == 0:
            # Primal feasible — compute true objective
            primal_val  = _compute_primal(served_by, z_UB_node)
            n_feasible += 1
            if best_primal is None or primal_val > best_primal:
                best_primal = primal_val
                best_assign = {i: next((k for k in K if served_by[i][k] > 0.5), None) for i in P}
            node_log.append({"node_id": node_id, "depth": depth, "bound": bound,
                              "best_primal": best_primal, "n_dups": 0,
                              "action": "FEASIBLE", "branch_req": None, "branch_k": None})
            if verbose:
                print(f"{node_id:>6} | {depth:>5} | {bound:>12.4f} | {best_primal:>12.4f} | {0:>4} | FEASIBLE  primal={primal_val:.4f}")
            continue

        # Select branching request: most claimants; break ties by fewest remaining feasible couriers
        i_branch = max(
            duplicates,
            key=lambda i: (
                sum(1 for k in K if served_by[i][k] > 0.5),
                -sum(1 for k in K if z_UB_node.get((i, k), 0) > 0)
            )
        )
        claimants = [k for k in K if served_by[i_branch][k] > 0.5]

        # Pick two claimants to branch on: prefer CDR couriers first (narrower TW)
        cdr_first = sorted(claimants, key=lambda k: (0 if k in KC else 1))
        k1 = cdr_first[0]
        k2 = cdr_first[1] if len(cdr_first) > 1 else cdr_first[0]

        node_log.append({"node_id": node_id, "depth": depth, "bound": bound,
                          "best_primal": best_primal, "n_dups": n_dups,
                          "action": "BRANCH",
                          "branch_req": i_branch, "branch_k": k1})

        if verbose:
            bp_str = f"{best_primal:>12.4f}" if best_primal is not None else f"{'None':>12}"
            print(f"{node_id:>6} | {depth:>5} | {bound:>12.4f} | {bp_str} | {n_dups:>4} | BRANCH  req={i_branch} k1={k1} k2={k2}")

        # Left child: only k1 may serve i_branch (all others excluded)
        new_excl_left = {i: set(s) for i, s in excluded.items()}
        new_excl_left[i_branch] = {k for k in K if k != k1}
        heapq.heappush(heap, (
            -bound, _nid(), depth + 1,
            new_excl_left,
            {i: 0.0 for i in P}   # fresh start
        ))

        # Right child: only k2 may serve i_branch (all others excluded)
        new_excl_right = {i: set(s) for i, s in excluded.items()}
        new_excl_right[i_branch] = {k for k in K if k != k2}
        heapq.heappush(heap, (
            -bound, _nid(), depth + 1,
            new_excl_right,
            {i: 0.0 for i in P}   # fresh start
        ))

    elapsed    = time.time() - t0
    timed_out  = elapsed >= bnb_time_limit

    best_gap = (100.0 * (UB_LP - best_primal) / abs(best_primal)
                if best_primal is not None and abs(best_primal) > 1e-9 else None)
    lag_gap  = (100.0 * (best_L_root - best_primal) / abs(best_primal)
                if best_L_root is not None and best_primal is not None and abs(best_primal) > 1e-9
                else None)

    if verbose:
        print(f"\nB&B finished. nodes={n_nodes}  pruned={n_pruned}  feasible={n_feasible}  max_depth={max_depth}")
        if best_primal is not None:
            print(f"best_primal={best_primal:.6f}  best_L(root)={best_L_root:.6f}  "
                  f"lag_gap={lag_gap:.2f}%  UB_LP_gap={best_gap:.2f}%")
        else:
            print("No feasible solution found within time/node limits.")
        print(f"Total time={elapsed:.1f}s  {'(TIME LIMIT)' if timed_out else ''}")

    return {
        "instance":            instance_path,
        "solver":              "BnB_GraphSearch",
        "UB_LP":               UB_LP,
        "best_L":              best_L_root,
        "best_primal":         best_primal,
        "best_gap":            best_gap,
        "lag_gap":             lag_gap,
        "iters_done":          iters_root,
        "lambda_final":        None,
        "total_time":          elapsed,
        "avg_subproblem_time": avg_sub_root,
        "n_nodes":             n_nodes,
        "n_pruned":            n_pruned,
        "n_feasible":          n_feasible,
        "max_depth":           max_depth,
        "node_log":            node_log,
        "best_assign":         best_assign,
        "status_msg":          "TIME_LIMIT" if timed_out else "OK",
    }


# ------------------------------------------------------------------
# Excel helpers
# ------------------------------------------------------------------
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
    out_xlsx   = os.path.join(script_dir, "lagrangian_bnb_fresh_results.xlsx")

    wb          = Workbook()
    header_fill = PatternFill(start_color="366092", end_color="366092", fill_type="solid")
    header_font = Font(bold=True, color="FFFFFF")

    def _style_header(ws):
        for cell in ws[1]:
            cell.fill = header_fill
            cell.font = header_font

    # ---------- Sheet: BnB_Summary ----------
    ws_sum = wb.active
    ws_sum.title = "BnB_Summary"
    summary_header = [
        "instance_file", "nbr_req", "nbr_cdr", "nbr_odr",
        "UB_LP", "best_L (root)", "best_primal",
        "best_gap_vs_LP (%)", "lag_gap (%)",
        "iters_root", "total_time (s)", "avg_subproblem_time (s)",
        "n_nodes", "n_pruned", "n_feasible", "max_depth", "status",
    ]
    ws_sum.append(summary_header)
    _style_header(ws_sum)

    # ---------- Sheet: BnB_NodeLog ----------
    ws_nodes = wb.create_sheet("BnB_NodeLog")
    ws_nodes.append([
        "instance_file", "node_id", "depth", "bound",
        "best_primal", "n_dups", "action", "branch_req", "branch_k",
    ])
    _style_header(ws_nodes)

    # ---------- Instance list (same as other files) ----------
    instances = []
    nbr_req = 10
    while nbr_req <= 60:
        for nbr_cdr in range(0, 12, 2):
            nbr_odr       = 10 - nbr_cdr
            instance_file = (f"InstanceswithDemand/"
                             f"Instance_REQ{nbr_req}_CDR{nbr_cdr}_ODR{nbr_odr}_FS0_SS0.txt")
            instances.append((nbr_req, nbr_cdr, nbr_odr, instance_file))
        nbr_req += 10

    print(f"Will solve {len(instances)} instances with Lagrangian B&B")
    print(f"Writing Excel to: {out_xlsx}")
    wb.save(out_xlsx)

    total = len(instances)

    for idx, (nbr_req, nbr_cdr, nbr_odr, instance_file) in enumerate(instances, start=1):
        print(f"\n{'='*70}")
        print(f"Instance {idx}/{total}: REQ={nbr_req}, CDR={nbr_cdr}, ODR={nbr_odr}")
        print(f"File: {instance_file}")
        print(f"{'='*70}")

        if not os.path.exists(instance_file):
            print("[SKIP] FILE NOT FOUND")
            skip_row = [instance_file, nbr_req, nbr_cdr, nbr_odr] + [None] * 13
            ws_sum.append(skip_row)
            autosize_columns(ws_sum)
            autosize_columns(ws_nodes)
            wb.save(out_xlsx)
            continue

        res = solve_instance_bnb(instance_file, verbose=True)

        ws_sum.append([
            instance_file, nbr_req, nbr_cdr, nbr_odr,
            res.get("UB_LP"),
            res.get("best_L"),
            res.get("best_primal"),
            res.get("best_gap"),
            res.get("lag_gap"),
            res.get("iters_done"),
            res.get("total_time"),
            res.get("avg_subproblem_time"),
            res.get("n_nodes"),
            res.get("n_pruned"),
            res.get("n_feasible"),
            res.get("max_depth"),
            res.get("status_msg"),
        ])

        for entry in res.get("node_log", []):
            ws_nodes.append([
                instance_file,
                entry.get("node_id"),
                entry.get("depth"),
                entry.get("bound"),
                entry.get("best_primal"),
                entry.get("n_dups"),
                entry.get("action"),
                entry.get("branch_req"),
                entry.get("branch_k"),
            ])

        autosize_columns(ws_sum)
        autosize_columns(ws_nodes)
        wb.save(out_xlsx)
        print(f"[SAVED] {out_xlsx}")


if __name__ == "__main__":
    main()
