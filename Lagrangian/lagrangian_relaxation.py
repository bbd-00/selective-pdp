# lagrangian_relaxation_batch.py
import os
import math
import time
import gurobipy as gp
from gurobipy import GRB
from openpyxl import Workbook
from openpyxl.utils import get_column_letter

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root: instance_loader
from instance_loader import load_instance_from_txt


def solve_instance(instance_path: str,
                   max_iters: int = 100,
                   sub_time_limit: int = 60,
                   lp_time_limit: int = 300,
                   lambda0: float = 2.0,
                   max_noimpv: int = 7,
                   lambda_min: float = 1e-4,
                   verbose: bool = True):
    """
    Solve ONE instance and return summary numbers for Excel.
    Returns a dict with: instance, UB_LP, best_L, iters_done, lambda_final, status_msg
    """

    t0 = time.time()

    # -----------------------
    # Load instance
    # -----------------------
    if verbose:
        print(f"\n=== Loading instance: {instance_path} ===")

    instance = load_instance_from_txt(instance_path)

    b = int(instance["b"])
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

    # Pruning unfeasible links and assignments for
    # This takes care of courier-request combos that are infeasible
    z_UB = {
        (pickup, driver): (
            1 if (li[pickup] - si[pickup] >= max(dijk[0, pickup, driver] + Ek[driver], ei[pickup])
                  and Lk[driver] >= ei[pickup + num_requests] + si[pickup + num_requests]) else 0
        )
        for driver in K for pickup in P
    }

    # This takes care of cases where j cannot be after i due to travel time restrictions or going from a delivery node back to its pickup
    x_UB = {
        (i, j): (0 if ((ei[i] + dij[i, j] + si[j] > li[j]) or (i in D and j == i - num_requests)) else 1)
        for (i, j) in A2
    }
    UBxIPC1 = {(n2, d2): 0 if (d2 in D and d2 - num_requests != n2 and x_UB[d2 - num_requests, n2] == 0) else 1 for (n2, d2) in A2}
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
    # Model builder function (uses outer-scope data for this instance)
    # -----------------------
    def build_model(relax_pickup_once=False,
                    relax_integrality=False,
                    name="SPDPTW_relaxed",
                    timeLimit=300):

        m = gp.Model(name)
        m.Params.OutputFlag = 0
        if timeLimit is not None:
            m.Params.TimeLimit = timeLimit

        x_vtype = GRB.CONTINUOUS if relax_integrality else GRB.BINARY
        y_vtype = GRB.CONTINUOUS if relax_integrality else GRB.BINARY

        x = m.addVars(A, K, vtype=x_vtype, name="x")
        Lvar = m.addVars(A2, K, vtype=GRB.CONTINUOUS, name="L")
        y = m.addVars(P + D, K, vtype=y_vtype, name="y")
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

        if not relax_pickup_once:
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

    def build_vehicle_subproblem(k, multipliers, timeLimit=60):
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

        return m, {"y": y}

    # ---------------
    # 1) LP relaxation UB
    # ---------------
    m_lp = build_model(relax_pickup_once=False, relax_integrality=True, name="LP_relax", timeLimit=lp_time_limit)
    m_lp.optimize()

    if m_lp.status not in (GRB.OPTIMAL, GRB.TIME_LIMIT):
        if verbose:
            print(f"[LP] FAILED status={m_lp.status}")
        return {
            "instance": instance_path,
            "UB_LP": None,
            "best_L": None,
            "iters_done": 0,
            "lambda_final": None,
            "status_msg": f"LP failed status={m_lp.status}"
        }

    UB_LP = float(m_lp.objVal)
    if verbose:
        print(f"[LP] UB_LP = {UB_LP:.6f}")

    # ---------------
    # 2) Lagrangian subgradient
    # ---------------
    u = {i: 0.0 for i in P}
    lambda_r = float(lambda0)
    no_improve_count = 0
    best_L = float("inf")
    it_done = 0

    if verbose:
        print("Starting subgradient method ...")
        print("Iter | L(u)           | best_L         | ||g||^2       | lambda   | noimp")

    for it in range(1, max_iters + 1):
        it_done = it
        sum_sub_obj = 0.0
        served_sum_by_i = {i: 0.0 for i in P}

        for k in K:
            mk, vk = build_vehicle_subproblem(k, u, timeLimit=sub_time_limit)
            mk.optimize()

            if mk.status not in (GRB.OPTIMAL, GRB.TIME_LIMIT, GRB.SUBOPTIMAL):
                if verbose:
                    print(f"[LAG] vehicle {k} failed status={mk.status}")
                return {
                    "instance": instance_path,
                    "UB_LP": UB_LP,
                    "best_L": None,
                    "iters_done": it_done,
                    "lambda_final": lambda_r,
                    "status_msg": f"veh {k} failed status={mk.status}"
                }

            sum_sub_obj += mk.objVal
            for i in P:
                served_sum_by_i[i] += vk["y"][i].X

        lagrangian_value = sum_sub_obj + sum(u.values())

        # Subgradient: g_i = 1 - sum_k y_i^k
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
            print(f"{it:4d} | {lagrangian_value:14.6f} | {best_L:14.6f} | {norm_g_sq:12.6f} | {lambda_r:7.4f} | {no_improve_count:5d}")

        if norm_g_sq <= 1e-12:
            if verbose:
                print("Zero (or very small) subgradient -> stopping.")
            break

        # Subgradient step
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

    elapsed = time.time() - t0
    if verbose:
        print(f"Done instance. best_L={best_L:.6f}, UB_LP={UB_LP:.6f}, iters={it_done}, lambda_final={lambda_r:.6f}, time={elapsed:.1f}s")

    return {
        "instance": instance_path,
        "UB_LP": UB_LP,
        "best_L": float(best_L),
        "iters_done": it_done,
        "lambda_final": float(lambda_r),
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
    out_xlsx = os.path.join(script_dir, "lagrangian_results.xlsx")

    wb = Workbook()
    ws = wb.active
    ws.title = "Results"

    ws.append([
        "instance_file",
        "nbr_req", "nbr_cdr", "nbr_odr",
        "UB_LP",
        "best_L (Lagrangian dual bound)",
        "iters_done",
        "lambda_final",
        "status"
    ])

    # precompute instance list so we can show progress like (3/28)
    instances = []
    nbr_req = 10
    while nbr_req <= 30:
        for nbr_cdr in range(0, 12, 2):
            nbr_odr = 10 - nbr_cdr
            instance_file = f"InstanceswithDemand/Instance_REQ{nbr_req}_CDR{nbr_cdr}_ODR{nbr_odr}_FS0_SS0.txt"
            instances.append((nbr_req, nbr_cdr, nbr_odr, instance_file))
        nbr_req += 10

    print(f"Writing Excel to: {out_xlsx}")
    wb.save(out_xlsx)  # ensures file exists immediately

    total = len(instances)
    for idx, (nbr_req, nbr_cdr, nbr_odr, instance_file) in enumerate(instances, start=1):
        instance_path = instance_file

        print(f"\n==============================")
        print(f"Instance {idx}/{total}: REQ={nbr_req}, CDR={nbr_cdr}, ODR={nbr_odr}")
        print(f"File: {instance_path}")

        if not os.path.exists(instance_path):
            print("[SKIP] FILE NOT FOUND")
            ws.append([instance_file, nbr_req, nbr_cdr, nbr_odr, None, None, 0, None, "FILE NOT FOUND"])
            autosize_columns(ws)
            wb.save(out_xlsx)
            continue

        res = solve_instance(instance_path, verbose=True)

        ws.append([
            instance_file,
            nbr_req, nbr_cdr, nbr_odr,
            res["UB_LP"],
            res["best_L"],
            res["iters_done"],
            res["lambda_final"],
            res["status_msg"]
        ])

        autosize_columns(ws)
        wb.save(out_xlsx)
        print(f"[SAVED] Row written. Current rows: {ws.max_row}")

    print(f"\nAll done. Saved results to: {out_xlsx}")


if __name__ == "__main__":
    main()