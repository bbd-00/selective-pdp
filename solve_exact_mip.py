"""
Exact MIP for the SPDPTW with two courier types (3-index formulation, Gurobi).

solve_exact_mip(instance_path, time_limit, verbose, tighten) builds and solves the full binary
model and returns a dict with keys:
    obj, lp_relax_ub, mip_gap, status, time, time_to_best, routes, prep_time, prep_stats
Preprocessing (courier-request and arc elimination, plus time-window tightening when
tighten=True) comes from preprocessing.preprocess_instance, shared with the B&P solver; it and
the LP relaxation are not included in `time`. `time_to_best` is the Gurobi runtime at which the
final best incumbent was found; `routes` maps each courier index to its node sequence
(pickups 1..n, deliveries n+1..2n, no depots). Costs use the original courier windows.
"""

import os
import sys
import math
import time

import gurobipy as gp
from gurobipy import GRB

# Uses instance_loader.py from the repo root (the copies in Formulations/ and Cuts/ are older,
# different versions).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from instance_loader import load_instance_from_txt
from preprocessing import preprocess_instance

def solve_exact_mip(instance_path, time_limit=3600, verbose=False, tighten=False):
    """Solve the full binary MILP with Gurobi (see the module docstring for the result)."""
    # Shared preprocessing; with tighten=True the windows are tightened first (not timed in `time`)
    prep     = preprocess_instance(load_instance_from_txt(instance_path), tighten=tighten)
    instance = prep["instance"]

    b         = int(instance["b"])
    num_comm  = len(instance["committed"])
    num_occ   = len(instance["occasional"])
    num_req   = len(instance["requests"])

    P  = list(range(1, num_req + 1))
    D  = [i + num_req for i in P]
    KC = list(range(0, num_comm))
    KO = list(range(num_comm, num_comm + num_occ))
    n  = len(P)
    V  = [0] + P + D + [2 * n + 1]
    K  = KC + KO

    A1 = [(0, j) for j in P]
    A2 = [(i, j) for i in P + D for j in P + D if i != j]
    A3 = [(i, 2 * n + 1) for i in D] + [(0, 2 * n + 1)]

    coords = {}
    for req in instance["requests"]:
        pid = req["id"]
        coords[pid]           = req["pickup"]
        coords[pid + num_req] = req["dest"]

    ok = ([d["location"] for d in instance["committed"]] +
          [d["location"] for d in instance["occasional"]])

    dij_A2 = {(i, j): math.dist(coords[i], coords[j]) for (i, j) in A2}
    dij    = {**dij_A2, **{(i, j): 0 for (i, j) in A3}}
    dijk   = {(i, j, k): math.dist(ok[k], coords[j]) for (i, j) in A1 for k in K}
    tij    = dij.copy()
    tijk   = dijk.copy()

    si = {0: 0, 2 * num_req + 1: 0}
    ei, li, dem = {}, {}, {}
    for req in instance["requests"]:
        pid = req["id"]
        ei[pid], li[pid]               = req["e_pickup"],  req["l_pickup"]
        ei[pid + num_req], li[pid + num_req] = req["e_dest"],    req["l_dest"]
        si[pid]           = req["delay_p"]
        si[pid + num_req] = req["delay_d"]
        dem[pid], dem[pid + num_req] = req["l_dem"], -req["l_dem"]
    dem[0] = 0
    dem[2 * n + 1] = 0

    couriers = instance["committed"] + instance["occasional"]
    Ek = {k: d["e"] for k, d in enumerate(couriers)}      # (possibly tightened) shift window
    Lk = {k: d["l"] for k, d in enumerate(couriers)}
    Qk = {k: d["cap"] for k, d in enumerate(couriers)}
    Ek_org = {k: d["e_orig"] for k, d in enumerate(couriers)}   # original shift, for the block cost
    Lk_org = {k: d["l_orig"] for k, d in enumerate(couriers)}

    p      = {req["id"]: req["rev"] for req in instance["requests"]}   # per-request revenue
    wC     = instance["c_com_h"] / b
    cO     = instance["c_occ_o"]
    deltaC = instance["c_com_d"]
    deltaO = instance["c_occ_d"]

    # Courier-request (z_UB) and arc (x_ind) elimination, shared with the B&P solver
    z_UB  = prep["z_UB"]
    x_ind = prep["x_ind"]
    A2    = [(i, j) for (i, j) in A2 if x_ind[i, j] == 1]
    A     = A1 + A2 + A3
    PK    = [(pp, kk) for pp in P for kk in K if z_UB[pp, kk] == 1]

    m = gp.Model("exact_mip")
    m.Params.OutputFlag = 1 if verbose else 0
    m.Params.TimeLimit  = time_limit

    x    = m.addVars(A, K, vtype=GRB.BINARY,     name="x")
    Lvar = m.addVars(A2, vtype=GRB.CONTINUOUS, name="L")    # load on arc, shared by all couriers
    y    = m.addVars(P + D, K, vtype=GRB.BINARY,  name="y")
    T    = m.addVars(P + D + [0, 2*n+1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")

    obj = gp.quicksum(p[i] * gp.quicksum(y[i,k] for k in K) for i in P)
    for k in KC:
        obj -= wC*(Lk_org[k]-Ek_org[k]) + deltaC*(
            gp.quicksum(dijk[0,j,k]*x[0,j,k] for (_,j) in A1) +
            gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2)
        )
    for k in KO:
        obj -= cO*gp.quicksum(y[i,k] for i in P) + deltaO*gp.quicksum(
            dij[i,j]*x[i,j,k] for (i,j) in A2
        )
    m.setObjective(obj, GRB.MAXIMIZE)

    for i in P:
        m.addConstr(gp.quicksum(x[i,j,k] for k in K for j in V if (i,j) in A) <= 1)
    for i in P + D:
        for k in K:
            m.addConstr(
                gp.quicksum(x[j,i,k] for j in V if (j,i) in A) -
                gp.quicksum(x[i,j,k] for j in V if (i,j) in A) == 0
            )
    for i in P:
        for k in K:
            m.addConstr(
                gp.quicksum(x[i,j,k] for j in V if (i,j) in A) -
                gp.quicksum(x[j,i+num_req,k] for j in V if (j,i+num_req) in A) == 0
            )
    for k in K:
        m.addConstr(gp.quicksum(x[0,j,k] for j in V if (0,j) in A) == 1)
        m.addConstr(gp.quicksum(x[j,2*n+1,k] for j in V if (j,2*n+1) in A) == 1)
    for i in P + D:
        for k in K:
            m.addConstr(y[i,k] == gp.quicksum(x[i,l,k] for l in V if (i,l) in A))
    for i in P:
        for k in K:
            if z_UB[i,k] == 0:
                m.addConstr(y[i,k] == 0)

    svMTZ = {(i,j): min(-si[i]-dij[j,i], ei[j]-ei[i]) for (i,j) in A2}
    for (i,j) in A2:
        for k in K:
            if (j,i) in A2:
                m.addConstr(
                    T[i,k]-T[j,k]+(tij[i,j]+si[j])*x[i,j,k]+svMTZ[i,j]*x[j,i,k]
                    <= li[i]*y[i,k]-ei[j]*y[j,k]-(li[i]-ei[j])*(x[i,j,k]+x[j,i,k])
                )
            else:
                m.addConstr(
                    T[i,k]-T[j,k]+(tij[i,j]+si[j])*x[i,j,k]
                    <= li[i]*y[i,k]-ei[j]*y[j,k]-(li[i]-ei[j])*x[i,j,k]
                )
    for (i,j) in A3:
        if i != 0:
            for k in K:
                m.addConstr(
                    T[i,k]-T[j,k]+(tij[i,j]+si[j])*x[i,j,k]
                    <= li[i]*y[i,k]
                    - Ek[k]*gp.quicksum(x[0,ii,k] for ii in P if (ii,k) in PK)
                    - (li[i]-Ek[k])*x[i,j,k]
                )
    for (i,j) in A1:
        for k in K:
            m.addConstr(
                T[0,k]-T[j,k]+(tijk[0,j,k]+si[j])*x[0,j,k]
                <= Lk[k]*gp.quicksum(x[0,ii,k] for ii in P if (ii,k) in PK)
                - ei[j]*y[j,k]-(Lk[k]-ei[j])*x[0,j,k]
            )
    for i in P + D:
        for k in K:
            m.addConstr(ei[i]*y[i,k] <= T[i,k])
            m.addConstr(T[i,k] <= li[i]*y[i,k])
    for i in P:
        for k in K:
            m.addConstr(T[i,k]+(tij[i,i+num_req]+si[i+num_req])*y[i,k] <= T[i+num_req,k])
    for k in K:
        m.addConstr(Ek[k]*gp.quicksum(x[0,i,k] for i in P if (i,k) in PK) <= T[0,k])
        m.addConstr(T[2*n+1,k] <= Lk[k]*gp.quicksum(x[0,i,k] for i in P if (i,k) in PK))
    # Load flow with one load variable per arc: each arc is used by at most one courier, so the
    # sums over k pick the capacity of whichever courier uses it.
    for i in P + D:
        m.addConstr(
            gp.quicksum(Lvar[i,j] for j in P+D if (i,j) in A2) -
            gp.quicksum(Lvar[j,i] for j in P+D if (j,i) in A2)
            == dem[i]*gp.quicksum(x[i,j,k] for k in K for j in V if (i,j) in A)
        )
    for (i,j) in A2:
        m.addConstr(Lvar[i,j] >= gp.quicksum(max(dem[i],0)*x[i,j,k] for k in K))
        m.addConstr(Lvar[i,j] <= gp.quicksum((Qk[k]+min(dem[i],0))*x[i,j,k] for k in K))

    # LP relaxation (untimed — excluded from MIP timing per user request).
    # update() first: relax() copies only processed changes, so without it the LP is empty.
    m.update()
    lp = m.relax()
    lp.Params.OutputFlag = 0
    lp.optimize()
    lp_relax_ub = float(lp.ObjVal) if lp.Status == GRB.OPTIMAL else None

    # Record when each improving incumbent is found (Gurobi runtime, measured from the
    # start of the MIP solve like `elapsed`); the last improvement is the time to best.
    best_found = {"obj": -math.inf, "time": None}

    def _track_incumbent(model, where):
        if where == GRB.Callback.MIPSOL:
            obj = model.cbGet(GRB.Callback.MIPSOL_OBJ)
            if obj > best_found["obj"] + 1e-9:
                best_found["obj"] = obj
                best_found["time"] = model.cbGet(GRB.Callback.RUNTIME)

    # MIP solve — timing starts here
    t0 = time.time()
    m.optimize(_track_incumbent)
    elapsed = time.time() - t0

    if m.SolCount == 0:
        return {"obj": None, "lp_relax_ub": lp_relax_ub, "mip_gap": None, "status": m.Status,
                "time": elapsed, "time_to_best": None, "routes": None,
                "prep_time": prep["stats"]["prep_time"], "prep_stats": prep["stats"]}

    # Each courier's route: follow its used arcs from the source 0 to the sink 2n+1
    # (node sequence without depots, same format as the B&P solver's routes).
    successor = {(i, k): j for (i, j) in A for k in K if x[i, j, k].X > 0.5}
    routes = {}
    for k in K:
        route, node = [], successor.get((0, k))
        while node is not None and node != 2 * n + 1 and len(route) <= 2 * n:
            route.append(node)
            node = successor.get((node, k))
        routes[k] = route

    return {
        "obj":          float(m.objVal),
        "lp_relax_ub":  lp_relax_ub,
        "mip_gap":      float(m.MIPGap) * 100.0,
        "status":       m.Status,
        "time":         elapsed,
        "time_to_best": best_found["time"],
        "routes":       routes,
        "prep_time":    prep["stats"]["prep_time"],
        "prep_stats":   prep["stats"],
    }
