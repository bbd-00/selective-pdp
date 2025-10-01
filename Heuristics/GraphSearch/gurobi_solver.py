
# gurobi_solver.py
# Solve the PDP with profits using the MILP in the provided formulation.
# Requires: gurobipy
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import math

try:
    import gurobipy as gp
    from gurobipy import GRB
except Exception as e:
    gp = None
    GRB = None

# Reuse the shared classes from dfs_solver
from dfs_solver import Request, NodeTW, PDPInstance

def solve_with_gurobi(inst: PDPInstance, time_limit: Optional[float] = None, verbose: bool = True):
    """
    Builds and solves the MILP to optimality (within Gurobi tolerances) for the given PDPInstance.
    Returns (routes, obj_value, runtime_sec).
      - routes: list of vehicle routes as node sequences (without the starting depot; printing can add depots)
      - obj_value: MILP objective value
      - runtime_sec: Gurobi's reported runtime in seconds (model.Runtime)
    """
    if gp is None:
        raise RuntimeError("gurobipy not available in this environment. Please `pip install gurobipy` or run where Gurobi is licensed.")

    I = inst
    K = list(range(I.n_vehicles))
    P = [r.pickup for r in I.requests]
    D = [r.delivery for r in I.requests]
    V = [I.depot_start] + P + D + [I.depot_end]

    # Map each node to a demand (pickup positive, delivery negative, depots zero)
    demand = {i: 0.0 for i in V}
    for r in I.requests:
        demand[r.pickup] = abs(r.demand)
        demand[r.delivery] = -abs(r.demand)

    # Profit only awarded at pickup node when it is "used".
    pickup_profit = {r.pickup: r.profit for r in I.requests}

    # Service times
    service = {i: (I.tw[i].service if i in I.tw else 0.0) for i in V}

    # Time windows
    l = {i: (I.tw[i].earliest if i in I.tw else 0.0) for i in V}
    u = {i: (I.tw[i].latest if i in I.tw else 1e9) for i in V}

    # Travel times/costs
    T = I.travel_time
    C = I.travel_cost

    # Big-M values
    BIGT = max(1.0, max(u[i] for i in V) - min(l[i] for i in V) + max(max(row) for row in T))
    BIGW = max(1.0, I.capacity + max(abs(demand[i]) for i in V))

    # Route duration upper bound Tbar (constraint (10))
    Tbar = (u[I.depot_end] - l[I.depot_start]) if (I.depot_start in l and I.depot_end in u) else BIGT

    m = gp.Model("PDP_Profit")

    if time_limit is not None:
        m.Params.TimeLimit = time_limit
    # Tighten MIP settings a bit for robustness
    m.Params.MIPGap = 1e-4
    if not verbose:
        m.Params.OutputFlag = 0

    # Decision variables
    # x[k,i,j] in {0,1}
    x = m.addVars(K, V, V, vtype=GRB.BINARY, name="x")

    # Time of service at node i for vehicle k
    v = m.addVars(K, V, lb=-GRB.INFINITY, name="v")

    # Load at node i for vehicle k
    w = m.addVars(K, V, lb=-GRB.INFINITY, name="w")

    # === Objective ===
    # max  sum_k ( sum_{i in P} pi_i * sum_{j in V} x_{k,i,j} )  -  sum_k sum_{i in V} sum_{j in V} c_ij * x_{k,i,j}
    profit_term = gp.quicksum(
        pickup_profit[i] * gp.quicksum(x[k, i, j] for j in V)
        for k in K for i in P
    )
    cost_term = gp.quicksum(
        C[i][j] * x[k, i, j] for k in K for i in V for j in V
    )
    m.setObjective(profit_term - cost_term, GRB.MAXIMIZE)

    # === Constraints ===

    # (3) Start and end exactly once for each vehicle (from start depot and into end depot)
    for k in K:
        m.addConstr(gp.quicksum(x[k, I.depot_start, j] for j in V) == 1, name=f"start_once[{k}]")
        m.addConstr(gp.quicksum(x[k, i, I.depot_end] for i in V) == 1, name=f"end_once[{k}]")

    # (4) Each pickup can be served by at most one vehicle (i ∈ P)
    for i in P:
        m.addConstr(gp.quicksum(x[k, i, j] for k in K for j in V) <= 1, name=f"pickup_once[{i}]")

    # (5) Same-vehicle pickup-delivery pairing (for each k and each pickup i)
    for k in K:
        for idx, r in enumerate(I.requests):
            i = r.pickup
            ip = r.delivery
            m.addConstr(gp.quicksum(x[k, i, j] for j in V) - gp.quicksum(x[k, ip, j] for j in V) == 0,
                        name=f"pair_same_vehicle[k{k},i{i}]")

    # (6) Flow conservation at every non-depot node (for each k and i in P∪D)
    for k in K:
        for i in P + D:
            m.addConstr(gp.quicksum(x[k, i, j] for j in V) - gp.quicksum(x[k, j, i] for j in V) == 0,
                        name=f"flow_balance[k{k},i{i}]")

    # (7) Time propagation with big-M: v_j >= (v_i + t_ij + d_i) x_ij
    for k in K:
        for i in V:
            for j in V:
                if i == j: 
                    continue
                m.addConstr(v[k, j] >= v[k, i] + T[i][j] + service[i] - BIGT * (1 - x[k, i, j]),
                            name=f"time_prop[k{k},i{i},j{j}]")

    # (8) Pickup precedes its delivery in time
    # NOTE: CHATGPT added this by itself whereas original formulation did not have it. Not sure which is better
    # Guard with "if served" switch: sum_j sum_k x_{k,i,j} is 1 if pickup is visited
    for k in K:
        for r in I.requests:
            i, ip = r.pickup, r.delivery
            serve_k_i = gp.quicksum(x[k, i, j] for j in V)  # 0/1 by degree
            m.addConstr(
                v[k, ip] >= v[k, i] + T[i][ip] - BIGT * (1 - serve_k_i),
                name=f"time_pick_before_deliv[k{k},i{i}]"
            )
        # m.addConstr(
        #     v[k, ip] >= v[k, i] + T[i][ip],
        #     name=f"time_pick_before_deliv[k{k},i{i}]"
        # )

    # (9) Time windows
    for k in K:
        for i in V:
            m.addConstr(v[k, i] >= l[i], name=f"tw_lb[k{k},i{i}]")
            m.addConstr(v[k, i] <= u[i], name=f"tw_ub[k{k},i{i}]")

    # (10) Overall route duration bound per vehicle
    for k in K:
        m.addConstr(v[k, I.depot_end] - v[k, I.depot_start] <= Tbar, name=f"route_duration[k{k}]")

    # (11) Load propagation with big-M: w_j >= (w_i + q_j) x_ij
    for k in K:
        for i in V:
            for j in V:
                if i == j:
                    continue
                m.addConstr(w[k, j] >= w[k, i] + demand[j] - BIGW * (1 - x[k, i, j]),
                            name=f"load_prop[k{k},i{i},j{j}]")

    # (12) Load bounds
    for k in K:
        for i in V:
            lb = max(0.0, demand[i])
            ub = min(I.capacity, I.capacity + demand[i])
            m.addConstr(w[k, i] >= lb, name=f"w_lb[k{k},i{i}]")
            m.addConstr(w[k, i] <= ub, name=f"w_ub[k{k},i{i}]")

    # Eliminate self-loops
    for k in K:
        for i in V:
            m.addConstr(x[k, i, i] == 0, name=f"no_self[k{k},i{i}]")

    # Solve
    m.optimize()

    if m.Status not in (GRB.OPTIMAL, GRB.TIME_LIMIT):
        # return what we have (if feasible)
        if m.Status == GRB.INFEASIBLE:
            m.computeIIS()
            # Print names of constraints/vars in the IIS
            for c in m.getConstrs():
                if c.IISConstr:
                    print("IIS constr:", c.ConstrName)
            for vvar in m.getVars():
                if vvar.IISLB or vvar.IISUB:
                    print("IIS var bound:", vvar.VarName, "LB?" , vvar.IISLB, "UB?", vvar.IISUB)
        if m.SolCount == 0:
            raise RuntimeError(f"Gurobi solve ended with status {m.Status} and no solution.")

    # Extract routes
    routes = [[] for _ in K]
    # Build adjacency for each vehicle
    succ = { (k,i): None for k in K for i in V }
    for k in K:
        for i in V:
            for j in V:
                if i != j and x[k, i, j].X > 0.5:
                    succ[(k,i)] = j

    for k in K:
        cur = I.depot_start
        while True:
            nxt = succ.get((k, cur), None)
            if nxt is None:
                break
            if nxt == I.depot_end:
                break
            routes[k].append(nxt)
            cur = nxt

    obj = m.ObjVal if m.SolCount > 0 else float('nan')
    runtime = float(getattr(m, "Runtime", float("nan")))
    return routes, obj, runtime
