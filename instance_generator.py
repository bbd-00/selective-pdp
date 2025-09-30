# instance_generator.py
import math
import random
from typing import List, Dict, Tuple
from dfs_solver import Request, NodeTW, PDPInstance  # use the SAME classes as solver

def generate_random_instance(
    n_requests: int,
    n_vehicles: int,
    revenue_level: str = "low",     # "low" or "high"
    tw_size: str = "regular",       # "regular" or "large"
    alpha: float = None,            # default: 0.2 (low) / 0.6 (high)
    seed: int = 0,
) -> PDPInstance:
    """
    Implements the paper's instance recipe + Table 2.
    Nodes: 0 = depot. Pickups 1..n, deliveries n+1..2n.
    """
    rng = random.Random(seed)

    # ---- Table 2 ranges ----
    cost_min, cost_max = 50.0, 400.0      # euros
    time_min, time_max = 0.5, 20.0        # hours
    horizon = 128.0                        # hours

    if tw_size == "regular":
        tw_low, tw_high = 2.0, 10.0
    else:
        tw_low, tw_high = 4.0, 20.0

    service_low, service_high = 0.0, 10.0
    load_low, load_high = 1.0, 7.5
    vehicle_capacity = 7.5

    if alpha is None:
        alpha = 0.2 if revenue_level == "low" else 0.6
    if revenue_level == "low":
        delta_low, delta_high = 20.0, 100.0
    else:
        delta_low, delta_high = 60.0, 300.0

    # ---- Coordinates on [0,1]^2 ----
    n_nodes = 1 + 2 * n_requests
    xy = [(rng.random(), rng.random()) for _ in range(n_nodes)]
    depot = 0

    def euclid(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    D = [[0.0]*n_nodes for _ in range(n_nodes)]
    for i in range(n_nodes):
        for j in range(n_nodes):
            if i != j:
                D[i][j] = euclid(xy[i], xy[j])

    # ---- Distances scaled to costs in [50, 400] ----
    positive = [D[i][j] for i in range(n_nodes) for j in range(n_nodes) if i != j]
    dmin, dmax = min(positive), max(positive)
    scale = (cost_max - cost_min) / (dmax - dmin + 1e-12)

    C = [[0.0]*n_nodes for _ in range(n_nodes)]
    for i in range(n_nodes):
        for j in range(n_nodes):
            if i == j:
                C[i][j] = 0.0
            else:
                C[i][j] = cost_min + (D[i][j] - dmin) * scale

    # ---- Travel times: t_ij = 0.1*c_ij + eps, eps~U[-1,1], clipped ----
    T = [[0.0]*n_nodes for _ in range(n_nodes)]
    for i in range(n_nodes):
        for j in range(n_nodes):
            if i == j:
                T[i][j] = 0.0
            else:
                t = 0.1 * C[i][j] + rng.uniform(-1.0, 1.0)
                T[i][j] = max(time_min, min(time_max, t))

    # ---- Requests, time windows, profits ----
    requests: List[Request] = []
    tw: Dict[int, NodeTW] = {}
    tw[depot] = NodeTW(earliest=0.0, latest=horizon, service=0.0)

    for i in range(1, n_requests + 1):
        p = i
        d = n_requests + i

        sp = rng.uniform(service_low, service_high)
        sd = rng.uniform(service_low, service_high)
        q  = rng.uniform(load_low, load_high)
        W  = rng.uniform(tw_low, tw_high)

        # --- include return-to-depot in feasibility ---
        min_chain = sp + T[p][d] + sd + T[d][depot]
        latest_pickup_window_start = max(
            0.0,
            horizon - min_chain - W
        )
        earliest_pickup_window_start = T[depot][p]
        Ep = rng.uniform(earliest_pickup_window_start, latest_pickup_window_start)

        # delivery window anchored after direct p->d, but also leave time to return to depot to ensure at least one feasible request
        Ed_min = Ep + sp + T[p][d]
        Ed_max_by_return = horizon - sd - T[d][depot]
        Ed_cap = horizon - W
        Ed_ub = min(Ed_cap, Ed_max_by_return)

        slack_room = max(0.0, Ed_ub - Ed_min)
        Ed = Ed_min + rng.uniform(0.0, min(W, slack_room))

        # clamp (defensive)
        Ep = max(0.0, min(Ep, horizon - W))
        Ed = max(0.0, min(Ed, horizon - W))

        tw[p] = NodeTW(earliest=Ep, latest=Ep + W, service=sp)
        tw[d] = NodeTW(earliest=Ed, latest=Ed + W, service=sd)

        direct_cost = C[p][d]
        delta = rng.uniform(delta_low, delta_high)
        profit = alpha * direct_cost * q + delta

        requests.append(Request(i=i, pickup=p, delivery=d, demand=q, profit=profit))

    # --- Add a distinct end-depot that is a clone of the start depot (0) ---
    start_depot = depot  # should be 0
    # Compute current max node id across pickup/delivery nodes and the depot
    max_node = max(
        [start_depot]
        + [r.pickup for r in requests]
        + [r.delivery for r in requests]
    )
    end_depot = max_node + 1  # this will be your "n+1"

    # Extend travel time/cost matrices by cloning depot(0)'s row/col
    # 1) add a new column to each existing row
    for i in range(len(T)):
        T[i].append(T[i][start_depot])
        C[i].append(C[i][start_depot])
    # 2) add a new row cloned from depot(0)'s row
    T.append(T[start_depot][:] + [])  # copy row
    C.append(C[start_depot][:] + [])

    # Align the just-added row's last element (end_depot to end_depot) with depot self-loop
    T[end_depot][end_depot] = T[start_depot][start_depot]
    C[end_depot][end_depot] = C[start_depot][start_depot]

    # Copy depot time window to end-depot
    tw[end_depot] = NodeTW(earliest=tw[start_depot].earliest, latest=tw[start_depot].latest)

    # --- Return instance with two depots ---
    return PDPInstance(
        n_requests=n_requests,
        n_vehicles=n_vehicles,
        capacity=vehicle_capacity,
        depot_start=start_depot,   # 0
        depot_end=end_depot,       # n+1
        requests=requests,
        travel_time=T,
        travel_cost=C,
        tw=tw,
    )