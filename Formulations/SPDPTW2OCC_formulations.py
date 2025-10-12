#!/usr/bin/env python
# coding: utf-8

import math
import gurobipy as gp
from gurobipy import GRB
from instance_loader import *

# ------------------------------------------------------------
# LOAD INSTANCE FROM FILE
# ------------------------------------------------------------
instance = load_instance_from_txt("Instances/Instance_REQ15_CDR4_ODR2_FS0_SS0.txt")

H = int(instance["H"])
b = int(instance["b"])
num_comm_drivers = len(instance["committed"])
num_occ_drivers = len(instance["occasional"])
num_requests = len(instance["requests"])

# Node indexing
P = list(range(1, num_requests+1))
D = [i+num_requests for i in P]
KC = list(range(1, num_comm_drivers+1))
KO = list(range(num_comm_drivers+1, num_comm_drivers+num_occ_drivers+1))
n = len(P)
V = [0] + P + D + [2*n+1]
K = KC + KO

# Arcs
A1 = [(0,j) for j in P]
A2 = [(i,j) for i in P+D for j in P+D if i != j]
A3 = [(i,2*n+1) for i in D] + [(0,2*n+1)]
A = A1 + A2 + A3

# Build coordinates: requests only
coords = {}
for req in instance["requests"]:
    pid = req["id"]
    coords[pid] = req["pickup"]
    coords[pid+num_requests] = req["dest"]

# Build driver initial locations (in order committed+occasional)
ok = [d["location"] for d in instance["committed"]] + \
     [d["location"] for d in instance["occasional"]]

# Distances
dij_A2 = {(i,j): math.dist(coords[i], coords[j]) for (i,j) in A2}
dij_A3 = {(i,j): 0 for (i,j) in A3}
dij = {**dij_A2, **dij_A3}
dijk = {(i,j,k): math.dist(ok[k-1], coords[j]) for (i,j) in A1 for k in K}
tij = dij.copy()
tijk = dijk.copy()

# Build time windows and service times
si = {0: 0, 2*num_requests+1: 0}
ei, li = {}, {}
for req in instance["requests"]:
    pid = req["id"]
    ei[pid], li[pid] = req["e_pickup"], req["l_pickup"]
    ei[pid+num_requests], li[pid+num_requests] = req["e_dest"], req["l_dest"]
    si[pid] = req["delay_p"]    # pickup service time
    si[pid+num_requests] = req["delay_d"]  # delivery service time

Ek = {d["id"]: d["e"] for d in instance["committed"]} | {d["id"]: d["e"] for d in instance["occasional"]}
Lk = {d["id"]: d["l"] for d in instance["committed"]} | {d["id"]: d["l"] for d in instance["occasional"]}

# Costs and revenue
p = {i: instance["rev_order"] for i in P}

wC = instance["c_com_h"] / b   # committed cost/hour -> cost per minute
cO = instance["c_occ_o"]          # occasional cost per order
deltaC = instance["c_com_d"]      # committed distance cost (same as cC if you want)
deltaO = instance["c_occ_d"]      # occasional distance cost


# ------------------------------------------------------------
# BUILD MODEL (eqns 1–15)
# ------------------------------------------------------------
m = gp.Model("SPDPTW2OCC")

x = m.addVars(A, K, vtype=GRB.BINARY, name="x")
y = m.addVars(P, K, vtype=GRB.BINARY, name="y")
T = m.addVars(P+D+[0,2*n+1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")

# Objective (1)
obj = gp.quicksum(p[i] * gp.quicksum(y[i,k] for k in K) for i in P)

obj -= gp.quicksum(
    (wC*(Lk[k]-Ek[k]) + deltaC*(
        gp.quicksum(dijk[0,j,k]*x[0,j,k] for (i,j) in A1) +
        gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2)
    )) for k in KC
)

obj -= gp.quicksum(
    (cO * gp.quicksum(y[i,k] for i in P) +
     deltaO * gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2 if i in P and j in D))
    for k in KO
)

m.setObjective(obj, GRB.MAXIMIZE)

# (2) Each request served at most once
m.addConstrs(gp.quicksum(x[i,j,k] for k in K for j in V if (i,j) in A) <= 1 for i in P)

# (3) Flow conservation
m.addConstrs(
    gp.quicksum(x[j,i,k] for j in V if (j,i) in A) -
    gp.quicksum(x[i,j,k] for j in V if (i,j) in A) == 0
    for i in P+D for k in K
)

# (4) Pickup-delivery flow
m.addConstrs(
    gp.quicksum(x[i,j,k] for j in V if (i,j) in A) -
    gp.quicksum(x[j,i+n,k] for j in V if (j,i+n) in A) == 0
    for i in P for k in K
)

# (5) Depart depot
m.addConstrs(gp.quicksum(x[0,j,k] for j in V if (0, j) in A) == 1 for k in K)

# (6) Return depot
m.addConstrs(gp.quicksum(x[j,2*n+1,k] for j in V if (j,2*n+1) in A) == 1 for k in K)

# (7a) Time propagation 1
m.addConstrs(
    T[j,k] >= T[i,k] + tij[i,j] + si[j] - H*(1-x[i,j,k])
    for (i,j) in A2+A3 for k in K
)

# (7b) Time propagation 2
m.addConstrs(
    T[j,k] >= T[0,k] + tijk[i,j,k] + si[j] - H*(1-x[i,j,k])
    for (i,j) in A1 for k in K
)

# (8a) Time windows 1
m.addConstrs((ei[i] <= T[i,k]) for i in P+D for k in K)

# (8b) Time windows 2
m.addConstrs((T[i,k] <= li[i]) for i in P+D for k in K)

# (9) Pickup before delivery
m.addConstrs(T[i,k] + tij[i,i+n] + si[i+n] <= T[i+n,k] for i in P for k in K)

# (10) y = arc usage
m.addConstrs(y[i,k] == gp.quicksum(x[i,j,k] for j in V if (i,j) in A) for i in P for k in K)

# (11) Earliest departure
m.addConstrs(Ek[k-1] <= T[0,k] for k in K)

# (12) Latest return
m.addConstrs(T[2*n+1,k] <= Lk[k-1] for k in K)

# ------------------------------------------------------------
# SOLVE
# ------------------------------------------------------------
m.optimize()

if m.status == GRB.OPTIMAL:
    print("\n=== Optimal Objective Value ===")
    print(round(m.objVal, 4))
    print("\n=== Courier Schedules ===")

    for k in K:
        start_time = T[0,k].X
        end_time   = T[2*n+1,k].X

        print(f"\nCourier {k}:")
        print(f"  Start time: {start_time:.2f}  (Earliest allowed {Ek[k-1]:.2f})")
        print(f"  End time:   {end_time:.2f}  (Latest allowed {Lk[k-1]:.2f})")

        # list served pickups
        served = [i for i in P if y[i,k].X > 0.5]
        if not served:
            print("  No requests assigned")
            continue

        print("  Served requests:")
        for i in served:
            pickup_time   = T[i,k].X
            delivery_time = T[i+n,k].X
            print(f"    Request {i}: "
                  f"Pickup at {pickup_time:.2f} (TW [{ei[i]:.2f},{li[i]:.2f}]), "
                  f"Delivery at {delivery_time:.2f} (TW [{ei[i+n]:.2f},{li[i+n]:.2f}])")

        # show route arcs chosen
        route = []
        current = 0
        while True:
            next_nodes = [j for j in V if (current,j) in A and x[current,j,k].X > 0.5]
            if not next_nodes:
                break
            nxt = next_nodes[0]
            route.append(nxt)
            if nxt == 2*n+1:
                break
            current = nxt

        print(f"  Route: 0 -> " + " -> ".join(str(node) for node in route))

elif m.status == GRB.INFEASIBLE:
    m.computeIIS()
    m.write("infeasible.ilp")


# ------------------------------------------------------------
# SECOND MODEL: Extension of Furtado's Two-Index Formulation
# ------------------------------------------------------------

# ------------------------------------------------------------
# DUPLICATE DEPOTS FOR EACH COURIER
# ------------------------------------------------------------

print("\n\n=== Building Two-Index Model (SPDP-TW-2CC) ===")

m2 = gp.Model("SPDP_TW_2CC")

# K = [x + 1 for x in K]
# KC = list(range(1, num_comm_drivers+1))
# KO = list(range(num_comm_drivers+1, num_comm_drivers+num_occ_drivers+1))

# ------------------------------------------------------------
# 1. Duplicate depots for each courier
# ------------------------------------------------------------
O = [f"o{k}" for k in K]  # origin depot for each courier
F = [f"f{k}" for k in K]  # end depot for each courier
V2 = O + P + D + F

# Arc sets
A1_2 = [(o, j) for o in O for j in P]                       # origin depots -> pickups
A2_2 = [(i, j) for i in P + D for j in P + D if i != j]     # between pickups/deliveries
A3_2 = [(i, f) for f in F for i in D]  + [(o,f) for o in O for f in F]        # deliveries -> end depots
A2CC = A1_2 + A2_2 + A3_2

# ------------------------------------------------------------
# 2. Distances
# ------------------------------------------------------------
coords2 = {}
for req in instance["requests"]:
    pid = req["id"]
    coords2[pid] = req["pickup"]
    coords2[pid + num_requests] = req["dest"]

for k, (ox, oy) in enumerate(ok):
    coords2[f"o{k+1}"] = (ox, oy)
    coords2[f"f{k+1}"] = (ox, oy)

dij2_A2 = {(i,j): math.dist(coords2[i], coords2[j]) for (i,j) in A1_2+A2_2}
dij2_A3 = {(i,j): 0 for (i,j) in A3_2}
dij2 = {**dij2_A2, **dij2_A3}
tij2 = dij2.copy()

for o in O:
    si[o] = 0
for f in F:
    si[f] = 0

# ------------------------------------------------------------
# 3. Variables
# ------------------------------------------------------------
x2 = m2.addVars(A2CC, vtype=GRB.BINARY, name="x")
y2 = m2.addVars(V2, K, vtype=GRB.BINARY, name="y")
T2 = m2.addVars(V2, vtype=GRB.CONTINUOUS, lb=0, name="T")
v2 = m2.addVars(V2, vtype=GRB.CONTINUOUS, lb=0, name="v")
w2 = m2.addVars(A2CC, vtype=GRB.CONTINUOUS, lb=0, name="w")

# ------------------------------------------------------------
# 4. Objective (eq. 16)
# ------------------------------------------------------------
obj2 = gp.quicksum(p[i] * gp.quicksum(y2[i, k] for k in K) for i in P)
obj2 -= gp.quicksum((wC * (Lk[k-1] - Ek[k-1])) for k in KC)
obj2 -= gp.quicksum((cO * gp.quicksum(y2[i, k] for i in P)) for k in KO)
obj2 -= gp.quicksum(w2[i, j] for (i, j) in A2CC)
m2.setObjective(obj2, GRB.MAXIMIZE)

# ------------------------------------------------------------
# 5. Constraints
# ------------------------------------------------------------

# (17) Each pickup served at most once
m2.addConstrs(gp.quicksum(x2[i, j] for j in V2 if (i, j) in A2CC) <= 1 for i in P)

# (18) Flow conservation for pickups/deliveries
m2.addConstrs(
    gp.quicksum(x2[j, i] for j in V2 if (j, i) in A2CC) -
    gp.quicksum(x2[i, j] for j in V2 if (i, j) in A2CC) == 0
    for i in P + D
)

# (19) Depart from each courier’s own origin
m2.addConstrs(
    gp.quicksum(x2[o, j] for j in V2 if (o, j) in A2CC) == 1
    for o in O
)

# (20) Return to each courier’s own finish depot
m2.addConstrs(
    gp.quicksum(x2[j, f] for j in V2 if (j, f) in A2CC) == 1
    for f in F
)

# (21) Time propagation along arcs
m2.addConstrs(
    T2[j] >= T2[i] + tij2[i, j] + si[j] - H * (1 - x2[i, j])
    for (i, j) in A2CC
)

# (22) Node time windows
m2.addConstrs(ei[i] <= T2[i] for i in P + D)
m2.addConstrs(T2[i] <= li[i] for i in P + D)

# (23) Precedence
m2.addConstrs(T2[i] + tij[i,i+n] + si[i+n] <= T2[i+n] for i in P)

# (24–25) Courier time constraints
m2.addConstrs(Ek[k-1] <= T2[f"o{k}"] for k in K)
m2.addConstrs(T2[f"f{k}"] <= Lk[k-1] for k in K)

# (26–27) v-index propagation
m2.addConstrs(v2[j] >= v2[i] - len(K) * (1 - x2[i, j]) for (i, j) in A2_2+A3_2)
m2.addConstrs(v2[j] <= v2[i] + len(K) * (1 - x2[i, j]) for (i, j) in A2_2+A3_2)

# (28–29) v-index from origin depots
m2.addConstrs(v2[j] >= k * x2[f"o{k}",j] for k in K for j in P+[f"d{k}"] if (f"o{k}", j) in A2CC)
m2.addConstrs(v2[j] <= k * x2[f"o{k}",j] + len(K) * (1 - x2[f"o{k}", j]) for k in K for j in P+[f"d{k}"] if (f"o{k}", j) in A2CC)

# (30) Pickup and delivery same courier
m2.addConstrs(v2[i] == v2[i + n] for i in P)

# (31) Linking v with y
m2.addConstrs(v2[j] == gp.quicksum(k * y2[j, k] for k in K) for j in V2)

# (32) Linking x with y
m2.addConstrs(gp.quicksum(y2[i, k] for k in K) == gp.quicksum(x2[i,j] for j in V2 if (i,j) in A2CC) for i in O+P+D)

# (33)-(34) Initital and final conditions
m2.addConstrs(v2[f"o{k}"] == k for k in K)
m2.addConstrs(v2[f"f{k}"] == k for k in K)

# (35) w_ij cost factors
def delta_cost(i, j, k):
    delta = 0

    if k in KC:
        if (i, j) in A1_2 or (i, j) in A2_2:
            delta = deltaC

    elif k in KO:
        if i in P and j in D:
            delta = deltaO

    return delta

m2.addConstrs(
    (w2[i, j] >= delta_cost(i, j, k) * dij2[i, j]
     - 1000 * (3 - y2[i, k] - y2[j, k] - x2[i,j]))
    for k in K for (i, j) in A1_2+A2_2
)

# m2.addConstrs(
#     (w2[i, j] <= delta_cost(i, j, k) * dij2[i, j]
#      + 1000 * (2 - y2[i, k] - y2[j, k]))
#     for k in K for (i, j) in A2CC
# )

# ------------------------------------------------------------
# 6. Solve
# ------------------------------------------------------------
m2.write("model.lp")
print("\nSolving Extended Model...")
m2.optimize()

if m2.status == GRB.OPTIMAL:
    print("\n=== Extended Model Objective ===")
    print(round(m2.objVal, 4))

    print("\n=== Courier Schedules ===")

    for k in K:
        start_time = T2[f"o{k}"].X
        end_time   = T2[f"f{k}"].X

        print(f"\nCourier {k}:")
        print(f"  Start time: {start_time:.2f}  (Earliest allowed {Ek[k-1]:.2f})")
        print(f"  End time:   {end_time:.2f}  (Latest allowed {Lk[k-1]:.2f})")

        # list served pickups
        served = [i for i in P if y2[i,k].X > 0.5]
        if not served:
            print("  No requests assigned")
            continue

        print("  Served requests:")
        for i in served:
            pickup_time   = T2[i].X
            delivery_time = T2[i+n].X
            print(f"    Request {i}: "
                    f"Pickup at {pickup_time:.2f} (TW [{ei[i]:.2f},{li[i]:.2f}]), "
                    f"Delivery at {delivery_time:.2f} (TW [{ei[i+n]:.2f},{li[i+n]:.2f}])")

        # show route arcs chosen
        route = []
        current = f"o{k}"
        while True:
            next_nodes = [j for j in V2 if (current,j) in A2CC and x2[current,j].X > 0.5]
            if not next_nodes:
                break
            nxt = next_nodes[0]
            route.append(nxt)
            if nxt == f"f{k}":
                break
            current = nxt

        print(f"  Route: 0 -> " + " -> ".join(str(node) for node in route))

    # print("X")
    # for (i,j) in A2CC:
    #     if x2[i,j].X > 0.2:
    #         print(i, j, x2[i,j].X)

    # print("Y")
    # for i in P:
    #     for k in K:
    #         if y2[i,k].X > 0.5:
    #             print(i, k, y2[i,k].X)

    # print("V")
    # for i in P+D:
    #     print(i, v2[i])

    # print("T2")
    # for i in V2:
    #     print(i, T2[i])

    # # Loop through all keys in w2
    # for (i, j), value in w2.items():
    #     expected = dij2.get((i, j))
    #     if expected is None:
    #         print(f"⚠️ Missing dij2[{i},{j}]")
    #         continue

    #     expected_val = expected * 0.2
    #     is_positive = value.X > 0
    #     if is_positive:
    #         is_same = abs(value.X - expected_val) < 1e-9  # float-safe comparison

    #         print(f"w2[{i},{j}] = {value.X:.3f} | positive={is_positive} | "
    #             f"expected={expected_val:.3f} | match={is_same}")

    # print(gp.quicksum(p[i] * gp.quicksum(y2[i, k] for k in K) for i in P).getValue())
    # print(gp.quicksum((wC * (Lk[k-1] - Ek[k-1])) for k in KC).getValue())
    # print(gp.quicksum((cO * gp.quicksum(y2[i, k] for i in P)) for k in KO).getValue())
    # print(gp.quicksum(w2[i, j] for (i, j) in A2CC).getValue())

elif m2.status == GRB.INFEASIBLE:
    m2.computeIIS()
    m2.write("infeasible.ilp")