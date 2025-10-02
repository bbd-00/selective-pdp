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
KC = list(range(0, num_comm_drivers))
KO = list(range(num_comm_drivers, num_comm_drivers+num_occ_drivers))
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
dijk = {(i,j,k): math.dist(ok[k], coords[j]) for (i,j) in A1 for k in K}
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
m.addConstrs(Ek[k] <= T[0,k] for k in K)

# (12) Latest return
m.addConstrs(T[2*n+1,k] <= Lk[k] for k in K)

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
        print(f"  Start time: {start_time:.2f}  (Earliest allowed {Ek[k]:.2f})")
        print(f"  End time:   {end_time:.2f}  (Latest allowed {Lk[k]:.2f})")

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