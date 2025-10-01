#!/usr/bin/env python
# coding: utf-8

import math
import gurobipy as gp
from gurobipy import GRB

# ------------------------------------------------------------
# HARDCODED INSTANCE DATA
# ------------------------------------------------------------

# horizon
T = 600

# number of requests (pickups)
num_requests = 5
# number of drivers
num_comm_drivers = 2
num_occ_drivers = 4

P = list(range(1, num_requests+1))             # pickup indices
D = [i+num_requests for i in P]                # delivery indices
KC = list(range(0, num_comm_drivers))                                     # committed drivers
KO = list(range(num_comm_drivers, num_comm_drivers+num_occ_drivers))    # occasional drivers
n = len(P)

V = [0] + P + D + [2*n+1]
K = KC + KO

# Arcs
A1 = [(0,j) for j in P]
A2 = [(i,j) for i in P+D for j in P+D if i != j]
A3 = [(i,2*n+1) for i in D] + [(0,2*n+1)]
A = A1 + A2 + A3

# Driver initial locations
#NOTE: This list contains all drivers in order (committed, then occasional)
#NOTE: They are only used to compute distances for arcs in A1
ok = [(25.189142545745025, 45.51114009188783), (48.52633272698175, 6.133210101269265), (13.419810713489582, 1.1329761181920917), (9.042718542485218, 24.918763133241285), (37.43719404648841, 9.010582673410406), (0.3607474469261074, 47.88592959399976)]

# Simple Euclidean coordinates (hardcoded for reproducibility)
coords = {
    1: (32.34669415785327, 15.296098322319635),
    2: (6.347688873200408, 18.601341351063354),
    3: (45.53509319478232, 38.25895799856443),
    4: (21.442914886357627, 13.449716183946597),
    5: (49.98652272184054, 37.095726081156464),
    6: (31.6708195692333, 21.73725066831178),
    7: (37.639505417793664, 6.1009507088656765),
    8: (14.11086611557712, 2.3169118304723293),
    9: (21.840356906695384, 46.57844103735434), 
    10: (43.69370473930082, 14.714252307877585)
}

# Distances
dij_A2 = {(i,j): math.dist(coords[i], coords[j]) for (i,j) in A2}
dij_A3 = {(i,j): 0 for (i,j) in A3}
dij = {**dij_A2, **dij_A3}
dijk = {(i,j,k): math.dist(ok[k], coords[j]) for (i,j) in A1 for k in K}
tij = dij.copy()
tijk = dijk.copy()

# Service times
si = {
    0: 0,
    1: 3,
    2: 2,
    3: 5,
    4: 4,
    5: 3,
    6: 5,
    7: 4,
    8: 3,
    9: 5,
    10: 2,
    11: 0
}

# Time windows (simplified)
## Nodes
ei = {1: 551.9223689896112, 2: 442.09332023522626, 3: 525.0359642320418, 4: 419.3556746531706, 5: 470.7422744527197, 6: 558.3988842566218, 7: 475.7895744612975, 8: 572.778112594623, 9: 452.4867834570172, 10: 493.99157393518465}
li = {1: 611.9223689896112, 2: 502.09332023522626, 3: 585.0359642320418, 4: 479.3556746531706, 5: 530.7422744527197, 6: 618.3988842566218, 7: 535.7895744612974, 8: 632.778112594623, 9: 512.4867834570173, 10: 553.9915739351846}
## Couriers
Ek = {0: 120, 1: 0, 2: 361.2472338253136, 3: 325.4691990824467, 4: 346.177755064363, 5: 476.6227309078524}
Lk = {0: 240, 1: 120, 2: 421.2472338253136, 3: 385.4691990824467, 4: 406.177755064363, 5: 536.6227309078524}

# Profits and costs
p = {i: 20 for i in P}
wC = 0.00666667             # 0.4 (which we used in other file) deivded by 60
cC = 0.00833333

deltaC, deltaO = 0.2, 0.2
M = 1000

# ------------------------------------------------------------
# BUILD MODEL (eqns 1–15)
# ------------------------------------------------------------
m = gp.Model("SPDPTW2OCC")

x = m.addVars(A, K, vtype=GRB.BINARY, name="x")
y = m.addVars(P, K, vtype=GRB.BINARY, name="y")
T = m.addVars(P+D+[0,2*n+1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")

# Objective (1)
obj = gp.quicksum(p[i] * gp.quicksum(y[i,k] for k in K) for i in P)

obj += gp.quicksum(
    (wC*(Ek[k]-Lk[k]) + deltaC*(
        gp.quicksum(dijk[0,j,k]*x[0,j,k] for (i,j) in A1) +
        gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2)
    )) for k in KC
)

obj += gp.quicksum(
    (cC * gp.quicksum(y[i,k] for i in P) +
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
m.addConstrs(gp.quicksum(x[i,j,k] for (i,j) in A) == 1 for k in K)

# (6) Return depot
m.addConstrs(gp.quicksum(x[j,2*n+1,k] for j in P+D+[0] if (j,2*n+1) in A) == 1 for k in K)

# (7a) Time propagation 1
m.addConstrs(
    T[j,k] >= T[i,k] + tij[i,j] + si[j] - M*(1-x[i,j,k])
    for (i,j) in A2+A3 for k in K
)

# (7b) Time propagation 2
m.addConstrs(
    T[j,k] >= T[0,k] + tijk[i,j,k] + si[j] - M*(1-x[i,j,k])
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
    m.computeIIS("infeasible.ilp")