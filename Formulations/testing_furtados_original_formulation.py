#!/usr/bin/env python
# coding: utf-8

import math
import gurobipy as gp
from gurobipy import GRB
from instance_loader import *

# ------------------------------------------------------------
# LOAD INSTANCE FROM FILE
# ------------------------------------------------------------
# instance = load_instance_from_txt("InstanceswithDemand/Instance_REQ15_CDR2_ODR8_FS0_SS0.txt")
instance = load_instance_from_txt("InstanceswithDemand/Instance_REQ20_CDR2_ODR8_FS0_SS0.txt")

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

# K = KC + KO

# K = list(range(num_requests))
K=range(3)

# Arcs
A1 = [(0,j) for j in P]
A2 = [(i,j) for i in P+D for j in P+D if i != j]
A3 = [(i,2*n+1) for i in D] + [(0,2*n+1)]


# Build coordinates: requests only
coords = {}
coords[0] = (25, 25)
coords[2*n+1] = (25,25)
for req in instance["requests"]:
    pid = req["id"]
    coords[pid] = req["pickup"]
    coords[pid+num_requests] = req["dest"]

# # Build driver initial locations (in order committed+occasional)
# ok = [(25, 25) for driver in K]

# Distances
dij = {(i,j): math.dist(coords[i], coords[j]) for (i,j) in A1+A2+A3}

# dij_A1 = {(i,j): math.dist(coords[i], coords[j]) for (i,j) in A1}
# dij_A2 = {(i,j): math.dist(coords[i], coords[j]) for (i,j) in A2}
# dij_A3 = {(i,j): math.dist(coords[i], coords[j]) for (i,j) in A3}
# # dij_A3 = {(i,j): 0 for (i,j) in A3}
# dij = {**dij_A1, **dij_A2, **dij_A3}
tij = dij.copy()
# tijk = dijk.copy()

# Build time windows and service times
si = {0: 0, 2*num_requests+1: 0}
ei, li, dem = {}, {}, {} #demand + for pickup and - for delivery
for req in instance["requests"]:
    pid = req["id"]
    ei[pid], li[pid] = req["e_pickup"], req["l_pickup"]
    ei[pid+num_requests], li[pid+num_requests] = req["e_dest"], req["l_dest"]
    si[pid] = req["delay_p"]    # pickup service time
    si[pid+num_requests] = req["delay_d"]  # delivery service time
    dem[pid], dem[pid+num_requests] = req["l_dem"], -req["l_dem"] #demand + for pickup and - for delivery
dem[0] = 0
dem[2*n+1] = 0

# Ek = {d["id"]: d["e"] for d in instance["committed"]} | {d["id"]: d["e"] for d in instance["occasional"]}
# Lk = {d["id"]: d["l"] for d in instance["committed"]} | {d["id"]: d["l"] for d in instance["occasional"]}
# CAP = {d["id"]: d["cap"] for d in instance["committed"]} | {d["id"]: d["cap"] for d in instance["occasional"]}
CAP = {k-1: 200 for k in K}

# Costs and revenue
p = {i: instance["rev_order"] for i in P}

wC = instance["c_com_h"] / b   # committed cost/hour -> cost per minute
cO = instance["c_occ_o"]          # occasional cost per order
deltaC = instance["c_com_d"]      # committed distance cost (same as cC if you want)
deltaO = instance["c_occ_d"]      # occasional distance cost


H = 0
for mm in li.keys():
    if H < li[mm]:
            H = li[mm]

# pruning unfeasible links and assignments
x_UB1 = {(i,j): (0 if ( (ei[i] + dij[i, j] + si[j] > li[j]) or (i in D and j==i-num_requests) ) else 1) for (i,j) in A2} #fixation based on early/latest time and x[delta(p),p]=0
x_UB2 = {(i,j): (0 if (dem[i] + dem[j] > CAP[0]) else 1) for i in P for j in P if (i, j) in A2}
x_UB2 = x_UB2 | {(i, j+n): (0 if (dem[i] + dem[j] > CAP[0]) else 1) for i in P for j in P if (i, j+n) in A2}
x_UB2 = x_UB2 | {(i+n, j+n): (0 if (dem[i] + dem[j] > CAP[0]) else 1) for i in P for j in P if (i+n, j+n) in A2}
x_UB = {
    k: min(x_UB1.get(k, 1), x_UB2.get(k, 1))
    for k in set(x_UB1) | set(x_UB2)
}
# UBxIPC1 = {(n,d): 0 if (d in D and d-num_requests!=n and x_UB[d-num_requests,n]==0 ) else 1 for (n,d) in A2}
# UBxIPC2 = {(p,n): 0 if (p in P and p+num_requests!=n and x_UB[n,p + num_requests]==0 ) else 1 for (p,n) in A2}

# Update set of Arcs based on 
A2 = [(i,j) for (i,j) in A2 if x_UB[i,j]==1] 

A = A1 + A2 + A3

# ------------------------------------------------------------
# BUILD MODEL (eqns 1–15)
# ------------------------------------------------------------
m = gp.Model("SPDPTW2OCC")

x = m.addVars(A, K, vtype=GRB.BINARY, name="x")
# y = m.addVars(P, K, vtype=GRB.BINARY, name="y")
T = m.addVars(P+D+[0,2*n+1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")
Q = m.addVars(P+D+[0,2*n+1], K, vtype=GRB.CONTINUOUS, lb=0, name="Q")

# Objective (1)
# obj = gp.quicksum(p[i] * gp.quicksum(y[i,k] for k in K) for i in P)

# obj -= gp.quicksum(
#     (wC*(Lk[k-1]-Ek[k-1]) + deltaC*(
#         gp.quicksum(dijk[0,j,k]*x[0,j,k] for (i,j) in A1) +
#         gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2)
#     )) for k in KC
# )

# obj -= gp.quicksum(
#     (cO * gp.quicksum(y[i,k] for i in P) +
#      deltaO * gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2 if i in P and j in D))
#     for k in KO
# )

# m.setObjective(obj, GRB.MAXIMIZE)

obj = gp.quicksum(dij[i, j] * x[i, j, k] for k in K for (i, j) in A) + gp.quicksum(10_000 * x[0,j,k] for k in K for j in P if (0,j) in A)

m.setObjective(obj, GRB.MINIMIZE)

# (2) Each request served exactly once
m.addConstrs(gp.quicksum(x[i,j,k] for k in K for j in V if (i,j) in A) == 1 for i in P)

# (3) Vehicle flow conservation
m.addConstrs(
    gp.quicksum(x[j,i,k] for j in V if (j,i) in A) -
    gp.quicksum(x[i,j,k] for j in V if (i,j) in A) == 0
    for i in P+D for k in K
)

# (4) Pickup-delivery consistency
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
    for (i,j) in A for k in K
)

# # (7b) Time propagation 2
# m.addConstrs(
#     T[j,k] >= T[0,k] + tijk[i,j,k] + si[j] - H*(1-x[i,j,k])
#     for (i,j) in A1 for k in K
# )

# (8a) Time windows 1
m.addConstrs((ei[i] <= T[i,k]) for i in P+D for k in K)

# (8b) Time windows 2
m.addConstrs((T[i,k] <= li[i]) for i in P+D for k in K)

# (9) Pickup before delivery
m.addConstrs(T[i,k] + tij[i,i+n] + si[i+n] <= T[i+n,k] for i in P for k in K)

# # (10) y = arc usage
# m.addConstrs(y[i,k] == gp.quicksum(x[i,j,k] for j in V if (i,j) in A) for i in P for k in K)

# # (11) Earliest departure
# m.addConstrs(Ek[k-1] <= T[0,k] for k in K)

# # (12) Latest return
# m.addConstrs(T[2*n+1,k] <= Lk[k-1] for k in K)

#  Capacity propagation 1
m.addConstrs(
    Q[j,k] >= Q[i,k] + dem[j] - CAP[0]*(1-x[i,j,k])
    for (i,j) in A1+A2+A3 for k in K
)

# Capacity 1
m.addConstrs((Q[i, k] <= min(CAP[0], CAP[0]+dem[i])) for i in P+D+[0] for k in K)

# Capacity 2
m.addConstrs((max(0, dem[i]) <= Q[i, k]) for i in P+D+[0] for k in K)

# ------------------------------------------------------------
# SOLVE
# ------------------------------------------------------------
m.optimize()

if m.status == GRB.OPTIMAL:
    print("\n=== Optimal Objective Value ===")
    print(round(m.objVal, 4))
elif m.status == GRB.INFEASIBLE:
    m.computeIIS()
    m.write("infeasible.ilp")


print("\n=== Vehicle Schedules ===")

for k in K:
    start_time = T[0,k].X
    end_time   = T[2*n+1,k].X

    print(f"\nVehicle {k}:")
    # print(f"  Start time: {start_time:.2f}  (Earliest allowed {Ek[k-1]:.2f})")
    # print(f"  End time:   {end_time:.2f}  (Latest allowed {Lk[k-1]:.2f})")

    # list served pickups
    served = [i for i in P if gp.quicksum(x[i,j,k].X for j in P+D+[2*n+1] if (i,j) in A).getValue() > 0.5]
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
# O = [0 for k in K]  # origin depot for each courier
# F = [2*n+1 for k in K]  # end depot for each courier
# V2 = O + P + D + F

V2 = [0, 2*n+1] + P + D

# Arc sets
A1_2 = [(0, j) for j in P]                       # origin depots -> pickups
A2_2 = [(i, j) for i in P + D for j in P + D if i != j]     # between pickups/deliveries
A3_2 = [(i, 2*n+1) for i in D] + [(0,2*n+1)]        # deliveries -> end depots

# ------------------------------------------------------------
# 2. Distances
# ------------------------------------------------------------

coords2 = {}
coords2[0] = (25, 25)
coords2[2*n+1] = (25,25)
for req in instance["requests"]:
    pid = req["id"]
    coords2[pid] = req["pickup"]
    coords2[pid + num_requests] = req["dest"]

# for k, (ox, oy) in enumerate(ok):
#     coords2[f"o{k+1}"] = (ox, oy)
#     coords2[f"f{k+1}"] = (ox, oy)

dij2_A2 = {(i,j): math.dist(coords2[i], coords2[j]) for (i,j) in A1_2+A2_2+A3_2}
# dij2_A3 = {(i,j): 0 for (i,j) in A3_2}
dij2 = {**dij2_A2}
tij2 = dij2.copy()

# for o in O:
#     si[o] = 0
# for f in F:
#     si[f] = 0

# pruning unfeasible links and assignments
x2_UB1 = {(i,j): (0 if ( (ei[i] + dij2[i, j] + si[j] > li[j]) or (i in D and j==i-num_requests) ) else 1) for (i,j) in A2_2} #fixation based on early/latest time and x[delta(p),p]=0
x2_UB2 = {(i,j): (0 if (dem[i] + dem[j] > CAP[0]) else 1) for i in P for j in P if (i, j) in A2_2}
x2_UB2 = x2_UB2 | {(i, j+n): (0 if (dem[i] + dem[j] > CAP[0]) else 1) for i in P for j in P if (i, j+n) in A2_2}
x2_UB2 = x2_UB2 | {(i+n, j+n): (0 if (dem[i] + dem[j] > CAP[0]) else 1) for i in P for j in P if (i+n, j+n) in A2_2}
x2_UB = {
    k: min(x2_UB1.get(k, 1), x2_UB2.get(k, 1))
    for k in set(x2_UB1) | set(x2_UB2)
}
# UBxIPC1 = {(n,d): 0 if (d in D and d-num_requests!=n and x_UB[d-num_requests,n]==0 ) else 1 for (n,d) in A2}
# UBxIPC2 = {(p,n): 0 if (p in P and p+num_requests!=n and x_UB[n,p + num_requests]==0 ) else 1 for (p,n) in A2}

# Update set of Arcs based on 
A2_2 = [(i,j) for (i,j) in A2_2 if x2_UB[i,j]==1] 

A2CC = A1_2 + A2_2 + A3_2

# ------------------------------------------------------------
# 3. Variables
# ------------------------------------------------------------
x2 = m2.addVars(A2CC, vtype=GRB.BINARY, name="x")
# y2 = m2.addVars(V2, K, vtype=GRB.BINARY, name="y")
T2 = m2.addVars(V2, vtype=GRB.CONTINUOUS, lb=0, name="T")
Q2 = m2.addVars(V2, vtype=GRB.CONTINUOUS, lb=0, name="Q")
v2 = m2.addVars(V2, vtype=GRB.CONTINUOUS, lb=0, name="v")
# w2 = m2.addVars(A2CC, vtype=GRB.CONTINUOUS, lb=0, name="w")

# ------------------------------------------------------------
# 4. Objective (eq. 16)
# ------------------------------------------------------------
# obj2 = gp.quicksum(p[i] * gp.quicksum(y2[i, k] for k in K) for i in P)
# obj2 -= gp.quicksum((wC * (Lk[k-1] - Ek[k-1])) for k in KC)
# obj2 -= gp.quicksum((cO * gp.quicksum(y2[i, k] for i in P)) for k in KO)
# obj2 -= gp.quicksum(w2[i, j] for (i, j) in A2CC)
# m2.setObjective(obj2, GRB.MAXIMIZE)

obj2 = gp.quicksum(dij2[i,j] * x2[i,j] for (i,j) in A2CC) + gp.quicksum(10_000 * x2[0,j] for j in V2 if (0,j) in A2CC)
m2.setObjective(obj2, GRB.MINIMIZE)

# ------------------------------------------------------------
# 5. Constraints
# ------------------------------------------------------------

# (17) Each pickup served exactly once
m2.addConstrs(gp.quicksum(x2[i, j] for j in V2 if (i, j) in A2CC) == 1 for i in P)

#NOTE: I think this is needed here explicitly (instead of just 18) because the v constraints don't force this necessarily on their own (unlike the first formulation where this was in terms of x)
# (17) Each delivery served exactly once
m2.addConstrs(gp.quicksum(x2[j, i] for j in V2 if (j, i) in A2CC) == 1 for i in D)

# # (18) Flow conservation for pickups/deliveries
# m2.addConstrs(
#     gp.quicksum(x2[j, i] for j in V2 if (j, i) in A2CC) -
#     gp.quicksum(x2[i, j] for j in V2 if (i, j) in A2CC) == 0
#     for i in P + D
# )

# # (19) Depart from each courier’s own origin
# m2.addConstrs(
#     gp.quicksum(x2[o, j] for j in V2 if (o, j) in A2CC) == 1
#     for o in O
# )

# # (20) Return to each courier’s own finish depot
# m2.addConstrs(
#     gp.quicksum(x2[j, f] for j in V2 if (j, f) in A2CC) == 1
#     for f in F
# )

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

#  Capacity propagation 1
m2.addConstrs(
    Q2[j] >= Q2[i] + dem[j] - CAP[0]*(1-x2[i,j])
    for (i,j) in A1_2+A2_2+A3_2
)

# Capacity 1
m2.addConstrs((Q2[i] <= min(CAP[0], CAP[0]+dem[i])) for i in P+D+[0])

# Capacity 2
m2.addConstrs((max(0, dem[i]) <= Q2[i]) for i in P+D+[0])

# # (24–25) Courier time constraints
# m2.addConstrs(Ek[k-1] <= T2[f"o{k}"] for k in K)
# m2.addConstrs(T2[f"f{k}"] <= Lk[k-1] for k in K)

# # (26–27) v-index propagation
# m2.addConstrs(v2[j] >= v2[i] - num_requests * (1 - x2[i, j]) for (i, j) in A2_2+A3_2)
# m2.addConstrs(v2[j] <= v2[i] + num_requests * (1 - x2[i, j]) for (i, j) in A2_2+A3_2)

# (26–27) v-index propagation
m2.addConstrs(v2[j] >= v2[i] - n * (1 - x2[i, j]) for (i, j) in A2_2)
m2.addConstrs(v2[j] <= v2[i] + n * (1 - x2[i, j]) for (i, j) in A2_2)

# # (28–29) v-index from origin depots
# m2.addConstrs(v2[j] >= k * x2[f"o{k}",j] for k in K for j in P+[f"d{k}"] if (f"o{k}", j) in A2CC)
# m2.addConstrs(v2[j] <= k * x2[f"o{k}",j] + len(K) * (1 - x2[f"o{k}", j]) for k in K for j in P+[f"d{k}"] if (f"o{k}", j) in A2CC)

# (28–29) v-index from origin depots
m2.addConstrs(v2[j] >= j * x2[0,j] for j in P if (0, j) in A2CC)
m2.addConstrs(v2[j] <= j * x2[0,j] + n * (1 - x2[0, j]) for j in P if (0, j) in A2CC)

# (30) Pickup and delivery same courier
m2.addConstrs(v2[i] == v2[i + n] for i in P)

# # (31) Linking v with y
# m2.addConstrs(v2[j] == gp.quicksum(k * y2[j, k] for k in K) for j in V2)

# # (32) Linking x with y
# m2.addConstrs(gp.quicksum(y2[i, k] for k in K) == gp.quicksum(x2[i,j] for j in V2 if (i,j) in A2CC) for i in O+P+D)

# # (33)-(34) Initital and final conditions
# m2.addConstrs(v2[f"o{k}"] == k for k in K)
# m2.addConstrs(v2[f"f{k}"] == k for k in K)

# # (35) w_ij cost factors
# def delta_cost(i, j, k):
#     delta = 0

#     if k in KC:
#         if (i, j) in A1_2 or (i, j) in A2_2:
#             delta = deltaC

#     elif k in KO:
#         if i in P and j in D:
#             delta = deltaO

#     return delta

# m2.addConstrs(
#     (w2[i, j] >= delta_cost(i, j, k) * dij2[i, j]
#      - 1000 * (3 - y2[i, k] - y2[j, k] - x2[i,j]))
#     for k in K for (i, j) in A1_2+A2_2
# )

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

    # print("\n=== Courier Schedules ===")

    # for k in K:
    #     start_time = T2[f"o{k}"].X
    #     end_time   = T2[f"f{k}"].X

    #     print(f"\nCourier {k}:")
    #     print(f"  Start time: {start_time:.2f}  (Earliest allowed {Ek[k-1]:.2f})")
    #     print(f"  End time:   {end_time:.2f}  (Latest allowed {Lk[k-1]:.2f})")

    #     # list served pickups
    #     served = [i for i in P if y2[i,k].X > 0.5]
    #     if not served:
    #         print("  No requests assigned")
    #         continue

    #     print("  Served requests:")
    #     for i in served:
    #         pickup_time   = T2[i].X
    #         delivery_time = T2[i+n].X
    #         print(f"    Request {i}: "
    #                 f"Pickup at {pickup_time:.2f} (TW [{ei[i]:.2f},{li[i]:.2f}]), "
    #                 f"Delivery at {delivery_time:.2f} (TW [{ei[i+n]:.2f},{li[i+n]:.2f}])")

    #     # show route arcs chosen
    #     route = []
    #     current = f"o{k}"
    #     while True:
    #         next_nodes = [j for j in V2 if (current,j) in A2CC and x2[current,j].X > 0.5]
    #         if not next_nodes:
    #             break
    #         nxt = next_nodes[0]
    #         route.append(nxt)
    #         if nxt == f"f{k}":
    #             break
    #         current = nxt

    #     print(f"  Route: 0 -> " + " -> ".join(str(node) for node in route))

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

    print("\n=== Routes for Extended Model (SPDP-TW-2CC) ===")

# Find all unique courier IDs from v2 values (round to avoid tiny float errors)
route_ids = sorted(set(int(round(v2[i].X)) for i in P + D if v2[i].X > 0.1))

print(route_ids)

for k in route_ids:
    print(f"\Route {k}:")

    # Nodes belonging to courier k
    route_nodes = [i for i in V2 if abs(v2[i].X - k) < 1e-6]

    # Filter arcs for this courier
    arcs_k = [(i, j) for (i, j) in A2CC if x2[i, j].X > 0.5 and i in route_nodes+[0] and j in route_nodes+[2*n+1]]

    if not arcs_k:
        print("  No assigned requests.")
        continue

    # Build route
    route = []
    current = 0  # start from depot (0)
    while True:
        next_nodes = [j for (i, j) in arcs_k if i == current]
        if not next_nodes:
            break
        nxt = next_nodes[0]
        route.append(nxt)
        if nxt == 2 * n + 1:
            break
        current = nxt

    print("  Route: 0 -> " + " -> ".join(str(node) for node in route))

    # Print node times
    for node in route:
        if node in P:
            print(f"    Pickup {node}: T={T2[node].X:.2f} (TW [{ei[node]:.2f},{li[node]:.2f}])")
        elif node in D:
            print(f"    Delivery {node}: T={T2[node].X:.2f} (TW [{ei[node]:.2f},{li[node]:.2f}])")
        elif node == 0:
            print(f"    Start depot: T={T2[node].X:.2f}")
        elif node == 2*n+1:
            print(f"    End depot: T={T2[node].X:.2f}")