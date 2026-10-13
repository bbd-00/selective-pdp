import math
import gurobipy as gp
from gurobipy import GRB

import math
import gurobipy as gp
from gurobipy import GRB
from instance_loader import *

# ------------------------------------------------------------
# LOAD INSTANCE FROM FILE
# ------------------------------------------------------------
instance = load_instance_from_txt("InstanceswithDemand/Instance_REQ15_CDR2_ODR8_FS0_SS0.txt")
# instance = load_instance_from_txt("InstanceswithDemand/Instance_REQ20_CDR2_ODR8_FS0_SS0.txt")
# instance = load_instance_from_txt("InstanceswithDemand/Instance_REQ20_CDR4_ODR6_FS0_SS0.txt")

H = int(instance["H"])

b = int(instance["b"])
num_comm_drivers = len(instance["committed"])
num_occ_drivers = len(instance["occasional"])
num_drivers = num_occ_drivers + num_comm_drivers
num_requests = len(instance["requests"])

# Node indexing
P = list(range(1, num_requests+1))
D = [i+num_requests for i in P]
KC = list(range(0, num_comm_drivers))
KO = list(range(num_comm_drivers, num_comm_drivers+num_occ_drivers))
n = len(P)
V = [0] + P + D + [2*n+1]

K = KC + KO

# K = list(range(num_requests))
# K=range(14)

# Arcs
A1 = [(0,j) for j in P]
A2 = [(i,j) for i in P+D for j in P+D if i != j]
A3 = [(i,2*n+1) for i in D] + [(0,2*n+1)]


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

Ek = {d["id"]: d["e"] for d in instance["committed"]} | {d["id"]: d["e"] for d in instance["occasional"]}
Lk = {d["id"]: d["l"] for d in instance["committed"]} | {d["id"]: d["l"] for d in instance["occasional"]}
Qk = {d["id"]: d["cap"] for d in instance["committed"]} | {d["id"]: d["cap"] for d in instance["occasional"]}
# CAP = {k-1: 200 for k in K}

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
print("H:",H)

# pruning unfeasible links and assignments
z_UB = {(pickup,driver): (1 if (li[pickup] - si[pickup] >= max(dijk[0,pickup,driver] + Ek[driver],ei[pickup]) and Lk[driver] >= ei[pickup + num_requests] + si[pickup + num_requests]) else 0) for driver in K for pickup in P} #cart_prod_pickups_destinations                        
x_UB = {(i,j): (0 if ( (ei[i] + dij[i, j] + si[j] > li[j]) or (i in D and j==i-num_requests) ) else 1) for (i,j) in A2} #fixation based on early/latest time and x[delta(p),p]=0
UBxIPC1 = {(n,d): 0 if (d in D and d-num_requests!=n and x_UB[d-num_requests,n]==0 ) else 1 for (n,d) in A2}
UBxIPC2 = {(p,n): 0 if (p in P and p+num_requests!=n and x_UB[n,p + num_requests]==0 ) else 1 for (p,n) in A2}
x_ind ={(i,j):x_UB[i,j]*UBxIPC1[i,j]*UBxIPC2[i,j] for (i,j) in A2}

# Update set of Arcs based on 
# A2 = [(i,j) for (i,j) in A2 if x_ind[i,j]==1] 
A = A1 + A2 + A3

# # define PK set so variable y is defined on this set
PK = [(p,k) for p in P for k in K if z_UB[p,k]==1]#if z_UB[p,k]==1

#Ek_org = Ek.copy() #keeping orginal Ek for per hour calculations of committed
# Tightening seems meaningful for latest time of driver as it hard however as the vehicle could wait earlier cannot be tightened
Lk_org = Lk.copy() #keeping orginal Lk for per hour calculations of committed

# ------------------------------------------------------------
# BUILD MODEL (eqns 1–15)
# ------------------------------------------------------------
m = gp.Model("SPDPTW2OCC")
m.Params.timeLimit = 3600

x = m.addVars(A, K, vtype=GRB.BINARY, name="x")
L = m.addVars(A2, vtype=GRB.CONTINUOUS, name="L")
y = m.addVars(P+D,K, vtype=GRB.BINARY, name="y") #extending the definitin of y
T = m.addVars(P+D+[0,2*n+1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")

obj = gp.quicksum(p[i] * gp.quicksum(y[i,k] for k in K ) for i in P)

obj -= gp.quicksum(
    (wC*(Lk_org[k]-Ek[k]) + deltaC*(
        gp.quicksum(dijk[0,j,k]*x[0,j,k] for (i,j) in A1) +
        gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2)
    )) for k in KC
)

obj -= gp.quicksum(
    (cO * gp.quicksum(y[i,k] for i in P) +
    deltaO * gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2))
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
## super valid MTZ constants

# m.addConstrs(
#     T[j,k] >= T[i,k] + tij[i,j] + si[j] - H*(1-x[i,j,k])
#     for (i,j) in A2 + A3 for k in K
# )

# Supervalid 
## super valid MTZ constants
svMTZ = {(i,j): min(-si[i] -dij[j,i], ei[j]- ei[i]) for (i,j) in A2}

# New y's constraints
m.addConstrs( y[i,k] == gp.quicksum(x[i,l,k] for l in V if (i,l) in A ) for i in P+D for k in K )
#m.addConstrs( y[i,k] == y[i+n,k] for i in P for k in K )
m.addConstrs( y[i,k] == 0 for i in P for k in K if z_UB[i,k]==0 )

for (i,j) in A2:
    for k in K:
        if (j,i) in A2:                        
            m.addConstr(                            
                T[i,k] - T[j,k] + (tij[i, j] + si[j]) * x[i, j , k] + svMTZ[i,j] * x[j, i, k] 
                <= li[i]*y[i,k] 
                - ei[j]*y[j,k] 
                - (li[i] - ei[j]) * (x[i, j, k] + x[j, i, k]  )                             
            )
        else:
            m.addConstr(                            
                T[i,k] - T[j,k] + (tij[i, j] + si[j]) * x[i, j , k]  
                <= li[i]*y[i,k] 
                - ei[j]*y[j,k] 
                - (li[i] - ei[j]) * (x[i, j, k] )                             
            )
            
#A3 = [(i,2*n+1) for i in D] + [(0,2*n+1)]        
# for (i,j) in A3:
#     for k in K:                    
#         if i!=0:
#             m.addConstr(                            
#                 T[i,k] - T[j,k] + (tij[i, j] + si[j]) * x[i, j , k]  
#                 <= li[i]*y[i,k] 
#                 - Ek[k]*gp.quicksum(x[0,ii,k] for ii in P if (ii,k) in PK) 
#                 - (li[i] - Ek[k]) * (x[i, j, k] )                          
#             )
m.addConstrs(                            
                T[i,k] - T[j,k] + (tij[i, j] + si[j]) * x[i, j , k]  
                <= li[i]*y[i,k] 
                - Ek[k]*gp.quicksum(x[0,ii,k] for ii in P if (ii,k) in PK) 
                - (li[i] - Ek[k]) * (x[i, j, k] )  
                for (i,j) in A3 for k in K if i!=0
            )

#A1 = [(0,j) for j in P]
m.addConstrs(                
    T[0,k] - T[j,k] + (tijk[0, j,k] + si[j]) * x[0, j , k] 
    <= Lk[k]*gp.quicksum(x[0,ii,k] for ii in P if (ii,k) in PK)
    - ei[j]*y[j,k] 
    - (Lk[k] - ei[j]) * (x[0, j, k] ) 
    for (i,j) in A1 for k in K
)
            
# (8a) Time windows 1
m.addConstrs((ei[i]*y[i,k] <= T[i,k]) for i in P+D for k in K )

# (8b) Time windows 2
m.addConstrs( (T[i,k] <= li[i]*y[i,k]) for i in P+D for k in K )

# (9) Pickup before delivery
m.addConstrs(T[i,k] + (tij[i,i+n] + si[i+n])*y[i,k] <= T[i+n,k] 
            for i in P for k in K )
# (11) Earliest departure
m.addConstrs(Ek[k]*gp.quicksum(x[0,i,k] for i in P if (i,k) in PK) <= T[0,k] for k in K)

# (12) Latest return
m.addConstrs(T[2*n+1,k] <= Lk[k]*gp.quicksum(x[0,i,k] for i in P if (i,k) in PK) for k in K)

# Load constraint flow type
# \sum_{j} y_{ij}^k \;-\; \sum_{h} y_{hi}^k &= \text{dem}[i] \sum_{h} x_{hi}^k, && \forall i, \forall k
m.addConstrs (gp.quicksum(L[i,j] for j in P+D if (i,j) in A2) - gp.quicksum(L[j,i] for j in P+D if (j,i) in A2) 
            >=  dem[i]*gp.quicksum(x[i,j,k] for k in K for j in P+D+[2*n+1] if (i,j) in A2+A3 ) for i in P+D)            
#0 &\le y_{ij}^k \le Q_k \, x_{ij}^k, && \forall (i,j), \forall k
m.addConstrs( L[i,j] <= gp.quicksum((Qk[k]+ min(dem[i],0))*x[i,j,k] for k in K ) for (i,j) in A2)
m.addConstrs( L[i,j] >= gp.quicksum((max(dem[i],0))*x[i,j,k] for k in K ) for (i,j) in A2) # lower bound on L[i,j]

# ####### Fixation ############
# #  if x_ijk = 0 if z_UB[(i if i in P else i-n),k]==0 or z_UB[(j if j in P else j-n)]==0 or ei[i]+tij[i,j]+si[j]>Lk[k]
# aijk = {(i,j,k): (0 if (max(ei[i],min(Ek[k],li[i]))+tij[i,j]+si[j] > Lk[k] 
#                     or z_UB[(i if i in P else i-n),k] == 0 
#                     or z_UB[(j if j in P else j-n),k] == 0 
#                     ) 
#         else 1) for (i,j) in A2 for k in K}
# print('aijk',len(aijk.values())-sum(aijk.values()))
# m.addConstrs ( x[i,j,k] == 0 for (i,j) in A2 for k in K if aijk[i,j,k] == 0) # swithcing off this constraint gave better results for NONE

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
    print(f"  Start time: {start_time:.2f}  (Earliest allowed {Ek[k]:.2f})")
    print(f"  End time:   {end_time:.2f}  (Latest allowed {Lk[k]:.2f})")

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

# # ------------------------------------------------------------
# # Loader matching save_instance_to_txt(...)
# # ------------------------------------------------------------
# def load_instance_from_txt(path):
#     with open(path, "r") as f:
#         lines = [ln.strip() for ln in f if ln.strip()]

#     idx = 0

#     def expect(token):
#         nonlocal idx
#         if lines[idx] != token:
#             raise ValueError(f"Expected '{token}', got '{lines[idx]}'")
#         idx += 1

#     expect("PARAMETERS")
#     T, b, W = map(float, lines[idx].split()); idx += 1
#     c_com_h, c_com_d, c_occ_o, c_occ_d, rev_order = map(float, lines[idx].split()); idx += 1

#     expect("COMMITTED_DRIVERS")
#     numC = int(lines[idx]); idx += 1
#     committed = []
#     for _ in range(numC):
#         did, x, y, e, l, cap = lines[idx].split()
#         committed.append({
#             "id": int(did),
#             "location": (float(x), float(y)),
#             "e": float(e),
#             "l": float(l),
#             "capacity": float(cap),
#         })
#         idx += 1

#     expect("OCCASIONAL_DRIVERS")
#     numO = int(lines[idx]); idx += 1
#     occasional = []
#     for _ in range(numO):
#         did, x, y, e, l, cap = lines[idx].split()
#         occasional.append({
#             "id": int(did),
#             "location": (float(x), float(y)),
#             "e": float(e),
#             "l": float(l),
#             "capacity": float(cap),
#         })
#         idx += 1

#     expect("REQUESTS")
#     numR = int(lines[idx]); idx += 1
#     requests = []
#     for _ in range(numR):
#         parts = lines[idx].split()
#         # rid px py dx dy delay_p delay_d eP lP eD lD demand
#         requests.append({
#             "id": int(parts[0]),
#             "pickup": (float(parts[1]), float(parts[2])),
#             "dest": (float(parts[3]), float(parts[4])),
#             "delay_p": float(parts[5]),
#             "delay_d": float(parts[6]),
#             "e_pickup": float(parts[7]),
#             "l_pickup": float(parts[8]),
#             "e_dest": float(parts[9]),
#             "l_dest": float(parts[10]),
#             "demand": float(parts[11]),
#         })
#         idx += 1

#     return {
#         "H": T,
#         "b": b,
#         "W": W,
#         "c_com_h": c_com_h,
#         "c_com_d": c_com_d,
#         "c_occ_o": c_occ_o,
#         "c_occ_d": c_occ_d,
#         "rev_order": rev_order,
#         "committed": committed,
#         "occasional": occasional,
#         "requests": requests
#     }


# # ------------------------------------------------------------
# # Reindex drivers to make Ek[k-1] logic valid
# # ------------------------------------------------------------
# def reindex_drivers_zero_based(instance):
#     drivers = instance["committed"] + instance["occasional"]
#     for new_id, d in enumerate(drivers):  # 0..m-1
#         d["id"] = new_id


# # ------------------------------------------------------------
# # Build the exact sets/data your model uses
# # ------------------------------------------------------------
# def build_model_data(instance):
#     H = int(instance["H"])
#     b = int(instance["b"])
#     num_comm_drivers = len(instance["committed"])
#     num_occ_drivers = len(instance["occasional"])
#     num_requests = len(instance["requests"])

#     # Node indexing
#     P = list(range(1, num_requests + 1))
#     D = [i + num_requests for i in P]
#     KC = list(range(1, num_comm_drivers + 1))
#     KO = list(range(num_comm_drivers + 1, num_comm_drivers + num_occ_drivers + 1))
#     n = len(P)
#     V = [0] + P + D + [2 * n + 1]
#     K = KC + KO

#     # Arcs
#     A1 = [(0, j) for j in P]
#     A2 = [(i, j) for i in (P + D) for j in (P + D) if i != j]
#     A3 = [(i, 2 * n + 1) for i in D] + [(0, 2 * n + 1)]
#     A = A1 + A2 + A3

#     # Coordinates: requests only
#     coords = {}
#     for req in instance["requests"]:
#         pid = req["id"]
#         coords[pid] = req["pickup"]
#         coords[pid + num_requests] = req["dest"]

#     # Driver initial locations (committed then occasional, in this exact order)
#     ok = [d["location"] for d in instance["committed"]] + [d["location"] for d in instance["occasional"]]

#     # Distances
#     dij_A2 = {(i, j): math.dist(coords[i], coords[j]) for (i, j) in A2}
#     dij_A3 = {(i, j): 0.0 for (i, j) in A3}
#     dij = {**dij_A2, **dij_A3}
#     dijk = {(i, j, k): math.dist(ok[k - 1], coords[j]) for (i, j) in A1 for k in K}
#     tij = dij.copy()
#     tijk = dijk.copy()

#     # Time windows and service times
#     si = {0: 0.0, 2 * num_requests + 1: 0.0}
#     ei, li = {}, {}
#     for req in instance["requests"]:
#         pid = req["id"]
#         ei[pid], li[pid] = req["e_pickup"], req["l_pickup"]
#         ei[pid + num_requests], li[pid + num_requests] = req["e_dest"], req["l_dest"]
#         si[pid] = req["delay_p"]
#         si[pid + num_requests] = req["delay_d"]

#     # Availability keyed by 0-based driver id; you use Ek[k-1], Lk[k-1]
#     Ek = {d["id"]: d["e"] for d in instance["committed"]} | {d["id"]: d["e"] for d in instance["occasional"]}
#     Lk = {d["id"]: d["l"] for d in instance["committed"]} | {d["id"]: d["l"] for d in instance["occasional"]}

#     # Costs and revenue (as in your code)
#     p = {i: instance["rev_order"] for i in P}
#     wC = instance["c_com_h"] / b
#     cO = instance["c_occ_o"]
#     deltaC = instance["c_com_d"]
#     deltaO = instance["c_occ_d"]

#     # Demand and capacity for load constraints
#     dem = {0: 0.0, 2 * n + 1: 0.0}
#     for req in instance["requests"]:
#         i = req["id"]
#         dem[i] = float(req["demand"])
#         dem[i + n] = -float(req["demand"])

#     # Qk keyed by 1..|K| (internal k), since you use k in constraints on L_ijk
#     Qk = {}
#     drivers = instance["committed"] + instance["occasional"]
#     for internal_k in K:
#         # internal_k=1 corresponds to drivers[0] (0-based id)
#         Qk[internal_k] = float(drivers[internal_k - 1]["capacity"])

#     return dict(
#         H=H, b=b,
#         num_requests=num_requests, num_comm=num_comm_drivers, num_occ=num_occ_drivers,
#         P=P, D=D, KC=KC, KO=KO, n=n, V=V, K=K,
#         A1=A1, A2=A2, A3=A3, A=A,
#         coords=coords, ok=ok,
#         dij=dij, dijk=dijk, tij=tij, tijk=tijk,
#         si=si, ei=ei, li=li,
#         Ek=Ek, Lk=Lk,
#         p=p, wC=wC, cO=cO, deltaC=deltaC, deltaO=deltaO,
#         dem=dem, Qk=Qk
#     )

