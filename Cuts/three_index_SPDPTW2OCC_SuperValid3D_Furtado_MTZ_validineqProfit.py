import math
import gurobipy as gp
from gurobipy import GRB
from instance_loader import *
from itertools import combinations
from typing import Dict, List, Tuple

def knapsack(k):# used to bounding the maximum number of requests that could be assigned to vehicle k
    # k: index of vehicle
    # function return the maximum number of requests that can assigned to k
    # PK = [(p,k) for p in P for k in K if z_UB[p,k]==1]#if z_UB[p,k]==1

    Setk = [p for (p,d) in PK if d==k] # retreiving the set of pickups that could be assigned to driver k
    ks = gp.Model("knapsack")
    z = ks.addVars(Setk, vtype=GRB.BINARY, name="z")

    obj = gp.quicksum(z[p] for p in Setk)
    ks.setObjective(obj, GRB.MAXIMIZE)

    ks.addConstr( gp.quicksum(dem[p]*z[p] for p in Setk)<=Qk[k] )
    for p1 in Setk:
        for p2 in Setk:
            # incompatible for pickups or pickup and destination or destiantions
            if p1!=p2 and (x_ind[p1,p2] + x_ind[p2,p1]==0 or x_ind[p1,p2+n] + x_ind[p2+n,p1]==0 or x_ind[p1+n,p2] + x_ind[p2,p1+n]==0 or x_ind[p1+n,p2+n] + x_ind[p2+n,p1+n]==0):#nodes p1 and p2 are incompatible
                #print("incompatibility")
                ks.addConstr(z[p1]+z[p2]<=1)

    #ks.addConstrs (z[p1]+z[p2] <= max(x_ind[p1,p2],x_ind[p2,p1]) + 1 for p1 in Setk for p2 in Setk if p1!=p2)
    ks.setParam('LogToConsole', 0)
    ks.optimize()

    print(Setk)
    filtered_Setk = []
    for p in Setk:
        if z[p].X > 0.5:
            filtered_Setk.append((p,k))
            
    maxSetk_size = ks.getObjective().getValue()
    print(filtered_Setk,maxSetk_size)
    
    # for subset in combinations(Setk,maxSetk_size):
    #     if sum([dem[s] for s in subset])<=Qk[k]:
    #         print(subset)

    return ks.getObjective().getValue(), ks.Runtime

def buil_solve_3d(KO,KC,K,Nkmax_loc,maxobj,tot_req_loc,ObjType):
    # KO set of occasionl
    # KC set of committed
    # k SET OF DRIVERS 
    # Nkmax_loc dictionary {k:NN} on number of orders to be assigned to driver k
    # maxobj dictionary {k:maxprofitk} maxprofitk for vehicle k when solved individually
    # tot_req_loc bound on the max number of vehicles to be served mainly for the full problem
    # ObjType can be 'profit' or '#_requests'
  
    m = gp.Model("SPDPTW2OCC")
    m.Params.timeLimit = TimeLim

    x = m.addVars(A, K, vtype=GRB.BINARY, name="x")
    L = m.addVars(A2, vtype=GRB.CONTINUOUS, name="L")
    y = m.addVars(P+D,K, vtype=GRB.BINARY, name="y") #extending the definitin of y
    T = m.addVars(P+D+[0,2*n+1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")

    # Objective (1)
    if ObjType=='profit':
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
    elif ObjType=='#_requests':
        obj = gp.quicksum(gp.quicksum(y[i,k] for k in K ) for i in P)
    else: 
        print("Wrong objective specified")

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

    ####### Fixation ############
    #  if x_ijk = 0 if z_UB[(i if i in P else i-n),k]==0 or z_UB[(j if j in P else j-n)]==0 or ei[i]+tij[i,j]+si[j]>Lk[k]
    aijk = {(i,j,k): (0 if (max(ei[i],min(Ek[k],li[i]))+tij[i,j]+si[j] > Lk[k] 
                        or z_UB[(i if i in P else i-n),k] == 0 
                        or z_UB[(j if j in P else j-n),k] == 0 
                        ) 
            else 1) for (i,j) in A2 for k in K}
    print('aijk',len(aijk.values())-sum(aijk.values()))
    m.addConstrs ( x[i,j,k] == 0 for (i,j) in A2 for k in K if aijk[i,j,k] == 0) # swithcing off this constraint gave better results for NONE
    

    # -----------------------------------------------------------------------
    ################   Valid inequalities #########################
    # -----------------------------------------------------------------------

    if vineq == 'Arc' or vineq == 'ALL':
        ############### x2[i,j]+x2[j,i]<=1 this is called Arc selection Valid inequalities in GTSP ######################
        m.addConstrs((x[i,j,k] +  x[j,i,k] <= y[i,k] for (i,j) in A2 for k in K if (j,i) in A2), name="arc selection") #211 349 65%
        #m.addConstrs( (x[i,j,k] +  x[j,i,k] <= (3 - y[i,k] - y[j,k]) for (i,j) in A2 for k in K if (j,i) in A2), name="arc selection") #218 333 52%
        # m2.addConstrs((x2[i,j] +  x2[j,i] <= (3 - gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2) 
        #                                       - gp.quicksum(y2[j,k] for k in K if (j,k) in PDK2)) 
        #                for (i,j) in A2_2 if (j,i) in A2_2), name="arc selection") #218 333 52%
    elif vineq =='SOP' or vineq == 'ALL' :
        # SOP valid constraints
        for i in P+D:
            #d = p+num_pickups
            #successors = [d]
            successors = []
            for n1 in P+D:
                # is is predesessor of j if i ≺ j if Ei + Ti j ≤ L j and E j + Tji > Li.
                if (i,n1) in A2 and (ei[i] + si[i] + tij[i, n1] <= li[n1]) and (ei[n1] + si[n1] + tij[n1,i] > li[i]):
                        successors.append(n1)                        
            #m.addConstr( quicksum(x[i,p] for i in all_points_set if i not in successors and i!=p) == z1d[p], name="sop valid inequalities" )
            m.addConstrs((gp.quicksum(x[n,i,k] 
                for n in [0] + P + D for k in K if n not in successors and n!=i and (n,i) in A1 + A2) 
                >= y[i,k] for k in K if ( (i if i in P else i-num_requests),k) in PK), name="sop valid inequalities")
            #gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2)


    elif vineq=='2PC' or vineq == 'ALL': 
        # 2PC Valid inequalities
        for i in P+D:
            for j in P+D:
                if (i,j) not in A2 and (j,i) not in A2: # means cannot be together no need to check fij and fji       
                    m.addConstrs( (gp.quicksum(x[i,n,k] for n in P + D + [2*n+1] 
                                            if (i,n) in A2+A3 and n!=i and n!=j ) 
                                == y[i,k] for k in K 
                                if ( (i if i in P else i-num_requests),k) in PK), name="2PC_i") #what comes OUT In into set S={i,j}    
                    m.addConstrs( (gp.quicksum(x[j,n,k] for n in P + D + [2*n+1] if (j,n) in A2+A3 and n!=i and n!=j ) 
                                == y[j,k] for k in K if ( (j if j in P else j-num_requests),k) in PK), name="2PC_j") #what comes OUT In into set S={i,j}
    elif vineq=='RC' or vineq == 'ALL':
        # viol_pick_lst contains list of pickups that cannot be served in sequence due to load constraint
        for key in viol_pick_lst.keys(): # key is vehicle id                 
            for lst in viol_pick_lst[key]: # lst is set on node with total load larger than vehicle cap.                    
                if all((i,key) in PK for i in lst):                            
                    # m.addConstr(x[i,j,key]+x[j,k,key] 
                    #             <= 1 , name='load valid inequality')
                    # m.addConstr(x[k,j,key]+x[j,i,key] 
                    #             <= 1 , name='load valid inequality')
                    
                    m.addConstr(gp.quicksum(x[i,l,key] for l in P+D+[2*n+1] for i in lst if l !=i and (i,l) in A2+A3)
                                >= gp.quicksum(y[i,key] for i in lst) - len(lst) + 2 )
                    
                    m.addConstr(gp.quicksum(x[i+num_requests,l,key] for l in P+D+[2*n+1] for i in lst
                                            if l != i+num_requests and (i+num_requests,l) in A2+A3) 
                                >= gp.quicksum(y[i,key] for i  in lst) - len(lst) + 2 )

    ############################################ Profit valid inequalities#################
    m.addConstrs( (gp.quicksum(y[i,k] for i in P if (i,k) in PK) <= Nkmax_loc[k]) for k in K)
    #m.addConstr( gp.quicksum(gp.quicksum(y[i,k] for i in P if (i,k) in PK) for k in K) <=  tot_req_loc ) #sum(Nkmax_loc[k] for k in K)

    # for k in KC:        
    #     m.addConstr( gp.quicksum(p[i] * y[i,k] for i in P if (i,k) in PK)
    #                 - (wC*(Lk_org[k]-Ek[k]) + deltaC*
    #                    (gp.quicksum(dijk[0,j,k]*x[0,j,k] for (i,j) in A1) +
    #                     gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2)
    #                    )
    #                   ) <= maxobj[k]
    #                )
    # for k in KO:
    #     m.addConstr( gp.quicksum(p[i] * y[i,k] for i in P if (i,k) in PK)
    #                 - (cO * gp.quicksum(y[i,k] for i in P if (i,k) in PK) 
    #                    + deltaO * gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2 if i in P and j in D)
    #                   )
    #                 <= maxobj[k]
    #                 )
    ####################################### 2 nodes incompatibility #################################
    # for k in K:
    #     Setk = []
    #     Setk = [p for (p,d) in PK if d==k] # retreiving the set of pickups that could be assigned to driver k
    #     for p1 in Setk:
    #         for p2 in Setk:
    #             # incompatible for pickups or pickup and destination or destiantions
    #             if p1!=p2 and (x_ind[p1,p2] + x_ind[p2,p1]==0 
    #                            #or x_ind[p1,p2+num_requests] + x_ind[p2+num_requests,p1]==0 
    #                            #or x_ind[p1+num_requests,p2] + x_ind[p2,p1+num_requests]==0 
    #                            or x_ind[p1+num_requests,p2+num_requests] + x_ind[p2+num_requests,p1+num_requests]==0
    #                            ):#nodes p1 and p2 are incompatible
    #                 #print("incompatibility")
    #                 m.addConstr(y[p1,k]+y[p2,k]<=1)    
    # ------------------------------------------------------------
    # SOLVE
    # ------------------------------------------------------------
    m.optimize()

    if m.SolCount > 0: # feasible solution found could optimal or time reached
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
            served = [i for i in P if (i,k) in PK and y[i,k].X > 0.5 ]
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
                next_nodes = [j for j in V if (current,j) in A and x[current,j,k].X > 0.9]
                if not next_nodes:
                    break
                nxt = next_nodes[0]
                route.append(nxt)
                if nxt == 2*n+1:
                    break
                    
                route.append( (nxt,f"L[{current,nxt}]",(L[current,nxt].X if (current != 0 and nxt != 2*n+1 and (current,nxt) in A2) else 0)) )
                
                current = nxt
            
            print(f"  Route: 0 -> " + " -> ".join(str(node) for node in route))

    elif m.status == GRB.INFEASIBLE:
        m.computeIIS()
        m.write("infeasible.ilp")
    

    if  m.SolCount> 0:
        GAP = 100*(1-m.getObjective().getValue()/m.ObjBound) if m.ObjBound!=0 else 0
        output_list = ("3D_formulation",nbr_req, nbr_cdr, nbr_odr, m.Runtime, m.getObjective().getValue(), GAP, m.NumConstrs, m.NumVars, m.NumBinVars, m.NumIntVars)#, m.NumContVars
        #print("3D_formulation",nbr_req, nbr_cdr, nbr_odr, m.Runtime, m.getObjective().getValue(), GAP, m.NumConstrs, m.NumVars, m.NumBinVars, m.NumIntVars, file=out_file)#, m.NumContVars
    
    
    return output_list, m.Runtime, m.getObjective().getValue(),sum([y[i,k].X for i in P for k in K if (i,k) in PK])


def violating_subsets_fixed_size(
    node_demands: Dict,
    vehicle_capacity: float,
    subset_size: int,
    indicator:Dict,
    minimal_only: bool = False
) -> List[Tuple[Tuple, float]]:
    """
    Return violating subsets of exactly `subset_size` nodes.

    A subset S violates capacity if sum(d[i] for i in S) > Q.
    If minimal_only=True, additionally require that removing any one node
    makes the subset non-violating (i.e., <= Q).

    Parameters
    ----------
    node_demands : dict
        {node_id: demand}
    vehicle_capacity : float
        Q
    subset_size : int
        Exact size of subsets to consider
    minimal_only : bool, optional
        Keep only minimal violators among subsets of this size
    indicator : dictionary of (i , j): 1 meaning can be together 0 otherwise
    Returns
    -------
    List[(subset_tuple, total_demand)]
    """
    nodes = list(node_demands.keys())
    results = []

    for subset in combinations(nodes, subset_size):
                  
        total = sum(node_demands[i] for i in subset)
        if total > vehicle_capacity:
            feasible_subset = True # set initially true
            for pair in combinations(subset, 2):
                if indicator[pair]==0:
                    feasible_subset = False # pair cannot be together
                    break
            if feasible_subset:            
                if minimal_only:
                    # minimal within fixed size: removing any one node must fix feasibility
                    if all(total - node_demands[j] <= vehicle_capacity for j in subset):
                        results.append((subset))#, total
                else:
                    results.append((subset))#, total

    return results
    

# ------------------------------------------------------------
# LOAD INSTANCE FROM FILE
# ------------------------------------------------------------

filename = '3D_model_complexity.txt'
out_file = open(filename, 'a')
print("Num.req.", "Num.com.", "Num.occ.", "Runtime(sec)", "ObjValue", "GAP(%)", "SL", "SL_occ", "comdriverutilisation","occriverutilisation", "NumConstrs", "NumVars", "NumBinVars", "NumIntVars", "total time incl decomposition (sec)", 'Runtime (sec)', file=out_file)

TimeLim = 3600
for vineq in ['None']: #add valid cuts,'None','ALL','SOP','2PC','RC']:#'SOP','Arc'
    nbr_req = 50
    while nbr_req <= 50:# in range(40,100,5): #40,80,40 # 5,80,10
        for nbr_cdr in range(2,12,12): #2,8,2 # 2,8,2)
            for nbr_odr in [10 - nbr_cdr]: #6 - nbr_cdr           
                #Instances contain untightened TW
                #InstancesTTW contain tightened TW
                # instance = load_instance_from_txt(f"InstanceswithDemand/Instance_REQ{nbr_req}_CDR{nbr_cdr}_ODR{nbr_odr}_FS0_SS0.txt")
                instance = load_instance_from_txt("InstanceswithDemand/Instance_REQ15_CDR2_ODR8_FS0_SS0.txt")
                out_file = open(filename, 'a')

                #H = int(instance["H"])
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
                dij = {**dij_A2, **dij_A3} #pickups , destination, final destination
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
                
                Ek = {d["id"]: d["e"] for d in instance["committed"]} | {d["id"]: d["e"] for d in instance["occasional"]}
                Lk = {d["id"]: d["l"] for d in instance["committed"]} | {d["id"]: d["l"] for d in instance["occasional"]}
                Qk = {d["id"]: d["cap"] for d in instance["committed"]} | {d["id"]: d["cap"] for d in instance["occasional"]}
                
                # Set H to the maximum lastest time for requests
                H = 0
                for mm in li.keys():
                    if H < li[mm]:
                        H = li[mm]
                print("H:",H)
                
                # pruning unfeasible links and assignments
                z_UB = {(pickup,driver): (1 if (li[pickup] - si[pickup] >= max(dijk[0,pickup,driver] + Ek[driver], ei[pickup] - si[pickup]) and Lk[driver] >= ei[pickup + num_requests]) else 0) for driver in K for pickup in P} #cart_prod_pickups_destinations                        
                x_UB = {(i,j): (0 if ( (ei[i] + dij[i, j] + si[j] > li[j]) or (i in D and j==i-num_requests) ) else 1) for (i,j) in A2} #fixation based on early/latest time and x[delta(p),p]=0
                UBxIPC1 = {(n,d): 0 if (d in D and d-num_requests!=n and x_UB[d-num_requests,n]==0 ) else 1 for (n,d) in A2}
                UBxIPC2 = {(p,n): 0 if (p in P and p+num_requests!=n and x_UB[n,p + num_requests]==0 ) else 1 for (p,n) in A2}
                x_ind ={(i,j):x_UB[i,j]*UBxIPC1[i,j]*UBxIPC2[i,j] for (i,j) in A2}

                # Update set of Arcs based on 
                A2 = [(i,j) for (i,j) in A2 if x_ind[i,j]==1] 
                A = A1 + A2 + A3

                # define PK set so variable y is defined on this set
                PK = [(p,k) for p in P for k in K if z_UB[p,k]==1]#if z_UB[p,k]==1
                
                #Ek_org = Ek.copy() #keeping orginal Ek for per hour calculations of committed
                # Tightening seems meaningful for latest time of driver as it hard however as the vehicle could wait earlier cannot be tightened
                Lk_org = Lk.copy() #keeping orginal Lk for per hour calculations of committed
                
                for k in K:
                    #Ek[k] = max(Ek[k],min( [ei[p] for p in P if (p,k) in PK] ) )  #tightening Ek of drivers based on earliest time of pickup node in PK
                    mylist = [li[p+n] for p in P if (p,k) in PK]
                    maxlist = max(mylist) if  mylist else Lk[k] # preventing taking max of empty list
                    Lk[k] = min(Lk[k], maxlist ) # tightening Lk of drivers based on latest time of destinations of pickup node in PK
                              
                print('New tot driver TW length after tightening',sum([Lk[i]-Ek[i] for i in Ek.keys()]))
                print('Orginal tot driver TW length',sum([Lk_org[i]-Ek[i] for i in Ek.keys()]))
                
                # Costs and revenue
                p = {i: instance["rev_order"] for i in P}

                wC = instance["c_com_h"] / b   # committed cost/hour -> cost per minute
                cO = instance["c_occ_o"]          # occasional cost per order
                deltaC = instance["c_com_d"]      # committed distance cost (same as cC if you want)
                deltaO = instance["c_occ_d"]      # occasional distance cost

                #------------ constructing lists of nodes that can be together but there tot load is greater than veh. capacity
                if vineq=='RC':
                    pick_dem = {i: dem[i] for i in dem.keys() if dem[i]>0}
                    viol_pick_lst = {}
                    for k in K:
                        temp_list = [] 
                        for set_siz in range(4,6,1):
                            temp = violating_subsets_fixed_size(pick_dem, Qk[k], set_siz,x_ind,minimal_only=True)
                            if temp:
                                temp_list += temp
                        if temp_list:#not empty sets are added
                            viol_pick_lst  =  {**viol_pick_lst,**{k: temp_list }}
                            
                    print(viol_pick_lst)

                # first solving individually per single vehicle
                tot_time_single_veh, tot_profit_single_veh = 0, 0
                list_nkmax_ks = {} # list containing max number of requests to assign to drivers
                list_nkmax = {} # list containing max number of requests to assign to drivers
                list_maxprofit = {} # list containing max profit per driver
                tot_nkmax = 0 # tot number of requests to be served found using decomposition per vehicle
                                
                for k in K:
                    if k in KC:#single committed courier
                        KO_l = []
                        KC_l = [k]
                    else:# single occasional courier
                        KO_l = [k]
                        KC_l = []
                    print(f"########## Solving knapsack for couier {k} ###########")
                    list_nkmax_ks[k], tks =knapsack(k) # put upper bound on the maximum number of request to be served by the vehicle k
                    # to get nkmax we solve the per vehcile problem with knapsack as upper bound on the # of request to serve
                    print(f"############ Maximizing # of requests with knapsack bound for courier {k} ###############") 
                    _ , t1, _ , nkmax1 = buil_solve_3d(KO_l,KC_l,KO_l+KC_l,{k: list_nkmax_ks[k]},{k:20*list_nkmax_ks[k]},list_nkmax_ks[k],'#_requests') # in model we maximize the number of requests
                    print("knapsack", list_nkmax_ks[k], 'runtime KS',tks, 'Max served req',nkmax1,'time of max # req',t1)

                    # _ , t, profit, nkmax = buil_solve_3d(KO_l,KC_l,KO_l+KC_l,{k: nkmax1},{k:20*nkmax1},nkmax1,'profit') # could be an upper bound for driver's k profit
                    # tot_time_single_veh += t + t1
                    tot_time_single_veh += t1 + tks
                    
                    #tot_profit_single_veh += profit
                    list_nkmax[k] = nkmax1 # using maximizing number of requests                     
                    tot_nkmax += nkmax1 # found using max of served request problems
                    #print(f"Courier {k}: Profit {profit}, time {t}, tot_time {tot_time_single_veh}")
                    #list_maxprofit[k]= profit
                    
                print(list_nkmax,list_maxprofit,tot_nkmax )
                # solve full model with Nkmax
                output_list,t, zobj,_ = buil_solve_3d(KO,KC,K,list_nkmax,list_maxprofit,tot_nkmax,'profit') # here we bound the total # of served req. by # of requests found using decompostion per k
                print(output_list, t, t+tot_time_single_veh, file=out_file)#, m.NumContVars            
                
                print("Tot per vehicle time and profit", tot_time_single_veh, tot_profit_single_veh)
                print("All vehicles time and profit EXCL presolving per vehicle time", t, zobj)
                print("All vehicles time and profit INCL presolving per vehicle time", t+tot_time_single_veh, zobj)
                
                # solving full with Nkmax replaced by len(PK for k in K) 
                # list_lenPK = {k:len([(p,k) for p in P if (p,k) in PK]) for k in K}
                # output_list,t_full_no_limit, _,_ = buil_solve_3d(KO,KC,K,list_lenPK,{k:20*list_lenPK[k] for k in K})      
                # print(output_list, file=out_file)#, m.NumContVars          
                # print("All vehicles time and profit without Nkmax",t_full_no_limit)
                
                # ------------------------------------------------------------
                # SECOND MODEL: Extension of Furtado's Two-Index Formulation
                # ------------------------------------------------------------

                # # ------------------------------------------------------------
                # # DUPLICATE DEPOTS FOR EACH COURIER
                # # ------------------------------------------------------------

                # print("\n\n=== Building Two-Index Model (SPDP-TW-2CC) ===")

                # m2 = gp.Model("SPDP_TW_2CC")

                # m2.Params.timeLimit = TimeLim

                # K = [x + 1 for x in K]
                # KC = list(range(1, num_comm_drivers+1))
                # KO = list(range(num_comm_drivers+1, num_comm_drivers+num_occ_drivers+1))

                # # ------------------------------------------------------------
                # # 1. Duplicate depots for each courier
                # # ------------------------------------------------------------
                # O = [f"o{k}" for k in K]  # origin depot for each courier
                # F = [f"f{k}" for k in K]  # end depot for each courier
                # V2 = O + P + D + F

                # # Arc sets
                # A1_2 = [(o, j) for o in O for j in P]                       # origin depots -> pickups
                # A2_2 = [(i, j) for i in P + D for j in P + D if i != j]     # between pickups/deliveries
                # A3_2 = [(i, f) for f in F for i in D]  + [(o,f) for o in O for f in F]        # deliveries -> end depots
                # A2CC = A1_2 + A2_2 + A3_2

                # # ------------------------------------------------------------
                # # 2. Distances
                # # ------------------------------------------------------------
                # coords2 = {}
                # for req in instance["requests"]:
                #     pid = req["id"]
                #     coords2[pid] = req["pickup"]
                #     coords2[pid + num_requests] = req["dest"]

                # for k, (ox, oy) in enumerate(ok):
                #     coords2[f"o{k+1}"] = (ox, oy)
                #     coords2[f"f{k+1}"] = (ox, oy)

                # dij2_A2 = {(i,j): math.dist(coords2[i], coords2[j]) for (i,j) in A1_2+A2_2}
                # dij2_A3 = {(i,j): 0 for (i,j) in A3_2}
                # dij2 = {**dij2_A2, **dij2_A3}
                # tij2 = dij2.copy()

                # for o in O:
                #     si[o] = 0
                # for f in F:
                #     si[f] = 0

                # # pruning unfeasible links and assignments
                # z2_UB = {(pickup,driver): (1 if (li[pickup] - si[pickup] >= Ek[driver-1] and Lk[driver-1] >= ei[pickup + num_requests]) else 0) for driver in K for pickup in P} #cart_prod_pickups_destinations                                    
                # x2_UB = {(i,j): (0 if ( (ei[i] + dij2[i, j] + si[j] > li[j]) or (i in D and j==i-num_requests) ) else 1) for (i,j) in A2_2} #cart_prod_pickups_destinations
                # UBx2IPC1 = {(n,d): 0 if (d in D and d-num_requests!=n and x2_UB[d-num_requests,n]==0 ) else 1 for (n,d) in A2_2}
                # UBx2IPC2 = {(p,n): 0 if (p in P and p+num_requests!=n and x2_UB[n,p + num_requests]==0 ) else 1 for (p,n) in A2_2}

                # # Update set of Arcs based on
                # print( ((1,3) in A2_2) , ((3,1) in A2_2),len(A2_2))
                # A2_2 = [(i,j) for (i,j) in A2_2 if x2_UB[i,j]*UBx2IPC1[i,j]*UBx2IPC2[i,j]==1]             
                # A2CC = A1_2 + A2_2 + A3_2          

                # # define PDK2 set of variables y is defined on this set
                # PDK2 = [] # [(i,k) for i in V2 for k in K]
                # for i in V2:
                #     for k in K:
                #         if i in O + F:
                #             PDK2.append((i,k))
                #         elif i in P and z2_UB[i,k]==1:
                #             PDK2.append((i,k))
                #         elif i in D and z2_UB[i-num_requests,k]==1:
                #             PDK2.append((i,k))

                # # ------------------------------------------------------------
                # # 3. Variables
                # # ------------------------------------------------------------
                # x2 = m2.addVars(A2CC, vtype=GRB.BINARY, name="x")
                # #y2 = m2.addVars(V2, K, vtype=GRB.BINARY, name="y")
                # y2 = m2.addVars(PDK2, vtype=GRB.BINARY, name="y")
                # #z2 = m2.addVars(P+D, vtype=GRB.BINARY, name="z2")
                # T2 = m2.addVars(V2, vtype=GRB.CONTINUOUS, lb=0, name="T")
                # v2 = m2.addVars(V2, vtype=GRB.CONTINUOUS, lb=0, name="v")#ub=1,
                # w2 = m2.addVars(A2CC, vtype=GRB.CONTINUOUS, lb=0, name="w")
                # L2 = m2.addVars(P+D, vtype=GRB.CONTINUOUS, lb=0, name="L2") # MTZ type ub=Qk[1]
                # #L2 = m2.addVars(A2_2, vtype=GRB.CONTINUOUS, lb=0, name="L2")
                # # w2 = m2.addVars(O+P+D, K, vtype=GRB.CONTINUOUS, lb=0, name="w") #implementing w_i,k in place of w_i,j
                
                # # ------------------------------------------------------------
                # # 4. Objective (eq. 16)
                # # ------------------------------------------------------------
                # #obj2 = gp.quicksum(p[i] * z2[i] for i in P)
                # obj2 = gp.quicksum(p[i] * gp.quicksum(y2[i, k] for k in K if (i,k) in PDK2) for i in P)
                # obj2 -= gp.quicksum((wC * (Lk[k-1] - Ek[k-1])) for k in KC)
                # obj2 -= gp.quicksum((cO * gp.quicksum(y2[i, k] for i in P if (i,k) in PDK2)) for k in KO)
                # obj2 -= gp.quicksum(w2[i, j] for (i, j) in A1_2 +A2_2)
                # # obj2 -= gp.quicksum(w2[i, k] for i in O+P+D for k in K) # w_i,k is used
                # # obj2 -= 0.0001*gp.quicksum(L2[i] for i in P+D) # this is added to minimize the load in the vehicles
                # # obj2 -= 0.0001*gp.quicksum(L2[i,j] for (i,j) in A2_2) # this is added to minimize the load in the vehicles
                
                # m2.setObjective(obj2, GRB.MAXIMIZE)

                # # ------------------------------------------------------------
                # # 5. Constraints
                # # ------------------------------------------------------------
                
                # # (17) Each pickup served at most once
                # m2.addConstrs(gp.quicksum(x2[i, j] for j in V2 if (i, j) in A2CC) <= 1 for i in P)

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

                # # (21) Time propagation along arcs            
                # # m2.addConstrs(
                # #     T2[j] >= T2[i] + tij2[i, j] + si[j] - H * (1 - x2[i, j])
                # #     for (i, j) in A1_2 + A2_2  + A3_2
                # # )
                
                # # Lifted                
                # M = {(i,j): max( (li[i] if i not in O else Lk[O.index(i)])-(ei[j] if j not in F else Ek[F.index(j)]) + si[j] + tij2[i,j],0) for (i,j) in A1_2 + A2_2 + A3_2}
                # # #T2[j] >= T2[i]  + si[j] + tij2[i, j] - M[i,j] * (1- x2[i, j]) + (M[i,j] - t2[i, j] - si[j] + ei[j] - li[i]) * x2[j, i]
                # m2.addConstrs(
                #     T2[j] >= T2[i] + tij2[i, j] + si[j] - M[i,j] * (1 - x2[i, j]) 
                #     + (M[i,j] - tij2[i, j] - si[j] + ei[j] - li[i]) * (x2[j, i] if (j,i) in A2_2 else 0)
                #     for (i, j) in A2_2 
                # )
                
                # m2.addConstrs(
                #     T2[j] >= T2[i] + tij2[i, j] + si[j] - M[i,j] * (1 - x2[i, j])
                #     for (i, j) in A1_2 + A3_2
                # )
                
                # # (22) Node time windows
                # m2.addConstrs(ei[i] <= T2[i] for i in P + D)
                # m2.addConstrs(T2[i] <= li[i] for i in P + D)

                # # (23) Precedence
                # m2.addConstrs(T2[i] + tij[i,i+n] + si[i+n] <= T2[i+n] for i in P)

                # # (24–25) Courier time constraints
                # m2.addConstrs(Ek[k-1] <= T2[f"o{k}"] for k in K)
                # m2.addConstrs(T2[f"f{k}"] <= Lk[k-1] for k in K)

                # # (26–27) v-index propagation
                # m2.addConstrs(v2[j] >= v2[i] -  len(K) *(1 - x2[i, j]) for (i, j) in A2_2+A3_2) #len(K) *
                # m2.addConstrs(v2[j] <= v2[i] +  len(K) *(1 - x2[i, j]) for (i, j) in A2_2+A3_2) #len(K) *

                # # (28–29) v-index from origin depots
                # m2.addConstrs(v2[j] >= k * x2[f"o{k}",j] for k in K for j in P+[f"d{k}"] if (f"o{k}", j) in A2CC)
                # m2.addConstrs(v2[j] <= k * x2[f"o{k}",j] + len(K) * (1 - x2[f"o{k}", j]) for k in K for j in P+[f"d{k}"] if (f"o{k}", j) in A2CC)

                # # (30) Pickup and delivery same courier
                # m2.addConstrs(v2[i] == v2[i + n] for i in P)

                # # (31) Linking v with y
                # m2.addConstrs(v2[j] == gp.quicksum(k * y2[j, k] for k in K if (j,k) in PDK2) for j in V2)

                # # (32) Linking x with y
                # m2.addConstrs(gp.quicksum(y2[i, k] for k in K if (i,k) in PDK2) == gp.quicksum(x2[i,j] for j in V2 if (i,j) in A2CC) for i in O+P+D)
                # m2.addConstrs( y2[i, k] == y2[i+num_requests,k] for i in P for k in K if (i,k) in PDK2)

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
                # #y2[i, k] +
                # #m2.addConstrs( (w2[i, j] >= delta_cost(i, j, k) * dij2[i, j] -1000 * (3 - y2[i, k] - y2[j, k] - x2[i,j])) for k in K for (i, j) in A1_2+A2_2 )
                # m2.addConstrs( (w2[i, j] >= delta_cost(i, j, k) * dij2[i, j]*(-1 + y2[i, k] + x2[i,j])) for k in K for (i, j) in A1_2+A2_2 if (i,k) in PDK2 and (j,k) in PDK2) 

                # # Load constraint MTZ type
                # m2.addConstrs(L2[j] >= L2[i] + dem[j] - (Qk[k-1]-min(0,dem[j])) * (3 - x2[i, j]-y2[i,k]-y2[j,k]) for (i,j) in A2_2 for k in K if (i,k) in PDK2 and (j,k) in PDK2)
                # #m2.addConstrs(L2[j] <= L2[i] + dem[j] + (Qk[1]-min(0,dem[j])) * (1 - x2[i, j]) for (i,j) in A2_2)
                # m2.addConstrs(L2[j] >= dem[j] - (Qk[k-1]-dem[j]) * (3 - x2[i, j]-y2[i,k]-y2[j,k]) for (i,j) in A1_2 for k in K if (i,k) in PDK2 and (j,k) in PDK2) #i,j in A1_2 means i in depot and j in pickup
                # m2.addConstrs(L2[i]<=gp.quicksum( (Qk[k-1] )*y2[i,k] for k in K if (i,k) in PDK2) for i in P)
                # m2.addConstrs(L2[i]<=gp.quicksum( (Qk[k-1] + dem[i] )*y2[i-num_requests,k] for k in K if (i-num_requests,k) in PDK2) for i in D) # i in D dem is negative
                
                # #m2.addConstrs(L2[j] >= dem[j] * x2[i, j] for (i,j) in A1_2) #i,j in A1_2 means i in depot and j in pickup
                # #m2.addConstrs(L2[j] <= dem[j] + (Qk[1]-min(0,dem[j])) * (1 - x2[i, j]) for (i,j) in A1_2)
                # #m2.addConstrs(L2[j] >= L2[i] - Qk[1] * (1 - x2[i, j]) for (i,j) in A3_2)
                # # Load constraint flow type
                # # # \sum_{j} y_{ij}^k \;-\; \sum_{h} y_{hi}^k &= \text{dem}[i] \sum_{h} x_{hi}^k, && \forall i, \forall k 
                # # #dem[i]*gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2)
                # # #dem[i]*gp.quicksum(x2[i,j] for j in P+D+F if (i,j) in A2CC)`
                # # m2.addConstrs (gp.quicksum(L2[i,j] for j in P+D if (i,j) in A2_2) - gp.quicksum(L2[j,i] for j in P+D if (j,i) in A2_2) ==  dem[i]*gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2) for i in P+D)            
                # # #0 &\le y_{ij}^k \le Q_k \, x_{ij}^k, && \forall (i,j), \forall k
                # # m2.addConstrs( L2[i,j] <= (Qk[k-1]+ min(dem[i],0))*(3-x2[i,j]-y2[i,k]) for (i,j) in A2_2 for k in K if (i,k) in PDK2 and (j,k) in PDK2)  #and (j,k) in PDK2          


                # # -----------------------------------------------------------------------
                # ################   Valid inequalities #########################
                # # -----------------------------------------------------------------------
                # if vineq == 'Arc' or vineq == 'ALL':                    
                #     ############### x2[i,j]+x2[j,i]<=1 this is called Arc selection Valid inequalities in GTSP ######################
                #     m2.addConstrs((x2[i,j] +  x2[j,i] <= 1 for (i,j) in A2_2 if (j,i) in A2_2), name="arc selection") #211 349 65%
                #     #m2.addConstrs((x2[i,j] +  x2[j,i] <= (3-y2[i,k] - y2[j,k]) for (i,j) in A2_2 for k in K if (j,i) in A2_2 and (i,k) in PDK2 and (j,k) in PDK2), name="arc selection") #218 333 52%
                #     # m2.addConstrs((x2[i,j] +  x2[j,i] <= (3 - gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2) 
                #     #                                       - gp.quicksum(y2[j,k] for k in K if (j,k) in PDK2)) 
                #     #                for (i,j) in A2_2 if (j,i) in A2_2), name="arc selection") #218 333 52%
                
                # elif vineq=='SOP' or vineq == 'ALL':
                #     # ############# Sequential ordering problem inequalities SOP GTSP ####################
                #     for i in P+D:
                #         #successors = [d]
                #         successors = []
                #         for n1 in P+D:
                #             # is is predesessor of j if i ≺ j if Ei + Ti j ≤ L j and E j + Tji > Li.
                #             if (i,n1) in A2_2 and (ei[i] + si[i] + tij2[i, n1] <= li[n1]) and (ei[n1] + si[n1] + tij2[n1,i] > li[i]):
                #                 successors.append(n1)                        
                                
                #         m2.addConstrs((gp.quicksum(x2[n,i] for n in O + P + D if n not in successors and n!=i and (n,i) in A1_2 + A2_2) 
                #                     >= y2[i,k] for k in K if (i,k) in PDK2), name="sop valid inequalities")
                #         #gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2)

                # #           
                # elif vineq == '2PC' or vineq == 'ALL':
                #     # 2PC constraint 
                #     for i in P+D: #pickups
                #         for j in P+D:                                    
                #             if (i,j) not in A2_2 and (j,i) not in A2_2 :# i and j cannot be on same route #and i in pairs.keys() and j in pairs.keys()                            
                #                 m2.addConstr( gp.quicksum(x2[i,n] for n in P + D + F if n!=i and n!=j and (i,n) in A2_2+A3_2) 
                #                             == gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2), name="2PC_i")#gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2) #what comes OUT In into set S={i,j}    
                #                 m2.addConstr( gp.quicksum(x2[j,n] for n in P + D + F if n!=i and n!=j and (j,n) in A2_2+A3_2) 
                #                             == gp.quicksum(y2[j,k] for k in K if (j,k) in PDK2), name="2PC_j")#gp.quicksum(y2[j,k] for k in K if (j,k) in PDK2) #what comes OUT In into set S={i,j}
                # elif vineq== 'RC' or vineq == 'ALL':
                #     # viol_pick_lst contains list of pickups that cannot be served in sequence due to load constraint
                #     for key in viol_pick_lst.keys(): # key is vehicle id                 
                #         for lst in viol_pick_lst[key]: # lst is set on node with total load larger than vehicle cap.                    
                #             if all((i,key) in PDK2 for i in lst):                            
                #                 # m2.addConstr(x[i,j,key]+x[j,k,key] 
                #                 #             <= 1 , name='load valid inequality')
                #                 # m2.addConstr(x[k,j,key]+x[j,i,key] 
                #                 #             <= 1 , name='load valid inequality')
                                
                #                 m2.addConstr(gp.quicksum(x2[i,l] for l in P+D+F for i in lst if l !=i and (i,l) in A2_2 + A3_2)
                #                             >= gp.quicksum(y2[i,key] for i in lst) -  len(lst) + 2)
                                
                #                 m2.addConstr(gp.quicksum(x2[i+num_requests,l] for l in P+D+F for i in lst
                #                                         if l != i+num_requests and (i+num_requests,l) in A2_2 + A3_2) 
                #                             >= gp.quicksum(y2[i,key] for i  in lst) - len(lst) + 2 )                                            
                
                # # ------------------------------------------------------------
                # # 6. Solve
                # # ------------------------------------------------------------
                # m2.write("model.lp")
                # print("\nSolving Extended Model...")
                # m2.optimize()

                # if m2.status == GRB.OPTIMAL:
                #     print("\n=== Extended Model Objective ===")
                #     print(round(m2.objVal, 4))

                #     print("\n=== Courier Schedules ===")

                #     for k in K:
                #         start_time = T2[f"o{k}"].X
                #         end_time   = T2[f"f{k}"].X

                #         print(f"\nCourier {k}:")
                #         print(f"  Start time: {start_time:.2f}  (Earliest allowed {Ek[k-1]:.2f})")
                #         print(f"  End time:   {end_time:.2f}  (Latest allowed {Lk[k-1]:.2f})")

                #         # list served pickups
                #         served = [i for i in P if (i,k) in PDK2 and y2[i,k].X > 0.5]
                #         if not served:
                #             print("  No requests assigned")
                #             continue

                #         print("  Served requests:")
                #         for i in served:
                #             pickup_time   = T2[i].X
                #             delivery_time = T2[i+n].X
                #             print(f"    Request {i}: "
                #                     f"Pickup at {pickup_time:.2f} (TW [{ei[i]:.2f},{li[i]:.2f}]), "
                #                     f"Delivery at {delivery_time:.2f} (TW [{ei[i+n]:.2f},{li[i+n]:.2f}])")

                #         # show route arcs chosen
                #         route = []
                #         current = f"o{k}"
                #         while True:
                #             next_nodes = [j for j in V2 if (current,j) in A2CC and x2[current,j].X > 0.5]
                            
                #             if not next_nodes:
                #                 break
                #             nxt = next_nodes[0]
                            
                #             #route.append( (nxt,f"L2[{current,next_nodes[0]}]",(L2[current,next_nodes[0]].X if (current not in O and next_nodes[0] not in F) else 0)) )
                #             route.append( (nxt,f"L2[{nxt}]",(L2[nxt].X if (nxt not in F) else 0)) )
                            
                #             if nxt == f"f{k}":
                #                 break
                #             current = nxt

                #         print(f"  Route: 0 -> " + " -> ".join(str(node) for node in route))                    

                #     # print("X")
                #     # for (i,j) in A2CC:
                #     #     if x2[i,j].X > 0.2:
                #     #         print(i, j, x2[i,j].X)

                #     # print("Y")
                #     # for i in P:
                #     #     for k in K:
                #     #         if y2[i,k].X > 0.5:
                #     #             print(i, k, y2[i,k].X)

                #     # print("V")
                #     # for i in P+D:
                #     #     print(i, v2[i])

                #     # print("T2")
                #     # for i in V2:
                #     #     print(i, T2[i])

                #     # # Loop through all keys in w2
                #     # for (i, j), value in w2.items():
                #     #     expected = dij2.get((i, j))
                #     #     if expected is None:
                #     #         print(f"⚠️ Missing dij2[{i},{j}]")
                #     #         continue

                #     #     expected_val = expected * 0.2
                #     #     is_positive = value.X > 0
                #     #     if is_positive:
                #     #         is_same = abs(value.X - expected_val) < 1e-9  # float-safe comparison

                #     #         print(f"w2[{i},{j}] = {value.X:.3f} | positive={is_positive} | "
                #     #             f"expected={expected_val:.3f} | match={is_same}")

                #     # print(gp.quicksum(p[i] * gp.quicksum(y2[i, k] for k in K) for i in P).getValue())
                #     # print(gp.quicksum((wC * (Lk[k-1] - Ek[k-1])) for k in KC).getValue())
                #     # print(gp.quicksum((cO * gp.quicksum(y2[i, k] for i in P)) for k in KO).getValue())
                #     # print(gp.quicksum(w2[i, j] for (i, j) in A2CC).getValue())

                # elif m2.status == GRB.INFEASIBLE:
                #     m2.computeIIS()
                #     m2.write("infeasible.ilp")
                    
                # if m2.SolCount > 0:
                #     GAP2 = 100*(1-m2.getObjective().getValue()/m2.ObjBound)
                #     print(vineq+"_2D_FurtadoExtended",nbr_req, nbr_cdr, nbr_odr, m2.Runtime, m2.getObjective().getValue(), GAP2, m2.NumConstrs, m2.NumVars, m2.NumBinVars, m2.NumIntVars,  file=out_file)#m2.NumContVars,
                
                out_file.close()
                
        #increasing nbr of requests
        nbr_req += 10 # if nbr_req < 50 else 10