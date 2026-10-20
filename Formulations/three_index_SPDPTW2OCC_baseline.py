import math

# import os
# os.add_dll_directory(r"C://gurobi1300/win64/bin")

import gurobipy as gp
from gurobipy import GRB
from instance_loader import *
from itertools import combinations
from typing import Dict, List, Tuple
import networkx as nx


def compute_reachability(link_dict):
    """
    link_dict: dictionary {(i,j):0/1}
    Returns: dictionary {(i,j): True/False} meaning j reachable from i
            (excluding direct edges)
    """
    G = nx.DiGraph()

    # Extract all nodes
    nodes = set()
    for (i, j) in link_dict:
        nodes.add(i)
        nodes.add(j)
    nodes = sorted(nodes)

    # Add edges for values=1
    for (i, j), v in link_dict.items():
        if v == 1:
            G.add_edge(i, j)

    # Compute reachability for every node using BFS
    reachability = {}
    for i in nodes:
        reachable = nx.descendants(G, i)  # all nodes reachable from i
        for j in nodes:
            if i == j:
                continue
            if link_dict.get((i,j), 0) == 0:
                reachability[(i,j)] = ((j in reachable),reachable)

    return reachability

def knapsack(k):# cannot be used as we we have a pickup and delivery so for exmaple for capacity used to bounding the maximum number of requests that could be assigned to vehicle k
    # k: index of vehicle
    # function return the maximum number of requests that can assigned to k
    # PK = [(p,k) for p in P for k in K if z_UB[p,k]==1]#if z_UB[p,k]==1

    Setk = [p for (p,d) in PK if d==k] # retreiving the set of pickups that could be assigned to driver k
    ks = gp.Model("knapsack")
    z = ks.addVars(Setk, vtype=GRB.BINARY, name="z")

    obj = gp.quicksum(z[p] for p in Setk)
    ks.setObjective(obj, GRB.MAXIMIZE)

    #ks.addConstr( gp.quicksum(dem[p]*z[p] for p in Setk)<=Qk[k] )
    for p1 in Setk:
        for p2 in Setk:
            # incompatible for pickups or pickup and destination or destiantions
            # NNNNBBBB I found that even if we cannot visit i->j (x_ind(i,j)==0) directly we can go through other nodes to go ti j
            if p1!=p2 and (x_ind[p1,p2] + x_ind[p2,p1]==0 
                          or x_ind[p1,p2+num_requests] + x_ind[p2+num_requests,p1]==0 
                          or x_ind[p1+num_requests,p2] + x_ind[p2,p1+num_requests]==0 
                          or x_ind[p1+num_requests,p2+num_requests] + x_ind[p2+num_requests,p1+num_requests]==0
                          ):#nodes p1 and p2 are incompatible
                print(f"Incompatibility nodes {p1} {p2} ")
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
    print(filtered_Setk,maxSetk_size,len(Setk))
    
    # for subset in combinations(Setk,maxSetk_size):
    #     if sum([dem[s] for s in subset])<=Qk[k]:
    #         print(subset)

    return ks.getObjective().getValue(), ks.Runtime

def build_solve_3d(KO,KC,P,D,A1,A2,A3,Nkmax_loc,maxobj,ObjType):
    # KO set of occasionl
    # KC set of committed
    # k SET OF DRIVERS 
    # Nkmax_loc dictionary {k:NN} on number of orders to be assigned to driver k
    # maxobj dictionary {k:maxprofitk} maxprofitk for vehicle k when solved individually
    # tot_req_loc bound on the max number of vehicles to be served mainly for the full problem
    # ObjType can be 'profit' or '#_requests'
    
    K = KC + KO
    A = A1 + A2 + A3
    #V = [0] + P + D + [2*n+1]

    m = gp.Model("SPDPTW2OCC")
    m.Params.timeLimit = TimeLim
    
    # local set
    PK_l = [(p,k) for p in P for k in K if (p,k) in PK] # PK is global set and PK_l is local set (p,k) for driver k restricted to feasible assignment
    P_l = {k: [p for p in P if z_UB[(p,k)]==1] for k in K} # set of pickups feasible to assingn to driver k
    DK_l = [(p+num_requests,k) for (p,k) in PK_l] # DK is global set and DK_l is local for driver k restricted to feasible assignement
    D_l = {k: [p+n for p in P_l[k]] for k in K} # set of destinations feasible to add to  assingn to driver k
    PKDK = PK_l + DK_l

    # local sets
    A1k = [(0,j,k) for (j,k) in PK_l]  # local set from depot to feasible pickup for driver k
    A2k = [(i,j,k) for (i,j) in A2 for k in K if aijk[(i,j,k)]==1] # local set from pickups/destinatins to pickups/destinatins depot for driver k
    A3k = [(i,2*n+1,k) for i,k in DK_l ] + [(0,2*n+1,k) for k in K] # local set from destinatins to final depot for driver k
    Ak  = A1k + A2k + A3k

    print('Savings in variables xijk on A2k',len(aijk.values())-sum(aijk.values())) #
    # no need for fixation as x is defined if i,j,k is feasible
    #m.addConstrs ( x[i,j,k] == 0 for (i,j) in A2 for k in K if aijk[i,j,k] == 0) # swithcing off this constraint gave better results for NONE

    x = m.addVars(Ak , vtype=GRB.BINARY, name="x")
    L = m.addVars(A2, vtype=GRB.CONTINUOUS, name="L") # here A2 is global but passed for driver k's only.
    #y = m.addVars(P+D,K, vtype=GRB.BINARY, name="y") #extending the definitin of y
    y = m.addVars(PKDK, vtype=GRB.BINARY, name="y") #extending the definitin of y

    #T = m.addVars(P+D+[0,2*n+1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")
    Tk_set = [(0,k) for k in K] + PKDK + [(2*n+1,k) for k in K]
    T = m.addVars(Tk_set, vtype=GRB.CONTINUOUS, lb=0, name="T")    

    # Objective (1)
    if ObjType=='profit':
        obj = gp.quicksum(rev[i] * y[i,k] for i,k in PK_l )

        obj -= gp.quicksum(
            (wC*(Lk_org[k]-Ek_org[k]) + deltaC*(
                gp.quicksum(dijk[0,j,k]*x[0,j,k] for (i,j,k1) in A1k if k1==k) +#(i,j) in A1 if (0,j,k) in Ak
                gp.quicksum(dij[i,j]*x[i,j,k] for (i,j,k1) in A2k if k1==k)  # (i,j) in A2 if (i,j,k) in Ak
            )) for k in KC
        )

        obj -= gp.quicksum(
            (cO * gp.quicksum(y[i,k] for i in P if (i,k) in PK_l) +
            deltaO * gp.quicksum(dij[i,j]*x[i,j,k] for (i,j) in A2 if aijk[(i,j,k)]==1))#(i,j) in A2 if i in P and j in D and (i,j,k) in Ak
            for k in KO
        )
        # m.addConstr(obj <= Lagragian_ub[(nbr_req,nbr_cdr,nbr_odr)],name='Lagrangian upper bound')
    elif ObjType=='#_requests':
        obj = gp.quicksum(y[i,k] for i,k in PK_l)

    else: 
        print("Wrong objective specified")

    m.setObjective(obj, GRB.MAXIMIZE)

    # (2) Each request served at most once
    m.addConstrs(gp.quicksum(x[i,j,k] for (l,j,k) in A2k if l==i ) <= 1 for i in P) # for k in K for j in V if (i,j) in A and (i,j,k) in Ak
    #m.addConstrs(gp.quicksum(y[i,k] for k in K if (i,k) in PK_l) <= 1 for i in P)
    
    # (3) Flow conservation
    m.addConstrs(
        gp.quicksum(x[j,i,k] for j in V if (j,i,k) in  Ak) - #j,i in A here A1k + A2k
        gp.quicksum(x[i,j,k] for j in V if (i,j,k) in Ak) == 0 # i,j in A A2k + A3k
        for (i,k) in PKDK
    )

    # (4) Pickup-delivery flow
    m.addConstrs(
        gp.quicksum(x[i,j,k] for j in V if (i,j,k) in A2k) - #(i,j) in A
        gp.quicksum(x[j,i+n,k] for j in V if (j,i+n,k) in A2k + A3k) == 0
        for (i,k) in PK_l
    )

    # (5) Depart depot
    m.addConstrs(gp.quicksum(x[0,j,k] for (j,k1) in PK_l if k1 == k) + x[0,2*n+1,k] == 1 for k in K) #V if (0, j, k) in A #j in P + [2*n+1] if aijk[(0,j,k)]==1

    # (6) Return depot
    m.addConstrs(gp.quicksum(x[j,2*n+1,k] for j,k1 in DK_l if k1==k ) + x[0,2*n+1,k] == 1 for k in K)# V if (j,2*n+1) in A

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
    m.addConstrs( y[i,k] == gp.quicksum(x[i,l,k] for l in P_l[k]+D_l[k]+[2*n+1] if (i,l) in A2+A3 and aijk[(i,l,k)]==1) for (i,k) in PKDK)#  for i in P+D for k in K if (i,k) in PKDK
    #m.addConstrs( y[i,k] == y[i+n,k] for i in P for k in K )
    #m.addConstrs( y[i,k] == 0 for i in P for k in K if z_UB[i,k]==0 )
    
    for (i,j,k) in A2k:                
        if (j,i) in A2 and aijk[(j,i,k)]==1 : #(j,i) in A2 and (j,i,k) in Ak                       
            m.addConstr(                            
                T[i,k] - T[j,k] + (tij[i, j] + si[j]) * x[i, j , k] + svMTZ[i,j] * x[j, i, k] 
                <= li[i]*y[i,k] #1
                - ei[j]*y[j,k] 
                - (li[i] - ei[j]) * (x[i, j, k] + x[j, i, k]  )                             
            )
        else:
            m.addConstr(                            
                T[i,k] - T[j,k] + (tij[i, j] + si[j]) * x[i, j , k]  
                <= li[i]*y[i,k]  #2
                - ei[j]*y[j,k] 
                - (li[i] - ei[j]) * (x[i, j, k] )                             
            )

    # for (i,j) in A2:
    #     for k in K:
    #         if (i,j,k) in Ak :#(i,k) in PKDK and (j,k) in PKDK and (i,j,k) in Ak :
    #             if (j,i,k) in A2k : #(j,i) in A2 and (j,i,k) in Ak                       
    #                 m.addConstr(                            
    #                     T[i,k] - T[j,k] + (tij[i, j] + si[j]) * x[i, j , k] + svMTZ[i,j] * x[j, i, k] 
    #                     <= li[i]*y[i,k] #1
    #                     - ei[j]*y[j,k] 
    #                     - (li[i] - ei[j]) * (x[i, j, k] + x[j, i, k]  )                             
    #                 )
    #             else:
    #                 m.addConstr(                            
    #                     T[i,k] - T[j,k] + (tij[i, j] + si[j]) * x[i, j , k]  
    #                     <= li[i]*y[i,k]  #2
    #                     - ei[j]*y[j,k] 
    #                     - (li[i] - ei[j]) * (x[i, j, k] )                             
    #                 )
                
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
                    T[i,k] - T[2*n+1,k] + (tij[i, 2*n+1] + si[2*n+1]) * x[i, 2*n+1 , k]  
                    <= li[i]*y[i,k] #3
                    - Ek[k]*gp.quicksum(x[0,ii,k] for ii in P_l[k]) #in P_l if (ii,k) in PK_l
                    - (li[i] - Ek[k]) * (x[i, 2*n+1, k] )  
                    for  (i,k) in DK_l if i!=0 #(i,j,k) in A3 for k in K if i!=0 if (i,k) in PKDK #(i,j,k) in A3k
                )
    
    #A1 = [(0,j) for j in P]
    m.addConstrs(                
                    T[0,k] - T[j,k] + (tijk[0, j,k] + si[j]) * x[0, j , k] 
                    <= Lk[k]*gp.quicksum(x[0,ii,k] for ii in P_l[k]) # if (ii,k) in PK_l
                    - ei[j]*y[j,k] 
                    - (Lk[k] - ei[j]) * (x[0, j, k] ) 
                    for  (j,k) in PK_l #(i,j) in A1 for k in K if (j,k) in PKDK (i,j,k) in A1k
    )
                
    # (8a) Time windows 1
    m.addConstrs((ei[i]*y[i,k] <= T[i,k]) for (i,k) in PKDK)

    # (8b) Time windows 2
    m.addConstrs( (T[i,k] <= li[i]*y[i,k]) for (i,k) in PKDK )

    # (9) Pickup before delivery
    m.addConstrs(T[i,k] + (tij[i,i+n] + si[i+n])*y[i,k] <= T[i+n,k] 
                for (i,k) in PK_l) # i in P for k in K 
    # (11) Earliest departure
    m.addConstrs(Ek[k]*gp.quicksum(x[0,i,k] for i in P_l[k]) <= T[0,k] for k in K) #(i,k)]==1 in PK_l

    # (12) Latest return
    m.addConstrs(T[2*n+1,k] <= Lk[k]*gp.quicksum(x[0,i,k] for i in P_l[k]) for k in K)#for i in P if (i,k) in PK_l #(i,k1) in PK_l if k1==k ,  if z_UB[i,k]==1

    # Load constraint flow type
    # Load L[i,j] on arc (i,j), shared by all couriers (each arc is used by at most one courier):
    # \sum_{j} L_{ij} \;-\; \sum_{h} L_{hi} &= \text{dem}[i] \sum_{k} \sum_{j} x_{ij}^k, && \forall i
    m.addConstrs (gp.quicksum(L[i,j] for j in P+D if (i,j) in A2) - gp.quicksum(L[j,i] for j in P+D if (j,i) in A2)
                ==  dem[i]*gp.quicksum(x[i,j,k] for k in K for j in P_l[k]+D_l[k]+[2*n+1] if (i,j) in A2+A3 and aijk[(i,j,k)]==1) for i in P+D)            #(i,j) in A2+A3 and
    #0 &\le y_{ij}^k \le Q_k \, x_{ij}^k, && \forall (i,j), \forall k
    m.addConstrs( L[i,j] <= gp.quicksum((Qk[k]+ min(dem[i],0))*x[i,j,k] for k in K if aijk[(i,j,k)]==1) for (i,j) in A2)
    m.addConstrs( L[i,j] >= gp.quicksum((max(dem[i],0))*x[i,j,k] for k in K if aijk[(i,j,k)]==1) for (i,j) in A2 ) # lower bound on L[i,j]

    # -----------------------------------------------------------------------
    ################   Valid inequalities #########################
    # -----------------------------------------------------------------------

    vineq_k = 'ALL' if len(K)==1 else 'Wrong' # adding all cuts for the per vehicle decomposed problem

    if vineq == 'Arc' or vineq == 'ALL' or vineq_k == 'ALL' or 'Arc' in all_except_vineq:
        ############### x2[i,j]+x2[j,i]<=1 this is called Arc selection Valid inequalities in GTSP ######################
        m.addConstrs((x[i,j,k] +  x[j,i,k] <= y[i,k] for (i,j) in A2 for k in K if (j,i) in A2 and aijk[i,j,k]==1 and aijk[j,i,k]==1), name="arc selection") #211 349 65%
        m.addConstrs((x[i,j,k] +  x[j,i,k] <= y[j,k] for (i,j) in A2 for k in K if (j,i) in A2 and aijk[i,j,k]==1 and aijk[j,i,k]==1), name="arc selection") #211 349 65%
        #m.addConstrs( (x[i,j,k] +  x[j,i,k] <= (3 - y[i,k] - y[j,k]) for (i,j) in A2 for k in K if (j,i) in A2), name="arc selection") #218 333 52%
        # m2.addConstrs((x2[i,j] +  x2[j,i] <= (3 - gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2) 
        #                                       - gp.quicksum(y2[j,k] for k in K if (j,k) in PDK2)) 
        #                for (i,j) in A2_2 if (j,i) in A2_2), name="arc selection") #218 333 52%
    if vineq=='SOP' or vineq == 'ALL' or vineq_k == 'ALL' or 'SOP' in all_except_vineq:
        # SOP valid constraints
        for i in P+D:
            #d = p+num_pickups
            #successors = [d]
            successors = [] # set of nodes that comes after i
            for n1 in P+D:
                # is is predesessor of j if i ≺ j if Ei + Ti j ≤ L j and E j + Tji > Li.
                if (i,n1) in A2 and x_UB[i,n1]==1 and x_UB[n1,i]==0: #(ei[i] + si[i] + tij[i, n1] <= li[n1]) and (ei[n1] + si[n1] + tij[n1,i] > li[i])
                        successors.append(n1)                        
            #m.addConstr( quicksum(x[i,p] for i in all_points_set if i not in successors and i!=p) == z1d[p], name="sop valid inequalities" )
            if i in P:
                m.addConstr((gp.quicksum(x[n,i,k] 
                    for n in [0] + P + D for k in K if n not in successors and n!=i and (n,i,k) in x) # only arcs that exist in this model
                    >= gp.quicksum( y[i,k] for k in K if z_UB[i,k]==1) ), name="sop valid inequalities")
            else:
                m.addConstr((gp.quicksum(x[n,i,k] 
                    for n in P + D for k in K if n not in successors and n!=i and (n,i,k) in x) # only arcs that exist in this model
                    >= gp.quicksum( y[i,k] for k in K if z_UB[i-num_requests,k]==1) ), name="sop valid inequalities")
            #gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2)

    if vineq=='SOP-k' or vineq == 'ALL' or vineq_k == 'ALL' or 'SOP' in all_except_vineq:
        # SOP valid constraints
        for i in P+D:
            #d = p+num_pickups
            #successors = [d]
            successors = [] # set of nodes that comes after i
            for n1 in P+D:
                # is is predesessor of j if i ≺ j if Ei + Ti j ≤ L j and E j + Tji > Li.
                if (i,n1) in A2 and x_UB[i,n1]==1 and x_UB[n1,i]==0: #(ei[i] + si[i] + tij[i, n1] <= li[n1]) and (ei[n1] + si[n1] + tij[n1,i] > li[i])
                        successors.append(n1)                        
            #m.addConstr( quicksum(x[i,p] for i in all_points_set if i not in successors and i!=p) == z1d[p], name="sop valid inequalities" )
            if i in P:
                m.addConstrs((gp.quicksum(x[n,i,k] 
                    for n in [0] + P + D for k in K if n not in successors and n!=i and (n,i) in A1 + A2 and aijk[n,i,k]==1) #and (n,i) in A1 + A2
                    >= y[i,k] for k in K if z_UB[i,k]==1 ), name="sop valid inequalities")
            else:
                m.addConstrs((gp.quicksum(x[n,i,k] 
                    for n in P + D for k in K if n not in successors and n!=i and (n,i) in A2 and  aijk[n,i,k]==1) #(n,i) in A2 and
                    >= y[i,k] for k in K if z_UB[i-num_requests,k]==1), name="sop valid inequalities")
            #gp.quicksum(y2[i,k] for k in K if (i,k) in PDK2)

    if vineq=='2PC' or vineq == 'ALL' or vineq_k == 'ALL' or '2PC' in all_except_vineq: 
        # 2PC Valid inequalities
        for i in P+D:
            for j in P+D:
                if i!=j and x_ind[i,j]==0 and x_ind[j,i]==0:# (i,j) not in A2 and (j,i) not in A2: # means cannot be together no need to check fij and fji       
                    m.addConstr( (gp.quicksum(x[i,n,k] for n in P + D + [2*n+1] for k in K 
                                            if (i,n) in A2+A3 and n!=i and n!=j and aijk[i,n,k]==1) 
                                >= gp.quicksum(y[i,k] for k in K 
                                if ( (i if i in P else i-num_requests),k) in PK)), name="2PC_i") #what comes OUT In into set S={i,j}    
                    m.addConstr( (gp.quicksum(x[j,n,k] for n in P + D + [2*n+1] for k in K if (j,n) in A2+A3 and n!=i and n!=j and aijk[j,n,k]==1) 
                                >= gp.quicksum(y[j,k] for k in K if ( (j if j in P else j-num_requests),k) in PK)), name="2PC_j") #what comes OUT In into set S={i,j}
    
    if vineq=='2PC-k>' or vineq == 'ALL' or vineq_k == 'ALL' or '2PC' in all_except_vineq:
        # 2PC Valid inequalities
        for i in P+D:
            for j in P+D:
                if (i,j) not in A2 and (j,i) not in A2: # means cannot be together no need to check fij and fji       
                # successorsi = [] # set of nodes that comes after i
                # for n1 in P+D:
                #     # is is predesessor of j if i ≺ j if Ei + Ti j ≤ L j and E j + Tji > Li.
                #     if (i,n1) in A2 and x_UB[i,n1]==1 and x_UB[n1,i]==0: #(ei[i] + si[i] + tij[i, n1] <= li[n1]) and (ei[n1] + si[n1] + tij[n1,i] > li[i])
                #             successors.append(n1)
                    m.addConstrs( (gp.quicksum(x[i,n,k] for n in P + D + [2*n+1] 
                                            if (i,n) in A2+A3 and n!=i and n!=j and aijk[i,n,k]==1) 
                                >= y[i,k] for k in K 
                                if ( (i if i in P else i-num_requests),k) in PK), name="2PC_i") #what comes OUT In into set S={i,j}    
                    m.addConstrs( (gp.quicksum(x[j,n,k] for n in P + D + [2*n+1] if (j,n) in A2+A3 and n!=i and n!=j and aijk[j,n,k]==1) 
                                >= y[j,k] for k in K if ( (j if j in P else j-num_requests),k) in PK), name="2PC_j") #what comes OUT In into set S={i,j}

    if vineq=='RC' or vineq == 'ALL' or vineq_k == 'ALL' or 'RC' in all_except_vineq:
        # viol_pick_lst contains list of pickups that cannot be served in sequence due to load constraint
        # viol_pick_lst is only built when RC cuts are requested for the full model (it is costly),
        # so the per-courier models (vineq_k == 'ALL') skip RC cuts otherwise
        for key in globals().get('viol_pick_lst', {}).keys(): # key is vehicle id                 
            for lst in viol_pick_lst[key]: # lst is set of nodes with total load larger than vehicle cap.                    
                if all((i,key) in PK for i in lst):                            
                    # m.addConstr(x[i,j,key]+x[j,k,key] 
                    #             <= 1 , name='load valid inequality')
                    # m.addConstr(x[k,j,key]+x[j,i,key] 
                    #             <= 1 , name='load valid inequality')
                    
                    m.addConstr(gp.quicksum(x[i,l,key] for l in P+D+[2*n+1] for i in lst if l !=i and l not in lst and (i,l) in A2+A3 and aijk[i,l,key]==1)
                                >= gp.quicksum(y[i,key] for i in lst) - len(lst) + 2 )
                    
                    m.addConstr(gp.quicksum(x[i+num_requests,l,key] for l in P+D+[2*n+1] for i in lst
                                            if l != i+num_requests and l not in lst and (i+num_requests,l) in A2+A3 and aijk[i+num_requests,l,key]==1) 
                                >= gp.quicksum(y[i,key] for i  in lst) - len(lst) + 2 )
    
    if vineq=='Incom' or vineq == 'ALL' or vineq_k == 'ALL' or 'Incom' in all_except_vineq:
        #I found out even if x_ind[i,j]==0 we can go through a node k i->k->j so we cannot say that y[p1,k]+y[p2,k]<=1
        for k in K:
            Setk = [p for (p,nn) in PK if nn==k] # retreiving the set of pickups that could be assigned to driver k
            for idx, p1 in enumerate(Setk[:-1]):
                for p2 in Setk[idx+1:]:
                    # incompatible 2 requests if pickups or destinations are incompatible, or pickup or destiantion are incompatible
                    if (x_UB[p1,p2]==0 and x_UB[p2,p1]==0 
                        or x_UB[p1,p2+num_requests] + x_UB[p2+num_requests,p1]==0 
                        or x_UB[p1+num_requests,p2] + x_UB[p2,p1+num_requests]==0 
                        or x_UB[p1+num_requests,p2+num_requests] + x_UB[p2+num_requests,p1+num_requests]==0
                          ):#nodes p1 and p2 are incompatible
                        print(f"Innnnnnnnnnnnnnnnnnnncommmmmmmmmmmmmmmmmmmmmmmmmmmmmpatibility requests {p1} {p2} {k}")
                        m.addConstr(y[p1,k]+y[p2,k]<=1)  #drivers incompatibility
                        m.addConstr(y[p1+num_requests,k]+y[p2+num_requests,k]<=1)  #drivers incompatibility y's defined over P and D
                        m.addConstr(y[p1,k]+y[p2+num_requests,k]<=1)  #drivers incompatibility
                        m.addConstr(y[p1+num_requests,k]+y[p2,k]<=1)  #drivers incompatibility
                        #print("incompatibility")

        for idx, p1 in enumerate(P):
            for p2 in P[idx+1:]:
                # incompatible 2 requests if pickups or destinations are incompatible, or pickup or destiantion are incompatible
                if (x_UB[p1,p2]==0 and x_UB[p2,p1]==0 
                    or x_UB[p1,p2+num_requests] + x_UB[p2+num_requests,p1]==0 
                    or x_UB[p1+num_requests,p2] + x_UB[p2,p1+num_requests]==0 
                    or x_UB[p1+num_requests,p2+num_requests] + x_UB[p2+num_requests,p1+num_requests]==0
                        ):#nodes p1 and p2 are incompatible
                    print(f"Innnnnnnnnnnnnnnnnnnncommmmmmmmmmmmmmmmmmmmmmmmmmmmmpatibility requests {p1} {p2} in full set")
                    m.addConstr(gp.quicksum(y[p1,k] + y[p2,k] for k in K if z_UB[p1,k]*z_UB[p2,k]==1) <=1 )     
                    m.addConstr(gp.quicksum(y[p1+num_requests,k] + y[p2,k] for k in K if z_UB[p1,k]*z_UB[p2,k]==1) <=1)     
                    m.addConstr(gp.quicksum(y[p1,k] + y[p2+num_requests,k] for k in K if z_UB[p1,k]*z_UB[p2,k]==1) <=1)     
                    m.addConstr(gp.quicksum(y[p1+num_requests,k] + y[p2+num_requests,k] for k in K if z_UB[p1,k]*z_UB[p2,k]==1) <=1)     
                    #print("incompatibility")

    ############################################ Profit valid inequalities#################
    #### limit number of served requests per driver
    m.addConstrs( (gp.quicksum(y[i,k] for i in P if (i,k) in PK) <= Nkmax_loc[k]) for k in K)

    ### limit profit per driver 
    #m.addConstr( gp.quicksum(gp.quicksum(y[i,k] for i in P if (i,k) in PK) for k in K) <=  tot_req_loc ) #sum(Nkmax_loc[k] for k in K)
    # for k in KC:        
    #     m.addConstr( gp.quicksum(rev[i] * y[i,k] for i in P if (i,k) in PK)
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
    
    
                    
    # ------------------------------------------------------------
    # SOLVE
    # ------------------------------------------------------------
    m.update()
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
                next_nodes = [j for j in V if (current,j) in A and (current,j,k) in Ak and x[current,j,k].X > 0.9]
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
    

    output_list, objval = None, None
    if  m.SolCount> 0:
        objval = m.getObjective().getValue()
        GAP = 100*(1-objval/m.ObjBound) if m.ObjBound!=0 else 0
        output_list = ("3D_formulation",nbr_req, nbr_cdr, nbr_odr, m.Runtime, objval, GAP, m.NumConstrs, m.NumVars, m.NumBinVars, m.NumIntVars)#, m.NumContVars
        #print("3D_formulation",nbr_req, nbr_cdr, nbr_odr, m.Runtime, m.getObjective().getValue(), GAP, m.NumConstrs, m.NumVars, m.NumBinVars, m.NumIntVars, file=out_file)#, m.NumContVars

    # Number of served requests. For the '#_requests' objective this is used as a cap on the
    # requests a courier may serve, so it must be an upper bound: take Gurobi's bound (rounded
    # down; the objective is integer), not the incumbent, which is too low if the time limit is hit.
    if ObjType == '#_requests':
        n_served = math.floor(m.ObjBound + 1e-6) if math.isfinite(m.ObjBound) else len(P)
    else:
        n_served = sum([y[i,k].X for i in P for k in K if (i,k) in PK]) if m.SolCount > 0 else None

    return output_list, m.Runtime, objval, n_served

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
                if indicator[pair]+indicator[pair[1],pair[0]]==0:
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

def tighten_time_windows(E, L, S, distanceN, arcs, tol=1e-2, max_iters=100):
    """
    In-place TW tightening (Desrochers, Desrosiers & Solomon, 1992), adapted to time windows
    on the END of service: service at j starts at max(arrival, E_j - S_j) and ends in [E_j, L_j].
    E, L: dicts {node: earliest/latest end of service} for pickups and deliveries (changed in place).
    S: dict {node: service time}.
    distanceN: dict {(i, j): t_ij}.
    arcs: set of (i, j) pickup/delivery arcs that are still possible (e.g. x_ind[i,j] == 1);
          the predecessors of i are {j: (j,i) in arcs}, its successors {j: (i,j) in arcs}.
          Courier starts (predecessors of pickups) and the end depot (successor of deliveries)
          are added from Ek, Lk and z_UB.
    tol: stop when the total change of all windows in one pass is below tol.
    max_iters: maximum number of passes.
    Returns (number of passes, set of requests found unservable). A request is unservable when
    one of its nodes cannot be reached in time from any predecessor, cannot be left in time for
    any successor, or its pickup and delivery windows are incompatible.
    """
    preds = {i: [j for (j, m) in arcs if m == i] for i in P+D}
    succs = {i: [j for (m, j) in arcs if m == i] for i in P+D}
    unservable = set()

    for ite in range(max_iters):
        oldE = E.copy()
        oldL = L.copy()
        for i in P+D:
            req = i if i in P else i-num_requests
            if req in unservable:
                continue

            # earliest arrival from predecessors: Ei = max{Ei, min (Ej + tji + Si)}; finishing a
            # predecessor j, travelling and serving i. Couriers start at Ek and travel d_ki.
            inner_min_arriv_list = [E[j] + distanceN[(j,i)] + S[i] for j in preds[i]]
            if i in P:
                inner_min_arriv_list += [Ek[k] + dijk[0,i,k] + S[i] for k in K if z_UB[(i,k)]==1]
            inner_min_arriv = min(inner_min_arriv_list) if inner_min_arriv_list else math.inf
            if inner_min_arriv > L[i] + 1e-9:     # no predecessor reaches i in time
                unservable.add(req)
                continue
            E[i] = max(E[i], inner_min_arriv)

            # earliest departure time to successors: Ei = max{Ei, min{Li, min (Ej - tij - Sj)}};
            # finishing i earlier only adds waiting before every possible next stop. The end
            # depot (deliveries only) contributes the courier start times.
            inner_min_dep_list = [E[j] - distanceN[(i,j)] - S[j] for j in succs[i]]
            if i in D:
                inner_min_dep_list += [Ek[k] - dij[i,2*num_requests+1] for k in K if z_UB[(req,k)]==1]
            inner_min_dep = min(inner_min_dep_list) if inner_min_dep_list else -math.inf
            E[i] = max(E[i], min(L[i], inner_min_dep))

            # latest arrival time from predecessors: Li = min{Li, max{Ei, max (Lj + tji + Si)}};
            # i cannot be finished later than the latest arrival from any predecessor plus service.
            inner_max_arriv_list = [L[j] + distanceN[(j,i)] + S[i] for j in preds[i]]
            if i in P:
                inner_max_arriv_list += [Lk[k] + dijk[0,i,k] + S[i] for k in K if z_UB[(i,k)]==1]
            inner_max_arriv = max(inner_max_arriv_list) if inner_max_arriv_list else math.inf
            L[i] = min(L[i], max(E[i], inner_max_arriv))

            # latest departure time to successors: Li = min{Li, max (Lj - tij - Sj)}; i must be
            # finished early enough to reach and serve some next stop (deliveries: or the end
            # depot by the courier's shift end).
            inner_max_dep_list = [L[j] - distanceN[(i,j)] - S[j] for j in succs[i]]
            if i in D:
                inner_max_dep_list += [Lk[k] - dij[(i,2*num_requests+1)] for k in K if z_UB[(req,k)]==1]
            inner_max_dep = max(inner_max_dep_list) if inner_max_dep_list else -math.inf
            if inner_max_dep < E[i] - 1e-9:       # i cannot be left in time for any next stop
                unservable.add(req)
                continue
            L[i] = min(L[i], inner_max_dep)

        # pickup-delivery rule: the delivery d of request p follows its pickup, at least t_pd + S_d
        # later (the direct trip is the fastest, travel times being Euclidean).
        for p in P:
            if p in unservable:
                continue
            d = p + num_requests
            L[p] = min(L[p], L[d] - distanceN[(p,d)] - S[d])
            E[d] = max(E[d], E[p] + distanceN[(p,d)] + S[d])
            if L[p] < E[p] - 1e-9 or E[d] > L[d] + 1e-9:
                unservable.add(p)

        if sum([abs(L[i]-oldL[i]) for i in P+D]) + sum([abs(E[i]-oldE[i]) for i in P+D]) < tol:
            print("num of iterations in tightening of nodes",ite)
            break
    return ite + 1, unservable  # number of passes performed, unservable requests

MAX_TW_ROUNDS = 20   # cap on rounds of tightening + arc/assignment elimination

def compute_z_UB(unservable):
    """
    Courier-request elimination: z_UB[p,k] = 1 if courier k can serve request p on the fastest
    possible route (start -> pickup -> delivery; travel times are Euclidean, so detours only
    make things later). Windows bound the end of service.
    """
    z = {}
    for k in K:
        for p in P:
            d = p + num_requests
            pickup_end = max(Ek[k] + dijk[0,p,k] + si[p], ei[p])           # earliest end of pickup
            delivery_end = max(ei[d], pickup_end + dij[p,d] + si[d])       # earliest end of delivery
            z[p,k] = (1 if (p not in unservable
                            and Lk[k] >= pickup_end                         # pickup within the shift
                            and li[p] >= pickup_end                         # pickup before its deadline
                            and min(li[d], Lk[k]) >= delivery_end)          # delivery before its deadline and the shift end
                      else 0)
    return z

def tighten_courier_windows():
    """Start a courier no earlier than needed for its earliest pickup, end it at its latest delivery."""
    for k in K:
        # service at p may start at ei[p] - si[p], so leaving at ei[p] - si[p] - d_kp is still useful
        mylist = [ei[p]-si[p]-dijk[0,p,k] for p in P if z_UB[(p,k)]==1]
        min_mylist = min(mylist) if mylist else Ek[k]
        Ek[k] = max(Ek[k],min_mylist)  #tightening Ek of drivers based on earliest time of pickup node in PK
        mylist = [li[p+num_requests] for p in P if z_UB[(p,k)]==1]
        maxlist = max(mylist) if  mylist else Lk[k] # preventing taking max of empty list
        Lk[k] = min(Lk[k], maxlist ) # tightening Lk of drivers based on latest time of destinations of pickup node in PK

def compute_arc_elimination():
    """Arc elimination on the full A2 from the current windows; returns (x_UB, x_ind)."""
    x_UB = {(i,j): (0 if ( (ei[i] + dij[i, j] + si[j] > li[j])
                            or (i in D and j==i-num_requests) # a delivery never precedes its own pickup
                            )
                            else 1) for (i,j) in A2} #fixation based on early/latest time and x[delta(p),p]=0
    UBxIPC1 = {(nn,d): 0 if (d in D and d-num_requests!=nn and x_UB[d-num_requests,nn]==0 ) else 1 for (nn,d) in A2} # active
    UBxIPC2 = {(p,nn): 0  if (p in P and p+num_requests!=nn and x_UB[nn,p + num_requests]==0 ) else 1 for (p,nn) in A2} # active
    x_ind = {(i,j):x_UB[i,j]*UBxIPC1[i,j]*UBxIPC2[i,j] for (i,j) in A2}
    return x_UB, x_ind

# ------------------------------------------------------------
# LOAD INSTANCE FROM FILE
# ------------------------------------------------------------

filename = '3D_model_complexity_experiment_corrected_OccCost.txt'
out_file = open(filename, 'a')
print("Num.req.", "Num.com.", "Num.occ.", "Runtime(sec)", "ObjValue", "GAP(%)", "SL", "SL_occ", "comdriverutilisation","occriverutilisation", "NumConstrs", "NumVars", "NumBinVars", "NumIntVars", "total time incl decomposition (sec)", 'Runtime (sec)', file=out_file)

TimeLim = 3600
for vineq in ['None']: # ['SOP-k','2PC-k>','RC','Arc','Incom'] ['None','SOP-k','2PC','2PC-k>','RC','Arc','Incom','ALL'] add valid cuts,'2PC-k','SOP','SOP-k','None','SOP','2PC','RC','Arc','Incom','ALL','Incom','ALL','SOP','2PC','RC']:#'SOP','Arc','None','SOP','2PC','RC','Arc',
    all_except_vineq = [] #[vineq] # ['SOP','2PC','RC','Arc','Incom'] # for all except vineq
    #all_except_vineq.remove(vineq) # all valid ineq. except vineq
    print(all_except_vineq,vineq)
    nbr_req = 10
    while nbr_req <= 100:# in range(40,100,5): #40,80,40 # 5,80,10
        for nbr_cdr in range(0,12,2): #2,8,2 # 2,8,2)
            for nbr_odr in [10 - nbr_cdr]: #6 - nbr_cdr           
                #Instances contain untightened TW
                #InstancesTTW contain tightened TW
                instance = load_instance_from_txt(f"c:/Users/ahmad.alhanbali/PythonCode/Code Files/InstanceswithDemand/Instance_REQ{nbr_req}_CDR{nbr_cdr}_ODR{nbr_odr}_FS0_SS0.txt")
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

                # Tightening seems meaningful for latest time of driver as it hard however as the vehicle could wait earlier cannot be tightened
                Ek_org = Ek.copy() #keeping orginal Ek for per hour calculations of committed                
                Lk_org = Lk.copy() #keeping orginal Lk for per hour calculations of committed 
                
                # Pruning unfeasible links and assignments, alternated with time-window tightening
                # until nothing changes: tighter windows remove more arcs and courier-request pairs,
                # and fewer arcs/pairs give smaller predecessor/successor sets, i.e. tighter windows.
                print("Before tightening TW of nodes:",sum([li[i]-ei[i] for i in P+D]))
                print('Original tot driver TW length',sum([Lk_org[i]-Ek_org[i] for i in Ek.keys()]))
                unservable = set()   # requests that no courier can serve in time
                z_UB = compute_z_UB(unservable)
                x_UB, x_ind = compute_arc_elimination()
                for tw_round in range(MAX_TW_ROUNDS):
                    old_state = (dict(ei), dict(li), dict(Ek), dict(Lk), dict(z_UB), dict(x_ind))
                    arcs = {(i,j) for (i,j) in A2 if x_ind[i,j]==1
                            and (i if i in P else i-num_requests) not in unservable
                            and (j if j in P else j-num_requests) not in unservable}
                    iters, new_unservable = tighten_time_windows(ei, li, si, dij, arcs)
                    unservable |= new_unservable
                    tighten_courier_windows()
                    z_UB = compute_z_UB(unservable)
                    unservable |= {p for p in P if not any(z_UB[p,k] for k in K)}   # no courier can serve p
                    x_UB, x_ind = compute_arc_elimination()
                    change = (sum(abs(ei[i]-old_state[0][i]) + abs(li[i]-old_state[1][i]) for i in P+D)
                              + sum(abs(Ek[k]-old_state[2][k]) + abs(Lk[k]-old_state[3][k]) for k in K))
                    if change < 1e-2 and z_UB == old_state[4] and x_ind == old_state[5]:
                        break
                print("Tightening rounds:", tw_round+1, "| unservable requests:", sorted(unservable))
                print("After tightening TW of nodes:",sum([li[i]-ei[i] for i in P+D]))
                print('New tot driver TW length after tightening',sum([Lk[i]-Ek[i] for i in Ek.keys()]))
                print("Arcs kept (x_ind, x_UB):",sum(x_ind.values()),sum(x_UB.values()),
                      "| courier-request pairs kept:", sum(z_UB.values()))

                ####### Global Fixation used to avoid checking if a combination is in a feasible set############
                #  if x_ijk = 0 if z_UB[(i if i in P else i-n),k]==0 or z_UB[(j if j in P else j-n)]==0 or ei[i]+tij[i,j]+si[j]>Lk[k]
                aijk = {(i,j,k): (0 if (max(ei[i],min(Ek[k],li[i]))+tij[i,j]+si[j] > Lk[k] 
                                    or x_UB[(i,j)] == 0
                                    or z_UB[(i if i in P else i-num_requests),k] == 0 
                                    or z_UB[(j if j in P else j-num_requests),k] == 0 
                                    ) 
                        else 1) for i in P+D for j in P+D for k in K if i!=j} # for links in A2
                aijk = {**aijk,**{ (0,j,k) :(0 if ( z_UB[(j,k)] == 0 ) else 1) for j in P for k in K}} #add to it links in A1 i=0 and j in Pickups
                aijk = {**aijk,**{ (i,2*n+1,k) :(0 if ( z_UB[(i-num_requests,k)] == 0 ) else 1) for i in D for k in K if i!=0} }#add to it links in A3 i in destination and j in 2*n+1
                aijk = {**aijk,**{ (0,2*n+1,k) : 1 for k in K } }#add to it links 0 to 2*n+1

                # mydict = compute_reachability(x_ind)
                # for (i,j) in x_ind:
                #     if x_ind[i,j] == 0:
                #         print(f"no link from {i,j} {mydict[i,j]}")

                #print(compute_reachability(x_ind))                              

                # Update set of Arcs based on 
                A2 = [(i,j) for (i,j) in A2 if x_ind[i,j]==1]                
                A = A1 + A2 + A3

                # define PK set so variable y is defined on this set
                PK = [(p,k) for p in P for k in K if z_UB[p,k]==1]#if z_UB[p,k]==1
              
                # Costs and revenue
                # per-request revenue when the instance has one (Sun instances), else rev_order
                rev = {req["id"]: req.get("rev", instance["rev_order"]) for req in instance["requests"]}

                wC = instance["c_com_h"] / b   # committed cost/hour -> cost per minute
                cO = instance["c_occ_o"]          # occasional cost per order
                deltaC = instance["c_com_d"]      # committed distance cost (same as cC if you want)
                deltaO = instance["c_occ_d"]      # occasional distance cost

                #------------ constructing lists of nodes that can be together but there tot load is greater than veh. capacity
                if vineq in ('RC', 'ALL') or 'RC' in all_except_vineq:                    
                    viol_pick_lst = {}
                    for k in K:
                        pick_dem = {i: dem[i] for i in dem.keys() if dem[i]>0 and z_UB[(i,k)]==1} # violatinmg nodes that are assigned to courier k
                        temp_list = [] 
                        for set_siz in range(4,6,1):
                            temp = violating_subsets_fixed_size(pick_dem, Qk[k], set_siz,x_ind,minimal_only=True)
                            if temp:
                                temp_list += temp
                        if temp_list:#not empty sets are added
                            viol_pick_lst  =  {**viol_pick_lst,**{k: temp_list }}
                            
                    # print(viol_pick_lst)

                # first solving individually per single vehicle
                tot_time_single_veh, tot_profit_single_veh = 0, 0
                list_nkmax_ks = {} # list containing max number of requests to assign to drivers
                list_nkmax = {} # list containing max number of requests to assign to drivers
                list_maxprofit = {} # list containing max profit per driver
                tot_nkmax = 0 # tot number of requests to be served found using decomposition per vehicle
                original_P_l = {} #tracking the orginal number of requests assigned to courier before knapsack and maximization # req 

                for k in K:
                    if k in KC:#single committed courier
                        KO_l = []
                        KC_l = [k]
                    else:# single occasional courier
                        KO_l = [k]
                        KC_l = []
                    
                    P_l = sorted([p for (p,d) in PK if d==k]) # retreiving the set of pickups that could be assigned to driver k                    
                    D_l = [p+num_requests for p in P_l] # set of destination for driver k feasible set                    
                    original_P_l = {**original_P_l,**{k:len(P_l)}}
                    print('len(P_l)',len(P_l))
                    
                    # Arcs
                    A1_l = [(0,j) for j in P_l]
                    A2_l = [(i,j) for i in P_l+D_l for j in P_l+D_l if i != j and x_ind[i,j]==1]
                    A3_l = [(i,2*n+1) for i in D_l] + [(0,2*n+1)]
                    
                    # print(f"########## Solving knapsack for couier {k} ###########")
                    # swithcing off the knapsack
                    tks = 0
                    # list_nkmax_ks[k], tks = knapsack(k) # put upper bound on the maximum number of request to be served by the vehicle k
                    # to get nkmax we solve the per vehcile problem with knapsack as upper bound on the # of request to serve
                    #print(f"############ Maximizing # of requests with knapsack bound for courier {k} ###############")                     
                    # P_l,D_l,A1_l,A2_l,A3_l,
                    #_ , t, _ , nkmax1 = build_solve_3d(KO_l,KC_l,P_l,D_l,A1_l,A2_l,A3_l,{k: list_nkmax_ks[k]},{k:20*list_nkmax_ks[k]},'#_requests') # in model we maximize the number of requests
                    _ , t, _ , nkmax1 = build_solve_3d(KO_l,KC_l,P_l,D_l,A1_l,A2_l,A3_l,{k: original_P_l[k]},{k:20*original_P_l[k]},'#_requests') # in model we maximize the number of requests
                    # t = 0
                    #_ , t, _ , nkmax1 = build_solve_3d(KO_l,KC_l,P_l,D_l,A1_l,A2_l,A3_l,{k: len(P_l)},{k:20*len(P_l)},'#_requests') # in model we maximize the number of requests
                    #print("len(P_l)",len(P_l),"knapsack", list_nkmax_ks[k], 'runtime KS',tks, 'Max served req',nkmax1,'time of max # req',t)

                    #_ , t, profit, _ = build_solve_3d(KO_l,KC_l,P_l,D_l,A1_l,A2_l,A3_l,{k: list_nkmax_ks[k]},{k:20*list_nkmax_ks[k]},'profit') # could be an upper bound for driver's k profit
                    #_ , t1, profit, _ = build_solve_3d(KO_l,KC_l,KO_l+KC_l,{k: nkmax1},{k:20*nkmax1},'profit') # could be an upper bound for driver's k profit
                    # tot_time_single_veh += t + t1
                    tot_time_single_veh += t + tks
                    
                    #tot_profit_single_veh += profit
                    list_nkmax[k] = nkmax1 # original_P_l[k] # using maximizing number of requests                     
                    #tot_nkmax += nkmax1 # found using max of served request problems
                    #print(f"Courier {k}: Profit {profit}, time {t}, tot_time {tot_time_single_veh}")
                    # list_maxprofit[k]= profit

                print("tot_time_single_veh",tot_time_single_veh)

                print("Original",original_P_l) 
                print("knapsack",list_nkmax_ks,list_maxprofit)    
                print("Max. req",list_nkmax)

                #print("max. # of req",list_nkmax,list_maxprofit)
                # solve full model with Nkmax
                output_list, t, zobj, _ = build_solve_3d(KO,KC,P,D,A1,A2,A3,list_nkmax,list_maxprofit,'profit') # here we bound the total # of served req. by # of requests found using decompostion per k
                #output_list,t, zobj,_ = build_solve_3d(KO,KC,P,D,A1,A2,A3,list_nkmax_ks,list_maxprofit,'profit') # here we bound the total # of served req. by # of requests found using decompostion per k
                print(output_list, t, t+tot_time_single_veh, vineq, file=out_file)#, m.NumContVars            
                
                print("Tot per vehicle time and profit", tot_time_single_veh, tot_profit_single_veh)
                print("All vehicles time and profit EXCL presolving per vehicle time", t, zobj)
                print("All vehicles time and profit INCL presolving per vehicle time", t+tot_time_single_veh, zobj)
                
                # solving full with Nkmax replaced by len(PK for k in K) 
                # list_lenPK = {k:len([(p,k) for p in P if (p,k) in PK]) for k in K}
                # output_list,t_full_no_limit, _,_ = build_solve_3d(KO,KC,K,list_lenPK,{k:20*list_lenPK[k] for k in K})      
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
                # z2_UB = {(pickup,driver): (1 if (li[pickup] - si[pickup] >= Ek[driver-1] and Lk[driver-1] >= ei[pickup + num_requests] + si[pickup + num_requests]) else 0) for driver in K for pickup in P} #cart_prod_pickups_destinations                                    
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


# print("earliest and latest time 10",ei[10],li[10])
# print("earliest and latest time 30",ei[30],li[30])
# # 0→13→33→1→21→10→30→41 206
# # 0->13-1->-21->33->41
# print([(p,k) for p,k in PK if k==9],list_nkmax[9])


