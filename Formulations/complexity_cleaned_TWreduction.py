# !/usr/bin/env python
# coding: utf-8

# In[1]:

def tighten_time_windows(E, L, S, distanceN, tol=1e-9, max_iters=10_000):
    """
    In-place TW tightening (Desrochers, Desrosiers & Solomon, 1992).
    E, L: lists (length n) of earliest and latest times.
    distance: dict {(i, j): t_ij} for arcs in A (asymmetric allowed).
    tol: convergence tolerance for changes.
    max_iters: safety cap on iterations.
    """
    nE = len(E)

    for ite in range(100):
        oldE = E.copy()
        oldL = L.copy()
        for i in range(nE):            
            #Ei = max{Ei, min{Li, min (E j + Tji)}}
            inner_min_arriv_list = [E[j]+distanceN[(j,i)]+S[i] for (j,n) in distanceN.keys() if n == i]
            if inner_min_arriv_list:
                inner_min_arriv = min(inner_min_arriv_list)
            else:
                inner_min_arriv = -math.inf            
            E[i] = max(E[i], min(L[i],inner_min_arriv))            

            #earliest departure time to successors:  Ei = max{Ei, min{Li, min (E j − Ti j )}}; 
            inner_min_dep_list = [E[j] - distanceN[(i,j)] - S[j] for (n,j) in distanceN.keys() if n == i]
            if inner_min_dep_list:
                inner_min_dep = min(inner_min_dep_list)
            else:
                inner_min_dep = -math.inf            
            E[i] = max(E[i], min(L[i],inner_min_dep))
            
            #Li = min{Li, max{Ei, max( Li + Tji)}}
            inner_max_arriv_list = [L[i]+distanceN[(j,i)]+S[i] for (j,n) in distanceN.keys() if n == i]
            if inner_max_arriv_list:
                inner_max_arriv = max(inner_max_arriv_list)
            else:
                inner_max_arriv = + math.inf
            L[i] = min(L[i],max(E[i],inner_max_arriv))

            #• latest departure time to successors:  Li = min{Li, max{Ei, max (L j − Ti j )}}.
            inner_max_dep_list = [L[j]-distanceN[(i,j)]-S[j] for (n,j) in distanceN.keys() if n == i]
            if inner_max_dep_list:
                inner_max_dep = max(inner_max_dep_list)
            else:
                inner_max_dep = + math.inf
            L[i] = min(L[i],max(E[i],inner_max_dep))             
        if sum([abs(L[i]-oldL[i]) for i in range(len(L))]) + sum([abs(E[i]-oldE[i]) for i in range(len(L))]) <1e-2:
            break
    return ite  # number of iterations performed
    
import numpy as np
import math
# import networkx as nx
import random
from matplotlib import pyplot as plt
from gurobipy import *
from itertools import product
from math import *
# get_ipython().run_line_magic('matplotlib', 'inline')


# NOTE: This code can produce subtours if nodes are located at the same location and the pickup times or delivery times are equal to zero. In practice, this may rarely happen

# Seed number(s) for reproducibility
random.seed(1980)
np.random.seed(1982)

# Parameters

GR = 50 # grid size GRxGR

#NOTE: Code currently assumes that T is an integer multiple of b
T = 600 # planning horizon length
W = 60 # time window
b = 60 # block length for drivers

max_ord_per_driv = 333 # maximum number of orders per driver

TimeLim = 3600 #optimisation algorithm time limit

filename = 'Testing-9-13-2025-No_Valid_Ineq'+'T'+str(T)+'W'+str(W)+'b'+str(b)+'ord_driver'+str(max_ord_per_driv)+'TimeLim'+str(TimeLim)+'.txt'#'ord_driver'+str(max_ord_per_driv)+'wid_small_xij_1.txt'
out_file = open(filename, 'a')
print("Num.req.", "Num.com.", "Num.occ.", "Runtime(sec)", "ObjValue", "GAP(%)", "SL", "SL_occ", "comdriverutilisation","occriverutilisation", "Num.Constr", "Num.Vars", file=out_file)

#NOTE: Code currently assumes that T is an integer multiple of b
num_blocks = T / b
assert num_blocks - math.floor(num_blocks) <= 10e-6, "Number of blocks T/b not integer"
num_blocks = int(num_blocks)    # convert it to integer after asserting that it is

start_of_ones_tab = []
end_of_ones_tab = []
drivers_l0_first_stage_tab = []
t_start_tab = []

#Drivers input (used for committed and occasional) to choose from the combination of drivers. 
#in this way, we guarantee that if we add more drivers the situation will be worse in term of orders served and revenue for requests
#Note here 40 as it is larger s used as max number of committed drivers will 20.
# these will be used by both drivers (according to their index) except otherwise stated

for i in range(40):
    start_of_ones_tab.append(random.randint(0, num_blocks - 1)) #exclusively used by committed drivers
    end_of_ones_tab.append(min(start_of_ones_tab[i]+1,random.randint(start_of_ones_tab[i], num_blocks - 1),)) #  + 3 for max 4 blocks
    #end_of_ones_tab.append(random.randint(start_of_ones_tab[i], min(start_of_ones_tab[i]+1,num_blocks - 1) ) )
    drivers_l0_first_stage_tab.append((random.uniform(0, GR),random.uniform(0, GR)))
    t_start_tab.append(random.uniform(0, T - W)) #exclusively used by occasional drivers

pickups_tab = [] 
destinations_tab = [] 
delays_p_tab = [] 
delays_d_tab = [] 

#Request input  to choose from all combination of requests. 
#in this way, we guarantee that if we add more requests the situation will be better in term of orders served and revenue for same drivers
#Note here 200 as it is larger s used as max number of committed drivers will 75.

for i in range(200):
    pickups_tab.append((random.uniform(0, GR),random.uniform(0, GR)))
    destinations_tab.append((random.uniform(0, GR),random.uniform(0, GR)))
    delays_p_tab.append(random.randint(1, 5)) # delay/service time at pickup point 
    delays_d_tab.append(random.randint(1, 5)) # delay/service time at dest point

#for pickup and destination earliest and latest time
emax_tab = []
for i in range(T):
    emax_tab.append(random.uniform(0, i)) 

    
#Lower bound on objective function
LB_obj = -np.inf

# loop for complexity analysis starts here
# To compare with Furtado et al the number of drivers is kept maximum 6 drivers
for nbr_req in range(5,55,5): #40,80,40 # 5,80,10
    for nbr_cdr in range(2,8,2): #2,8,2 # 2,8,2)
        for nbr_odr in [6 - nbr_cdr]: #6 - nbr_cdr           
            
            print(nbr_req,nbr_cdr,nbr_odr)
            out_file = open(filename, 'a')
            num_of_first_stage_decisions = 1
            num_of_second_stage_runs = 5 #5           
            
            max_num_com_drivers = nbr_cdr # maximum number of committed drivers, i.e., max number of possible copies/blocks offered for a given timeslot
            #NOTE: This is assumed to always be the same but the actual realization/number that sign up are different, and this is what we vary in the first stage to determine the number of blocks to offer
            max_com_driver_set = list(range(max_num_com_drivers))

            max_num_requests = nbr_req

            num_infeasible_scenarios_list = []
            objective_vals_list = []
            runtime_vals_list = []
            ubound_vals_list = []
            SL_vals_list = []
            SL_occ_vals_list = []
            Frac_com_drv_used_vals_list = []
            Frac_occ_drv_used_vals_list = []
            best_average = -math.inf
                        
            for first_stage_run in range(num_of_first_stage_decisions):

                print("\n******************************** FIRST STAGE RUN # {} ********************************".format(first_stage_run))

                ### SETTING TIME WINDOWS FOR COMMITTED DRIVERS
                # e and l for drivers
                # Given a matrix with a string of 1's and 0's for every block in which a driver can be available in (row = driver, column = block), then we can use this to convert it into a time window for the driver
                driver_availability_matrix = np.zeros((max_num_com_drivers, num_blocks))

                # Generate random data for this matrix -- generate a start point for the ones and an end point after it
                e_first_stage = []
                l_first_stage = []
                num_com_drivers = 0
                for driver in max_com_driver_set:
                    #NOTE: Made probability of a driver being available more than 0.5 but can change it later
                    #NOTE: The way the blocks are generated may not give rise to nice patterns with sensible peaks, so this may need to be re-thought later
                    #TODO: Also search should maybe be smarter than randomly checking a first stage assignment
                    if  1  <= 1:#random.random()
                        driver_available = True
                    else:
                        driver_available = False
                    if driver_available:
                        start_of_ones = start_of_ones_tab[driver] #random.randint(0, num_blocks - 1)
                        end_of_ones = end_of_ones_tab[driver] #random.randint(start_of_ones, num_blocks - 1)
                        driver_availability_matrix[driver, start_of_ones:end_of_ones + 1] = 1

                        # Set the time windows accordingly
                        e_first_stage.append(b * start_of_ones)
                        l_first_stage.append(b * (end_of_ones + 1))

                        num_com_drivers += 1
                
                
                driver_availability_matrix = np.array(sorted(driver_availability_matrix, key=lambda x: (np.count_nonzero(x), tuple(x!=0))))
                print(driver_availability_matrix)
                
                com_driver_set = list(range(num_com_drivers))

                # Gather statistics about number of blocks offered in each time period (list format)
                num_blocks_offered_per_timeslot = np.sum(driver_availability_matrix, axis=0).tolist()

                # imag_src = [(0,0)] # imaginary depot for all drivers to start from
                # imag_delay_src = [0] # imaginary delay at source

                # imag_dest = [(10,10)] # imaginary destination added as last destination to guarantee all destinations are visited 
                # imag_delay_dest = [0] # imaginary delay at destination

                drivers_l0_first_stage = [] # drivers' initial locations
                delays_l0_first_stage = [] # delays for drivers' initial locations
                pickups_first_stage = [] # pickup points
                destinations_first_stage = [] # destination points
                delays_p_first_stage = [] # delays/service times at pickup points
                delays_d_first_stage = [] # delays/service times at destination points

                # Append the location of committed drivers first
                #NOTE: Location of committed drivers should be decided in the 1st stage
                for i in com_driver_set:
                    #drivers_l0_first_stage.append((random.uniform(0, GR),random.uniform(0, GR)))
                    drivers_l0_first_stage.append(drivers_l0_first_stage_tab[i])
                    delays_l0_first_stage.append(0)

                # Initialize some helper variables
                num_infeasible_scenarios = 0
                objective_vals = []
                runtime_vals = [] #runtime by Gurobi for the second stage run
                ubound_vals = [] #upper bound by Gurobi for the second stage run
                SL_vals = [] # service level as fraction of order served
                SL_occ_vals = [] # service level as fraction of order served done by occ drivers
                Frac_com_drv_used_vals = [] # fraction of comm drivers used
                Frac_occ_drv_used_vals = [] # fraction of occ drivers used
                
                for second_stage_scenario in range(num_of_second_stage_runs):

                    print("\n~~~~~~~~~~ Second stage scenario # {} ~~~~~~~~~~".format(second_stage_scenario))

                    # Generating data about occasional drivers and requests for this second stage run
                    #NOTE: Added +1 to avoid dividing by zero
                    num_occ_drivers = nbr_odr #np.random.poisson(0.4 * T/W) + 1 # number of occasional drivers during the whole planning period
                    num_drivers = num_occ_drivers + num_com_drivers

                    occ_driver_set = list(range(num_com_drivers, num_drivers))

                    num_requests = nbr_req #random.randint(2, max_num_requests)

                    # Parameters and other things are reset from scenario to scenario here
                    drivers_l0 = drivers_l0_first_stage.copy()
                    delays_l0 = delays_l0_first_stage.copy()
                    pickups = pickups_first_stage.copy()
                    destinations = destinations_first_stage.copy()
                    delays_p = delays_p_first_stage.copy()
                    delays_d = delays_d_first_stage.copy()
                    e = e_first_stage.copy()
                    l = l_first_stage.copy()

                    # Append the location of occasional drivers next
                    # The occasional drivers locations are chosen randomly
                    for i in occ_driver_set:
                        drivers_l0.append(drivers_l0_first_stage_tab[i])
                        delays_l0.append(0)

                    for i in range(num_requests):
                        #pickups.append((random.randint(0, GR),random.randint(0, GR)))
                        #destinations.append((random.randint(0, GR),random.randint(0, GR)))
                        #delays_p.append(0.1 * random.randint(1, 3)) # delay/service time at pickup point 
                        #delays_d.append(0.2 * random.randint(1, 3)) # delay/service time at dest point
                        pickups.append(pickups_tab[i])
                        destinations.append(destinations_tab[i])
                        delays_p.append(delays_p_tab[i])
                        delays_d.append(delays_d_tab[i])
                        

                    # List of all points
                    # allpoints = imag_src + drivers_l0 + pickups + destinations + imag_dest # 1st part for pickups and second for destinations
                    allpoints = drivers_l0 + pickups + destinations # drivers, then pickups, then destinations

                    # List of all delays
                    # alldelays = imag_delay_src + delays_l0 + delays_p + delays_d + imag_delay_dest
                    alldelays = delays_l0 + delays_p + delays_d

                    # Compute key parameters of MIP model formulation
                    num_drivers = len(drivers_l0)
                    num_delays_l0 = len(delays_l0)

                    # Check if input dimensions match
                    assert num_drivers == num_delays_l0, 'Number of drivers and delays do not match.'

                    num_pickups = len(pickups)
                    num_destinations = len(destinations)
                    num_points = len(allpoints)
                    cart_prod = list(product(range(num_points),range(num_points))) # tuples of all nodes
                    cart_prod_pickups_destinations = list(product(range(num_drivers, num_points), range(num_drivers, num_points))) # tuples of pickups and destinations only
                    cart_prod_points_drivers = list(product(range(num_drivers, num_points),range(num_drivers))) # tuples of all points and drivers (i, d)
                    #cart_prod_allpoints_drivers = list(product(range(num_points),range(num_drivers))) # tuples of all points and drivers (i, d)

                    # Lists (sets) of indices for re-use later and better readability
                    driver_set = list(range(num_drivers))
                    pickup_set = list(range(num_drivers, num_drivers + num_pickups))
                    dest_set = list(range(num_drivers + num_pickups, num_points))
                    pickup_and_dest_set = list(range(num_drivers, num_points))
                    all_points_set = list(range(num_points))


                    # In[3]:
                    # For visualization, we make annotations to be added to the scatter plot 
                    annot = list(all_points_set) # label of nodes
                    cannotlist = list(all_points_set) # completion time of points from solver 

                    # annot[0] = "IS" # imaginary source node 
                    for i in driver_set:
                        annot[i] = "l" + str(i) # initial location of driver

                    for i in range(num_pickups):
                        annot[i + num_drivers] = "p" + str(i) # pickup point

                    for i in range(num_destinations):
                        annot[i + num_drivers + num_pickups] = "d" + str(i) # destination point

                    # annot[-1] = "Fd" # final imaginary destination

                    # print(annot)
                    # print(cannotlist)


                    # In[4]:


                    xa, ya = zip(*allpoints)
                    #xa = list(map(lambda x: x[0], allpoints))
                    #ya = list(map(lambda x: x[1], allpoints))
                    #print(xa,ya)

                    #fig, ax = plt.subplots()
                    #ax.scatter(current_loc[0],current_loc[1]) #initial position
                    #ax.annotate('l0', (current_loc[0]+0.05,current_loc[1]+0.05)) # label the initial point

                    #ax.scatter(xa[0:num_points], ya[0:num_points])
                    #ax.set_xlim([-1, max(xa[0:num_points]) + 1])
                    #ax.set_ylim([-1, max(ya[0:num_points]) + 1])
                    #ax.grid(True)

                    #Add annotation to the scattered points
                    #for i, txt in enumerate(annot[:]):
                    #    #ax.annotate(txt, (xa[i]+0.09, ya[i]))
                    #    ax.annotate(annot[i], (xa[i] - 0.5, ya[i]))


                    # In[5]:


                    # This function determines the Euclidean distance between any two locations/points
                    def compute_euclidean(loc1, loc2):
                        dx = loc1[0] - loc2[0]
                        dy = loc1[1] - loc2[1]
                        return sqrt((dx * dx) + (dy * dy))

                    #def compute_distance(loc1, loc2):
                    #    dx = loc1[0] - loc2[0]
                    #    dy = loc1[1] - loc2[1]
                    #    return np.sqrt(dx*dx + dy*dy)

                    ### COMPUTING TRAVEL TIMES
                    # Compute costs or travel times
                    distance = {(i, j): compute_euclidean(allpoints[i], allpoints[j]) for (i, j) in cart_prod}

                    ### SETTING (GENERATED) TIME WINDOWS FOR OCCASIONAL DRIVERS
                    # Assumed occasional will work for an hour and they show up at any point in time
                    #NOTE: We could assume that occasioanl have a higher probability of showing up during peak times
                    #NOTE: We could also make their time windows have variable lengths later
                    for driver in occ_driver_set:
                        t_start = t_start_tab[driver] #random.randint(0, T - W)
                        e.append(t_start)
                        l.append(t_start + W) # occasional drivers are working for for fixed window time. I think this can be randomized.

                    #l = e.copy()

                    # Setting e and l for pickups
                    for i in pickup_set:
                        emax = floor(T - distance[i,i+num_pickups] - alldelays[i+num_pickups])
                        #e.append(random.randint(0, emax)) # randomly choose e in this range
                        e.append(emax_tab[emax])
                        l.append(e[i] + W)

                    # Setting e and l for destinations
                    for i in dest_set:
                        e.append(e[i-num_pickups] + distance[i - num_pickups,i]) # chosen using Ropke and Coudreau paper PDP TW
                        l.append(e[i] + W)

                    # Old code form the complexity analysis results
                    # A pickup node can be assigned to a driver if its lateness time and the earliest time of its destination are within the time window of the driver
                    # z_UB is used for fixation to limit the search space
                    z_UB = {(pickup,driver): (1 if (l[pickup] - alldelays[pickup] >= e[driver] and l[driver] >= e[pickup + num_pickups] + alldelays[pickup + num_pickups]) else 0) for driver in driver_set for pickup in pickup_set} #cart_prod_pickups_destinations
                    
                    # Limit on x_UB_{ij} is an upper bound on variable x_{ij}: if e_i + t_{ij} + d_j - l_j > 0 (d_j is delay at j), then x_UB_{ij} = 0; otherwise it is 1
                    x_UB = {(i,j): (0 if (e[i] + distance[i, j] + alldelays[j] > l[j] ) else 1) for (i,j) in cart_prod} #cart_prod_pickups_destinations
                    # Infeasible path const: if x_pn == 0 then x_nd==0 (IPC2) if x_nd == 0 then x_pn==0 (IPC1)
                    #UBxIPC1 = {(n,d): 0 if (d in pairs.values() and dest_pickup_pairs[d]!=n and model.UBx[dest_pickup_pairs[d],n]==0 ) else 1 for n in model.N for d in model.N if n != d}
                    #UBxIPC2 = {(p,n): 0 if (p in pairs.keys() and pairs[p]!=n and model.UBx[n,pairs[p]]==0 ) else 1 for n in model.N for p in model.N if n != p}
                    UBxIPC1 = {(n,d): 0 if (d in dest_set and d-num_pickups!=n and x_UB[d-num_pickups,n]==0 ) else 1 for (n,d) in cart_prod_pickups_destinations}
                    UBxIPC2 = {(p,n): 0 if (p in pickup_set and p+num_pickups!=n and x_UB[n,p+num_pickups]==0 ) else 1 for (p,n) in cart_prod_pickups_destinations}
                    ## Defining M_{ij}. Currently, if node is not visited currently its c is between 0 and T
                    #big_M  =  {(i,j): T + W + alldelays[j] + distance[i,j] for (i,j) in cart_prod}
                    #big_M = {(i,j): l[i] - e[j] + alldelays[j] + distance[i,j] for (i,j) in cart_prod}                    
                    # min max - max min
                    #big_M = {(i,j): min(l[i],max(l[d] for d in driver_set)) - max(e[j],min(e[d] for d in driver_set)) + alldelays[j] + distance[i,j] for (i,j) in cart_prod}                    
                    #L_ij=max⁡{min⁡〖{l_i,max⁡{l_1,…,l_S } }-max⁡〖{e_j,min⁡{e_1,…,e_S } }+w_j+t_ij 〗 〗,0}
                    # max maxminus minmin
                    #big_M = {(i,j): max(l[i],max(l[d] for d in driver_set)) -min(e[j],min(e[d] for d in driver_set)) + alldelays[j] + distance[i,j] for (i,j) in cart_prod}                    
                    # li minus min min
                    #big_M = {(i,j): l[i] - min(e[j],min(e[d] for d in driver_set)) + alldelays[j] + distance[i,j] for (i,j) in cart_prod}                    

                    # li minus ej
                    big_M = {(i,j): max(0,l[i] - e[j] + alldelays[j] + distance[i,j]) for (i,j) in cart_prod}                    
                    
                    ## Defining Max_{i} repsenting the largest distance from i to other nodes 
                    #Max_d = {i: max(distance[i,j] for j in pickup_and_dest_set) for i in pickup_and_dest_set}
                    Max_d = {i: max(distance[i,j]*x_UB[i,j]*UBxIPC1[i,j]*UBxIPC1[i,j] for j in pickup_and_dest_set) for i in pickup_and_dest_set}
                    ## super valid MTZ constraints
                    svMTZ = {(i,j): min(-alldelays[i] -distance[j,i], e[j]- e[i]) for (i,j) in cart_prod}

                    # ## Reducing time windows of all pickups and destinations. NB doing them also for drivers make sense as well
                    origl = l.copy()
                    orige = e.copy()
                    #el = e#[num_drivers:].copy()
                    #nl = l#[num_drivers:].copy()
                    #delaysl = alldelays#[num_drivers:].copy()
                    print("Before E:",sum([l[i]-e[i] for i in range(len(l))]))
                    # #distanceN = {(i-num_drivers, j-num_drivers): compute_euclidean(allpoints[i], allpoints[j]) for (i, j) in cart_prod_pickups_destinations}
                    # #distanceN = {(i, j): compute_euclidean(allpoints[i], allpoints[j]) for (i, j) in cart_prod}
                    distanceN = {(i, j): compute_euclidean(allpoints[i], allpoints[j]) for (i, j) in cart_prod 
                                 if i!=j and ((i in driver_set and j in pickup_set ) 
                                              or ( (i,j) in cart_prod_pickups_destinations and x_UB[i,j]*UBxIPC1[i,j]*UBxIPC2[i,j]!=0))}
                    # print(num_drivers,len(el))
                    iters = tighten_time_windows(e, l, alldelays, distanceN)
                    print("iters:", iters)
                    print("After E:",sum([l[i]-e[i] for i in range(len(l))]))
                    # e[num_drivers:] = el.copy()
                    # l[num_drivers:] = nl.copy()
                    #e = el.copy()
                    #l = nl.copy()
                    
                    #Add cost factors for the committed and occasional drivers
                    c_com_h = 0.4 # fixed cost for committed driver per hour this includes wasted time in traffic jams 
                    c_com_d = 0.2 # cost paid to committed driver per unit distance including all travelled distance 
                    # from initial drivers location to first pickup point this makes sense as the platform gives advice 
                    # on initial drivers location

                    c_occ_o = 0.5 # fixed cost paid to occasional driver per order if driver is taking k order in a trip total cost will be k times
                    c_occ_d = 0.2 # cost paid to occasional driver per unit distance excluding distance from driver location to 1st pickup. This first mile drive 
                    # is influenced by driver decision and not platform. 
                    # c_occ_h = 0.05 # cost paid to occasional driver per hour to include time in traffic jams. I feel this is not needed as the driver 
                    # will have the incentive to increase travel time to earn more which can increase the lateness
                    rev_order = 20 # rev per order

                    # In[6]:

                    m = Model()
                    #m.Params.lazyConstraints = 1
                    m.Params.timeLimit = TimeLim #1000's
                    #m.Params.LogToConsole = 0 # to surpress output for individual models

                    ### DECISION VARIABLES
                    # Original variables from model x_{ij}
                    x = m.addVars(cart_prod, vtype=GRB.BINARY, name="x") # link i to j used
                    # Additional variables from model xp_{ij}
                    #xplus = m.addVars(cart_prod, vtype=GRB.BINARY, name="xplus") # link i to j used
                    # New variables z_{id} which enables us to set early and late times for the drivers
                    z = m.addVars(cart_prod_points_drivers, vtype=GRB.BINARY, name="z") # driver ID of node i
                    # New variables added to separate the travelled distance of committed drivers from that of the occasional drivers
                    w = m.addVars(cart_prod_points_drivers, vtype=GRB.CONTINUOUS, name="w") # distance of node i to next node in the trip done by driver
                    # Delta variables for linearization of constraints involving z_{id}
                    # delta_plus = m.addVars(cart_prod_points_drivers, vtype=GRB.BINARY, name="delta_+") # driver ID of node i
                    # delta_minus = m.addVars(cart_prod_points_drivers, vtype=GRB.BINARY, name="delta_-") # driver ID of node i
                    #delta_00 = m.addVars(cart_prod_points_drivers, vtype=GRB.BINARY, name="delta_0") # driver ID of node i

                    z1d = m.addVars(num_points, vtype=GRB.BINARY, name="z1d") # completion time of each point/node
                    c = m.addVars(num_points, lb=0, vtype=GRB.CONTINUOUS, name="c") # completion time of each point/node
                    # v = m.addVars(num_points, lb=0, vtype=GRB.CONTINUOUS, name="v") # route index assigned to single vehicle
                    # cmax = m.addVar(lb=0,vtype=GRB.CONTINUOUS, name="cmax") # maximum completion time
                    #cf = m.addVar(lb=0,vtype=GRB.CONTINUOUS, name="cf")
                    # cost = m.addVar(lb=0,vtype=GRB.CONTINUOUS, name="cost")

                    obj = m.addVar(lb=0,vtype=GRB.CONTINUOUS, name="obj")

                    m.update()

                    ### CONSTRAINTS
                    # ## c_{max} constraint
                    # m.addConstrs((cmax >= c[i] for i in all_points_set), name="Max C") # c_{max} >= c_i

                    ## Each node can be visited (and left) at most once
                    # Links in - Constraints 2 in model1.docx
                    m.addConstrs((quicksum(x[i,j] for i in all_points_set ) <= 1 for j in pickup_and_dest_set),name="linkstonode<=1 except for drivers")
                    # Links out - Constraints 3 in model1.docx
                    m.addConstrs((quicksum(x[i,j] for j in pickup_and_dest_set ) <= 1 for i in all_points_set),name="linksoutdrivers<=1")
                    
                    #m.addConstrs((quicksum(x[i,j] for i in all_points_set) == z1d[i] for j in pickup_and_dest_set),name="linkstonode<=1 except for drivers")
                    #m.addConstrs((quicksum(x[i,j] for i in all_points_set) <= z1d[i] for j in pickup_and_dest_set),name="linkstonode<=1 except for drivers")
                    # Links out - Constraints 3 in model1.docx
                    #m.addConstrs((quicksum(x[i,j] for j in pickup_and_dest_set) == z1d[i] for i in pickup_set),name="linksoutdrivers<=1")
                    #m.addConstrs((quicksum(x[i,j] for j in pickup_and_dest_set) <= z1d[i] for i in driver_set),name="linksoutdrivers<=1")#drivers not nessecarily used
                    #m.addConstrs((quicksum(x[i,j] for j in pickup_and_dest_set) <= z1d[i] for i in dest_set),name="linksoutdrivers<=1")#link out of destinations may not exist
                    #m.addConstrs((quicksum(x[i,j] for j in pickup_and_dest_set) <= z1d[i] for i in all_points_set),name="linksoutdrivers<=1")
                    #m.addConstrs((quicksum(x[driver,j] for j in pickup_and_dest_set) <= 1 for driver in driver_set),name="linksoutdrivers<=1")
                    #m.addConstrs((quicksum(x[pickup,j] for j in pickup_and_dest_set) <= 1 for pickup in pickup_set),name="linksoutpickups==1")
                    #m.addConstrs((quicksum(x[dest,j] for j in pickup_and_dest_set) <= 1 for dest in dest_set),name="linksoutdestination<=1")

                    # If a pickup node is visited, then its destination should also be visited -- Constraints 4 in model1.docx
                    #m.addConstrs( (z1d[i] == z1d[i+num_pickups] for i in pickup_set),name="if pickup visited, then its dest should be visited")
                    m.addConstrs((quicksum(x[j,i] for j in all_points_set ) == quicksum(x[j,i+num_pickups] for j in pickup_and_dest_set ) for i in pickup_set),name="if pickup visited, then its dest should be visited")

                    # For pickup nodes, we must have flow balance, i.e., the number of links going in to a node should be equal to the number going out of it - Constraints 5 in model1.docx
                    m.addConstrs((quicksum(x[i,j] for i in all_points_set ) == quicksum(x[j,i] for i in pickup_and_dest_set )  for j in pickup_set),name="flow balance for pickup nodes")

                    # For destination nodes, number of links going out should be <= number coming in (not necessarily equal since destination nodes can be terminal points) - Constraints 6 in model1.docx
                    m.addConstrs((quicksum(x[i,j] for j in pickup_and_dest_set ) <= quicksum(x[j,i] for j in pickup_and_dest_set ) for i in dest_set),name="we can only leave a destination if we visit it")

                    ## Drivers' time windows must be respected - Constraints 7 in model1.docx
                    m.addConstrs((c[i] >= quicksum((e[d] ) * z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time after driver early time")
                    #m.addConstrs((c[i] >= e[i] + quicksum((e[d] - e[i]) * z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time after driver early time")
                    # old bounding in complexity
                    #m.addConstrs((c[i] >= e[i] + (e[d] - e[i]) * z[i,d] for d in driver_set for i in pickup_and_dest_set), name="completion time after driver early time")
                    #m.addConstrs((c[i] >= e[d] + T * (z[i,d]-1) for d in driver_set for i in pickup_and_dest_set), name="completion time after driver early time")
                    #m.addConstrs((c[i] >= quicksum(e[d] * z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time after driver early time")
                    m.addConstrs((c[i] <= quicksum((l[d]) * z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time before driver late time")
                    #m.addConstrs((c[i] <= l[i] + quicksum((l[d] - l[i]) * z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time before driver late time")
                    # old bounding in complexity
                    #m.addConstrs((c[i] <= l[i] + (l[d] - l[i]) * z[i,d] for d in driver_set for i in pickup_and_dest_set), name="completion time before driver late time")
                    #m.addConstrs((c[i] <= l[d] + T * (1-z[i,d]) for d in driver_set for i in pickup_and_dest_set), name="completion time before driver late time")
                    #m.addConstrs((c[i] <= quicksum((l[d]) * z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time before driver late time")

                    ## Node time wimdows must be respected - Constraints 8 in model1.docx
                    m.addConstrs((c[i] >= e[i]*z1d[i] for i in all_points_set), name="completion time after early time")
                    #m.addConstrs((c[i] >= quicksum(e[i] * z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time should be after early time")        
                    m.addConstrs((c[i] <= l[i]*z1d[i] for i in all_points_set), name="completion time before late time")
                    #m.addConstrs((c[i] <= T + (l[i] - T) * quicksum(z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time before late time")
                    #m.addConstrs((c[i] <= (l[i]) * quicksum(z[i,d] for d in driver_set) for i in pickup_and_dest_set), name="completion time before late time")

                    ## Completion times of consecutive nodes must make sense: C_j >= C_i + t_ij - M(1 - x_ij) - Constraints 9 in model1.docx
                    #m.addConstrs((c[driver] == 0 for driver in driver_set), name="completion time for img source and drivers is 0")
                    #m.addConstrs((c[j] >= c[i] + alldelays[j] + distance[i,j] - big_M[i, j] * (1 - x[i,j]) for i in all_points_set for j in pickup_and_dest_set if i!=j), name="completion times of consecutive nodes")
                    #m.addConstrs((c[j] >= c[i] + alldelays[j] + distance[i,j] - big_M[i, j] * (1 - x[i,j]) for i in driver_set for j in pickup_set if i!=j), name="completion times of consecutive nodes drivers to pickups")
                    #m.addConstrs((c[j] >= c[i] + alldelays[j] + distance[i,j] - big_M[i, j] * (1 - x[i,j]) for i in pickup_and_dest_set for j in pickup_and_dest_set if i!=j), name="completion times of consecutive nodes")                    
                    
                    #lifted
                    #m.addConstrs( (big_M[i,j] - distance[i, j] >= c[i] - c[j] + alldelays[j] + big_M[i,j] * x[i, j] for i in driver_set for j in pickup_and_dest_set), name="lifted MTZ")
                    #m.addConstrs( (big_M[i,j] - distance[i, j] >= c[i] - c[j] + alldelays[j] + big_M[i,j] * x[i, j] + (big_M[i,j] - distance[i, j] - alldelays[j] + e[j] - l[i]) * x[j, i] for i,j in cart_prod_pickups_destinations if i!=j), name="lifted MTZ")
                    
                    #Supervalid
                    # no link from pick or destination to driver (open tours) xji=0 and z1d[driver]==1 (by definition)
                    m.addConstrs((  c[i] - c[j] + (distance[i, j] + alldelays[j]) * x[i, j] <= l[i] - e[j]*z1d[j] - (l[i] - e[j]) * (x[i, j] ) for i in driver_set for j in pickup_set), name="completion times of consecutive nodes drivers to pickups")
                    m.addConstrs((  c[i] - c[j] + (distance[i, j] + alldelays[j]) * x[i, j] + svMTZ[i,j] * x[j,i] <= l[i]*z1d[i] - e[j]*z1d[j] - (l[i] - e[j]) * (x[i, j] + x[j, i]) for i in pickup_and_dest_set for j in pickup_and_dest_set if i!=j), name="completion times of consecutive nodes")
                   
                                  
                    # Completion time of a destination node must be >= than that of the pickup node (pickups must be completed before corresponding deliveries) - Constraints 9 in model1.docx
                    # for lifted we removed z1d[pickup]
                    # m.addConstrs((c[pickup + num_pickups] >= c[pickup] + (alldelays[pickup + num_pickups] + distance[pickup,pickup + num_pickups]) for pickup in pickup_set), name="pickups must be completed before deliveries")
                    m.addConstrs((c[pickup + num_pickups] >= c[pickup] + z1d[pickup]*(alldelays[pickup + num_pickups] + distance[pickup,pickup + num_pickups]) for pickup in pickup_set), name="pickups must be completed before deliveries")
                    #m.addConstrs((c[pickup + num_pickups] >= c[pickup] + (alldelays[pickup + num_pickups] + distance[pickup,pickup + num_pickups] ) * z[pickup,driver] for pickup in pickup_set for driver in driver_set), name="pickups must be completed before deliveries")

                    # Pickups and deliveries should be on the same route
                    m.addConstrs((z[pickup, driver] == z[pickup + num_pickups, driver] for pickup in pickup_set for driver in driver_set), name="pickup and delivery are in the same route")

                    # Constraints connecting x_{ij} to z_{ik}
                    m.addConstrs(( z[i, driver] >= x[driver, i] for driver in driver_set for i in pickup_set), name="to ensure correct propogation of driver number")
                    #m.addConstrs(( z[i, driver] <= quicksum(x[j, i] for j in pickup_and_dest_set) for driver in driver_set for i in dest_set), name="to ensure correct propogation of driver number")
                    m.addConstrs((z[i, driver] >= z[j, driver] - (1 - x[i, j]) for driver in driver_set for i in pickup_and_dest_set for j in pickup_and_dest_set), name="nodes on same route have same route index")
                    m.addConstrs((z[i, driver] <= z[j, driver] + (1 - x[i, j]) for driver in driver_set for i in pickup_and_dest_set for j in pickup_and_dest_set), name="nodes on same route have same route index")

                    #one driver per pickup node
                    m.addConstrs( (quicksum(z[i, driver] for driver in driver_set) == z1d[i] for i in pickup_and_dest_set), name="node is visited at most by 1 driver")
                    
                    # as we introduce z1d as binary no need for this constraint
                    #m.addConstrs( (quicksum(z[i, driver] for driver in driver_set) <= 1 for i in pickup_and_dest_set), name="node is visited at most by 1 driver")
                    
                    #nbr of requests per driver
                    #m.addConstrs( (quicksum(z[i, driver] for i in pickup_set) <= max_ord_per_driv for driver in driver_set), name="driver can take maximum requests")

                    #m.addConstrs( (quicksum(z[i, driver] for i in dest_set) <= max_ord_per_driv for driver in driver_set), name="driver can take maximum requests")
                    #m.addConstrs((quicksum(x[i, j] for i in all_points_set) == quicksum(z[j, driver] for driver in driver_set) for j in pickup_and_dest_set), name="if a node is visited, then it should be visited by exactly one driver")

                    ## Constraints on w[i,driver] to compute the distances for the objective function - Constraints 15 in model1.docx
                    m.addConstrs((w[i,driver] <= quicksum(distance[i,j] * x[i,j] for j in pickup_and_dest_set) + Max_d[i] * (1 - z[i,driver]) for driver in driver_set for i in pickup_and_dest_set), name="w_i,d<=x_i,j_1")
                    m.addConstrs((w[i,driver] >= quicksum(distance[i,j] * x[i,j] for j in pickup_and_dest_set) - Max_d[i] * (1 - z[i,driver]) for driver in driver_set for i in pickup_and_dest_set), name="w_i,d<=x_i,j_2")
                    #m.addConstrs((w[i,driver] >= quicksum(distance[i,j] * x[i,j] for j in pickup_and_dest_set) for driver in driver_set for i in pickup_and_dest_set), name="w_i,d<=x_i,j")
                    m.addConstrs((w[i,driver] <= Max_d[i] * z1d[i] for driver in driver_set for i in pickup_and_dest_set), name="w_i,d<=z_i,d_3")
                    #m.addConstrs((w[i,driver] <= Max_d[i] * z[i,driver] for driver in driver_set for i in pickup_and_dest_set), name="w_i,d<=z_i,d_3")

                    ## VARIABLE FIXATIONS
                    #  Their purpose is to speed up the run time (these are as described in Section 4 of Furtado et al)
                    m.addConstrs((x[i,i] == 0 for i in all_points_set), name="xii==0") # no self-loops
                    m.addConstrs((x[i,driver] == 0 for i in pickup_and_dest_set for driver in driver_set) ,name="no link to drivers")
                    #m.addConstrs((x[num_points-1, i] == 0 for i in all_points_set),name="no link out final destination zero") 
                    #m.addConstrs((x[pickup, num_points-1] == 0 for pickup in pickup_set) , name = "no link from pickup to final destination")
                    m.addConstrs((x[pickup+num_pickups,pickup] == 0 for pickup in pickup_set), name="no link from destination to its pickup")
                    #m.addConstrs((xplus[pickup+num_pickups,pickup] == 0 for pickup in pickup_set), name="no link from destination to its pickup")        
                    m.addConstrs((x[driver,dest] == 0 for driver in driver_set for dest in dest_set), name="no link from drivers to destinations")                    
                    m.addConstrs((x[driver1,driver2] == 0 for driver1 in driver_set for driver2 in driver_set), name="no link from driver to driver")
                    #m.addConstrs((x[dest,driver] == 0 for driver in driver_set for dest in dest_set), name="no link from destination to drivers")
                    m.addConstrs((x[i,j] ==  0 for i in all_points_set for j in all_points_set if x_UB[i,j]==0),name="no link from i to j is x_UB_ij is zero")
                    m.addConstrs((x[i,j] == 0 for (i,j) in cart_prod_pickups_destinations if UBxIPC1[i,j]*UBxIPC2[i,j]==0),name="no link from i to j is UBxIPC12_ij is zero")
                    #m.addConstrs((x[driver,j] <= x_UB[driver,j] for driver in driver_set for j in pickup_and_dest_set),name="no link from i to j is x_UB i j is zero")                    m.addConstrs((x[driver1,driver2] == 0 for driver1 in driver_set for driver2 in driver_set),name="no link between drivers")
                    #m.addConstrs((x[i,j] + x[j,i] <=1 for i,j in cart_prod),name="x_ij + x_ji <=1")                    
                    m.addConstrs((z[pickup,driver] == 0 for driver in driver_set for pickup in pickup_set if z_UB[pickup,driver]==0), name="feasibility of driver to a node")
                    m.addConstrs((z[dest,driver] == 0 for driver in driver_set for dest in dest_set if z_UB[dest-num_pickups,driver]==0), name="feasibility of driver to a node")
                    # m.addConstrs((x[i,j] <= x_UB[i,j] for i in all_points_set for j in all_points_set),name="no link from i to j is x_UB_ij is zero")
                    # m.addConstrs((x[i,j] <= UBxIPC1[i,j]*UBxIPC2[i,j] for (i,j) in cart_prod_pickups_destinations),name="no link from i to j is UBxIPC12_ij is zero")
                    # #m.addConstrs((x[driver,j] <= x_UB[driver,j] for driver in driver_set for j in pickup_and_dest_set),name="no link from i to j is x_UB i j is zero")                    m.addConstrs((x[driver1,driver2] == 0 for driver1 in driver_set for driver2 in driver_set),name="no link between drivers")
                    # #m.addConstrs((x[i,j] + x[j,i] <=1 for i,j in cart_prod),name="x_ij + x_ji <=1")                    
                    # m.addConstrs((z[pickup,driver] <= z_UB[pickup,driver] for driver in driver_set for pickup in pickup_set), name="feasibility of driver to a node")
                    # m.addConstrs((z[dest,driver] <= z_UB[dest-num_pickups,driver] for driver in driver_set for dest in dest_set), name="feasibility of driver to a node")
                    
                    #newly added
                    #m.addConstrs((x[i,j] + x[j,i] <=1 for i,j in cart_prod),name="x_ij + x_ji <=1")                    
                    #m.addConstrs((x[pickup,driver] == 0 for driver in driver_set for pickup in pickup_set), name="no link from pickup to driver")
                    #m.addConstrs((x[destination,driver] == 0 for driver in driver_set for destination in dest_set), name="no link from pickup to driver")
                    m.addConstrs((z1d[driver] == 1 for driver in driver_set), name="driver is always visited by himself")

                    ############### x[i,j]+x[j,i]<=1 this is called Arc selection Valid inequalities in GTSP ######################
                    # m.addConstrs((x[i,j] +  x[j,i] <= 1 for i in pickup_and_dest_set for j in pickup_and_dest_set if i!=j), name="arc selection") #211 349 65%
                    m.addConstrs((x[i,j] +  x[j,i] <= z1d[i] for i in pickup_and_dest_set for j in pickup_and_dest_set if i!=j), name="arc selection") #218 333 52%
                    # Arcs imply both endpoints are used; forbid antiparallel pair (Chatgpt sugguested)
                    # m.addConstrs((x[i, j] <= z1d[i] for i in pickup_and_dest_set for j in pickup_and_dest_set if i != j), name="arc_implies_use_i")
                    # m.addConstrs((x[i, j] <= z1d[j] for i in pickup_and_dest_set for j in pickup_and_dest_set if i != j), name="arc_implies_use_j")
                    # m.addConstrs((x[i, j] + x[j, i] <= 1 for i in pickup_and_dest_set for j in pickup_and_dest_set if i != j), name="no_two_way_arc")

                    
                    # ############# Sequential ordering problem inequalities SOP GTSP ####################
                    # for i in pickup_and_dest_set:
                    #     #d = p+num_pickups
                    #     #successors = [d]
                    #     successors = []
                    #     for n1 in pickup_and_dest_set:
                    #         # is is predesessor of j if i ≺ j if Ei + Ti j ≤ L j and E j + Tji > Li.
                    #         fin1 = x_UB[i,n1]*UBxIPC1[i,n1]*UBxIPC2[i,n1]
                    #         if n1!=i and fin1!=0 and (e[i] + alldelays[i] + distance[i, n1] <= l[n1]) and (e[n1] + alldelays[n1] + distance[n1,i] > l[i]):
                    #             successors.append(n1)                        
                    #     #m.addConstr( quicksum(x[i,p] for i in all_points_set if i not in successors and i!=p) == z1d[p], name="sop valid inequalities" )
                    #     if i in pickup_set:
                    #         m.addConstr( quicksum(x[n,i] for n in all_points_set if n not in successors and n!=i) >= z1d[i], name="sop valid inequalities" )
                    #     else:
                    #         # means i is a destination
                    #         m.addConstr( quicksum(x[n,i] for n in pickup_and_dest_set if n not in successors and n!=i) >= z1d[i], name="sop valid inequalities" )
                    
                    # for p in pickup_set:
                    #     d = p+num_pickups
                    #     successors = [d]
                    #     for n1 in all_points_set:
                    #         # is is predesessor of j if i ≺ j if Ei + Ti j ≤ L j and E j + Tji > Li.
                    #         if n1!=p and n1!=d and n1 not in driver_set and (e[p] + alldelays[p] + distance[p, n1] <= l[n1]) and (e[n1] + alldelays[n1] + distance[n1,p] > l[p]):
                    #             successors.append(n1)                        
                    #     #m.addConstr( quicksum(x[i,p] for i in all_points_set if i not in successors and i!=p) == z1d[p], name="sop valid inequalities" )
                    #     m.addConstr( quicksum(x[i,p] for i in all_points_set if i not in successors and i!=p) >= z1d[p], name="sop valid inequalities" )
                    
                    # for d in dest_set:
                    #     p = d-num_pickups
                    #     predecessors = [p]
                    #     for n1 in all_points_set:
                    #         # is is predesessor of j if i ≺ j if Ei + Ti j ≤ L j and E j + Tji > Li.
                    #         if n1!=p and n1!=d and n1 not in driver_set and (e[d] + alldelays[d] + distance[d, n1] > l[n1]) and (e[n1] + alldelays[n1] + distance[n1,d] <= l[d]):
                    #             predecessors.append(n1)                        
                    #     m.addConstr( quicksum(x[d,i] for i in pickup_and_dest_set if i not in predecessors and i not in driver_set and i!=d) <= z1d[d], name="sop valid inequalities" )
                    
                    # # 2PC constraint 
                    # for i,j in cart_prod_pickups_destinations: #pickups
                    #     #  i and j cannot be on same path if fij=fji=0
                    #     fij=x_UB[i,j]*UBxIPC1[i,j]*UBxIPC2[i,j]#*(i in pairs.values() and j==dest_pickup_pairs[i])
                    #     fji=x_UB[j,i]*UBxIPC1[j,i]*UBxIPC2[j,i]#*(j in pairs.values() and i==dest_pickup_pairs[j])
                    #     if fij == 0 and fji == 0 and i!=j:# i and j cannot be on same route #and i in pairs.keys() and j in pairs.keys()                            
                    #         #print(f"nodes{i,j}")
                    #         #model.two_path_constraint.add(sum(model.x[n,i] + model.x[n,j] for n in model.N if n not in [i,j]) >= 2) #what comes In into set S={i,j}
                    #         #model.two_path_constraint.add( sum(model.x[i,n] + model.x[j,n] for n in model.N if n!=i and n!=j ) == 2) #what comes OUT In into set S={i,j}
                    #         if i in pickup_set:
                    #             m.addConstr( quicksum(x[i,n] for n in pickup_and_dest_set if n!=i and n!=j ) == z1d[i], name="2PC i") #what comes OUT In into set S={i,j}
                    #         else:
                    #             #m.addConstr( quicksum(x[n,i] for n in pickup_and_dest_set if n!=i and n!=j ) >= z1d[i], name="2PC i") #what comes OUT In into set S={i,j}
                    #             m.addConstr( quicksum(x[i,n] for n in pickup_and_dest_set if n!=i and n!=j ) <= z1d[i], name="2PC i") #what comes OUT In into set S={i,j}
                    #         if j in pickup_set:
                    #             m.addConstr( quicksum(x[j,n] for n in pickup_and_dest_set if n!=i and n!=j ) == z1d[j], name="2PC j") #what comes OUT In into set S={i,j}
                    #         else:
                    #             #m.addConstr( quicksum(x[n,j] for n in pickup_and_dest_set if n!=i and n!=j ) >= z1d[j], name="2PC j") #what comes OUT In into set S={i,j}
                    #             m.addConstr( quicksum(x[j,n] for n in pickup_and_dest_set if n!=i and n!=j ) <= z1d[j], name="2PC j") #what comes OUT In into set S={i,j}
                                
                    # for i,j in cart_prod_pickups_destinations: #pickups
                    #     #  i and j cannot be on same path if fij=fji=0
                    #     fij=x_UB[i,j]*UBxIPC1[i,j]*UBxIPC2[i,j]#*(i in pairs.values() and j==dest_pickup_pairs[i])
                    #     fji=x_UB[j,i]*UBxIPC1[j,i]*UBxIPC2[j,i]#*(j in pairs.values() and i==dest_pickup_pairs[j])
                    #     if fij == 0 and fji == 0 and i!=j:# i and j cannot be on same route #and i in pairs.keys() and j in pairs.keys()                            
                    #         #print(f"nodes{i,j}")
                    #         #model.two_path_constraint.add(sum(model.x[n,i] + model.x[n,j] for n in model.N if n not in [i,j]) >= 2) #what comes In into set S={i,j}
                    #         #model.two_path_constraint.add( sum(model.x[i,n] + model.x[j,n] for n in model.N if n!=i and n!=j ) == 2) #what comes OUT In into set S={i,j}
                    #         m.addConstr( quicksum(x[i,n] for n in pickup_and_dest_set if n!=i and n!=j ) <= z1d[i], name="2PC i") #what comes OUT In into set S={i,j}
                    #         m.addConstr( quicksum(x[j,n] for n in pickup_and_dest_set if n!=i and n!=j ) <= z1d[j], name="2PC j") #what comes OUT In into set S={i,j}
                        
                        
                    # changes for pickup and then first deliver with no links between 
                    #m.addConstrs((x[p1,p2] == 0 for p1 in pickup_set for p2 in pickup_set),name="no link btw pickup nodes")
                    #m.addConstrs((x[d1,d2] == 0 for d1 in dest_set for d2 in dest_set),name="no link btw destinations")
                    #m.addConstrs((x[p1,p1+num_pickups] >= z[p1,driver] for p1 in pickup_set for driver in driver_set),name="if pickup is visited then go directly from pick to its destination")

                    #limiting number of orders per drivers
                    #m.addConstrs((quicksum(z[pickup,driver] for pickup in pickup_set)<=max_ord_per_driv for driver in driver_set), name="feasibility of driver to a node")                            

                    # limiting the length xp1p2+xp2p3<=1
                    #m.addConstrs((x[p1,p2] + x[p2,p3] <= 1 for p1 in pickup_set for p2 in pickup_set for p3 in pickup_set),name="no 2 links consecutive in a pickup")
                    #limiting xp2d2+xp2d1>=xp1p2
                    #m.addConstrs((x[p2,p1+num_pickups] + x[p2,p2+num_pickups] >= x[p1,p2] for p1 in pickup_set for p2 in pickup_set),name="no link btw pickup nodes")

                    #m.update()

                    # Total hourly cost paid for committed drivers for commitment to work during the 'blocks'; these will be constant in the second stage as they are decided on in the first stage. We include them here to have full picture on cost  
                    c_com_hour_obj = c_com_h * quicksum(origl[driver] - orige[driver] for driver in com_driver_set) / 60
                    # l and e are in minutes so it should be scaled

                    # Total cost paid to committed drivers for travelled distance 
                    c_com_dist_obj = c_com_d * quicksum(w[i,driver] for i in pickup_and_dest_set for driver in com_driver_set)
                    # For a committed driver we need to add the distance from their initial location to the first pickup point; this makes sense as the platform gives advice on initial drivers location in the first stage (platform decision)
                    c_com_dist_obj = c_com_dist_obj + c_com_d * quicksum(distance[driver,i] * x[driver,i] for i in pickup_set for driver in com_driver_set)

                    # Total cost paid to occasional drivers for travelled distance; here we don't include the cost from initial location to first pickup as the driver is free to start wherever he/she wants (driver decision)  
                    c_occ_dist_obj = c_occ_d * quicksum(w[i,driver] for i in pickup_and_dest_set for driver in occ_driver_set)

                    # Total fixed cost paid to occasional drivers for the 'orders' served
                    c_occ_order_obj = c_occ_o * quicksum(z[i,driver] for i in pickup_set for driver in occ_driver_set)

                    # Total revenue generated from serving the orders
                    #NOTE: With the removal of the v-variable constraints, and the new constraints added above, both of these objectives achieve the same results as the x and z variables are perfectly connected (or so I hope)
                    rev_order_obj = rev_order * quicksum(x[i,j] for i in pickup_set for j in pickup_and_dest_set)
                    #rev_order_obj = rev_order * quicksum(z[i,driver] for i in pickup_set for driver in driver_set)
                    #rev_order_obj = rev_order * quicksum(z[i,driver] for i in dest_set for driver in driver_set)

                    ### OBJECTIVE FUNCTION
                    obj = rev_order_obj - (c_com_hour_obj + c_com_dist_obj +  c_occ_dist_obj + c_occ_order_obj) # +  quicksum(distance[i,j] * x[i,j] for (i,j) in cart_prod)
                    #obj =  quicksum(distance[i,j] * x[i,j] for (i,j) in cart_prod)

                    m.setObjective(obj, GRB.MAXIMIZE)

                    #def mycallback(model, where):
                    #    if where == GRB.Callback.MIPSOL:
                    #        sol = model.getVars("obj")
                    #    if  sol < LB_obj:
                    #        model.cbLazy(sol >= LB_obj)            
                    #m._vars =  m.getVars()                    
                    #m.optimize(mycallback)                    
                    #m.cbLazy(m.getVarByName("obj") >= LB_obj)

                    m.update()

                    m.optimize()
        

                    # If it is infeasible for this run, do some stuff
                    if m.status == GRB.Status.INFEASIBLE:
                        num_infeasible_scenarios += 1
                        m.computeIIS()
                        m.write("model.ilp")
                        print("Model Infeasible")
                        #print(annot)
                        continue
                    else:
                        objective_vals.append(m.getObjective().getValue())
                        runtime_vals.append(m.Runtime)
                        ubound_vals.append(m.ObjBound)

                    fig, ax = plt.subplots()

                    #ax.scatter(xa[0:num_points-1], ya[0:num_points-1])
                    ax.set_xlim([-1, max(xa[0:num_points])+1])
                    ax.set_ylim([-20, max(ya[0:num_points])+1])
                    ax.scatter(xa[0:num_com_drivers], ya[0:num_com_drivers],marker='^',label="Committed drivers")
                    ax.scatter(xa[num_com_drivers:num_drivers], ya[num_com_drivers:num_drivers],marker='o',label="Occasional drivers")
                    ax.scatter(xa[num_drivers:num_points], ya[num_drivers:num_points],color='red',marker='s',label="Pickup and destination")
                    ax.legend(loc='lower left')
                    #ax.grid(True)
                    #colorVal = scalarMap.to_rgba(num_points-1)

                    cmap = plt.get_cmap('prism')
                    colors = [cmap(i) for i in np.linspace(0, 1, num_points)]

                    # Get solution of z-variables and filter out the zero values, then format it as a dict with keys being the points, and values being the driver that visits them (for use below in the loop)
                    z_sol_temp = dict(filter(lambda v: v[1].X >= 1.0, z.items()))
                    z_sol = {k[0]: k[1] for (k, v) in z_sol_temp.items()}
                    
                    totx = 0                   
                    for i in pickup_and_dest_set:                                                                                               
                        for d in driver_set:
                            z_sol = m.getVarByName("z["+str(i)+","+str(d)+"]")
                            #print(f'z[{i},{d}]={z_sol.X}')
                    
                    for i in all_points_set:
                        c_sol = m.getVarByName("c["+str(i)+"]")
                        #v_sol = m.getVarByName("v["+str(i)+"]")
                                                
                        for j in all_points_set:
                            x_sol = m.getVarByName("x["+str(i)+","+str(j)+"]")
                            #print(f'x[{i},{j}]={x_sol.X}')
                            totx +=x_sol.X
                            if abs(x_sol.X - 1) <= 10e-4:    
                                u = allpoints[i]
                                vx = allpoints[j][0] - allpoints[i][0]
                                vy = allpoints[j][1] - allpoints[i][1]
                                if abs(vx) + abs(vy) >= 10e-4:
                                    ax.arrow(u[0], u[1], vx, vy, head_width=0.25, length_includes_head=False, color='blue') #colors[int(z_sol[j])])
                                    #ax.arrow(u[0], u[1], vx, vy, head_width=0.25, length_includes_head=False, color=colors[int(v_sol.X)])
                                # (vix.X/num_requests,0.5,0.25,0.45))  #(v.X/2,0.5,0.5,0.5)
                                #shape='full', lw=0, length_includes_head=False, head_width=arwidth
                        #print(f'Sum of X is {totx}')
                        ax.annotate(annot[i], (xa[i], ya[i]), color='blue')
                        # ax.annotate(round(c_sol.X), (xa[i]-1, ya[i]), color='red')
                        # ax.annotate(round(z_sol[i]), (xa[i]-2, ya[i]), color='green')

                    fig.savefig('Plots/solution_1st%s_2nd%s_Trial.pdf' % (first_stage_run, second_stage_scenario))

                    ### KPIs
                    nbr_served_orders = 0 # total nbr of served orders in the planning time T 
                    nbr_served_orders_occ = 0 # total nbr of served orders assigned to occasional drivers in the planning time T 
                    nbr_com_drivers_utilized = 0 # total nbr of committed drivers utilized (at least with one order assigned) in the planning time T 
                    nbr_occ_drivers_utilized = 0 # total nbr of occasional drivers utilized (at least with one order assigned) in the planning time T 
                    avg_utilisation_com_driver = 0 # average utilization of a committed drivers
                    work_balance = 0 #To be defined

                    # nbr_served_orders is computed using z[pickup,driver]
                    for pickup in pickup_set:
                        for driver in driver_set:
                            z_sol = m.getVarByName("z["+str(pickup)+","+str(driver)+"]")
                            nbr_served_orders += z_sol.X

                    #print("Service Level SL: ", nbr_served_orders / num_requests)
                    SL_vals.append(nbr_served_orders / num_requests)
                    
                    # nbr_served_orders_occ is computed using z[pickup,occ_driver]
                    for pickup in pickup_set:
                        for driver in occ_driver_set:
                            z_sol = m.getVarByName("z["+str(pickup)+","+str(driver)+"]")
                            nbr_served_orders_occ += z_sol.X

                    #print("SL due to occasional drivers: ", nbr_served_orders_occ / num_requests)
                    SL_occ_vals.append(nbr_served_orders_occ / num_requests)
                    
                    #nbr_com_drivers_utilized is computed using z[pickup,com_driver]
                    for driver in com_driver_set:
                        for pickup in pickup_set:
                            z_sol = m.getVarByName("z["+str(pickup)+","+str(driver)+"]")
                            nbr_com_drivers_utilized +=  z_sol.X
                            if  abs(z_sol.X-1)<=10e-4:
                                break

                    #print("Fraction of com drivers utilized: ", round(nbr_com_drivers_utilized / max_num_com_drivers, 3))                    
                    if max_num_com_drivers>0:
                        Frac_com_drv_used_vals.append(nbr_com_drivers_utilized / max_num_com_drivers )
                    else:
                        Frac_com_drv_used_vals.append(0)
                    
                    #nbr_occ_drivers_utilized is computed using z[pickup,occ_driver]
                    for driver in occ_driver_set:
                        for pickup in pickup_set:
                            z_sol = m.getVarByName("z["+str(pickup)+","+str(driver)+"]")
                            nbr_occ_drivers_utilized +=  z_sol.X
                            if abs(z_sol.X - 1) <= 10e-4:
                                break

                    #print("Fraction of occ drivers utilized: ", round(nbr_occ_drivers_utilized / num_occ_drivers,3))
                    if num_occ_drivers>0:
                        Frac_occ_drv_used_vals.append(nbr_occ_drivers_utilized / num_occ_drivers)
                    else:
                        Frac_occ_drv_used_vals.append(0)                    
                                        
                    #TODO: Missing work balance. We need to define it                                                        
                    
                    #if the time limit is reached then we don't have to rerun the model again as it will produce the same results
                    if m.Runtime>=TimeLim:
                        break                   
                    
                    #END 2nd stage loop
                    
                num_infeasible_scenarios_list.append(num_infeasible_scenarios)
                objective_vals_list.append(np.average(objective_vals))
                runtime_vals_list.append(np.average(runtime_vals))
                ubound_vals_list.append(np.average(ubound_vals))
                AGAP = np.average( 100-100*np.array(objective_vals)/np.array(ubound_vals) )
                SL_vals_list.append(np.average(SL_vals))
                SL_occ_vals_list.append(np.average(SL_occ_vals))
                Frac_com_drv_used_vals_list.append(np.average(Frac_com_drv_used_vals))
                Frac_occ_drv_used_vals_list.append(np.average(Frac_occ_drv_used_vals))                
                
                if objective_vals_list[-1] > best_average:
                    best_matrix = driver_availability_matrix
                    best_num_blocks_to_offer = num_blocks_offered_per_timeslot
                    best_average = objective_vals_list[-1]
                
                #End frist stage loop

            #Print MIP model stats that is the same for all 2nd stage loops as the num of request and drivers are same
            #m.printStats()
            OutputModel =str(m.getVars)
            OutputModel = OutputModel.replace("<bound method Model.getVars of <gurobi.Model MIP instance Unnamed: ",'')
            OutputModel = OutputModel.replace("Parameter changes:",'')
            OutputModel = OutputModel.replace("=",'')
            OutputModel = OutputModel.replace(">>",'')
            OutputModel = OutputModel.replace(" constrs,",'')
            OutputModel = OutputModel.replace(" vars,",'')
            OutputModel = OutputModel.replace(" TimeLimit",'')
            OutputModel = OutputModel.replace(", LogToConsole0",'')
            
            print("Arc_Slection_TW2_Valid_Ineq",nbr_req, nbr_cdr, nbr_odr, runtime_vals_list[-1], objective_vals_list[-1], AGAP, SL_vals_list[-1], SL_occ_vals_list[-1], Frac_com_drv_used_vals_list[-1], Frac_occ_drv_used_vals_list[-1], best_num_blocks_to_offer, OutputModel,file=out_file)
            print(best_matrix,best_num_blocks_to_offer,best_average,m.Runtime)
            LB_obj = objective_vals_list[-1] 
            
            out_file.close()