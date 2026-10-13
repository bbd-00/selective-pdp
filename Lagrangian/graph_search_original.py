"""
GraphSearch algorithm for solving single-vehicle SPDPTW subproblems.
Based on the provided pseudocode.
"""

import math
from typing import Dict, List, Tuple, Set, Optional


class GraphSearchSolver:
    def __init__(self, instance_data, vehicle_k, z_UB, multipliers, verbose=False):
        """
        Initialize GraphSearch solver for a single vehicle subproblem.
        
        Args:
            instance_data: Dictionary containing all instance data
            vehicle_k: The vehicle index to solve for
            multipliers: Dictionary {pickup_id: multiplier_value}
            verbose: Whether to print debug info
        """
        self.instance = instance_data
        self.k = vehicle_k
        self.u = multipliers
        self.verbose = verbose
        
        # Extract instance parameters
        self.num_requests = len(instance_data["requests"])
        self.n = self.num_requests
        
        # Build node sets
        self.P = list(range(1, self.num_requests + 1))
        self.D = [i + self.num_requests for i in self.P]
        
        # Extract vehicle data
        num_comm = len(instance_data["committed"])
        all_drivers = instance_data["committed"] + instance_data["occasional"]
        self.driver = all_drivers[vehicle_k]

        # Extract z_UB
        self.z_UB = z_UB
        
        # Determine if committed or occasional
        self.is_committed = (vehicle_k < num_comm)
        
        # Extract costs and parameters
        b = int(instance_data["b"])
        if self.is_committed:
            self.wC = instance_data["c_com_h"] / b
            self.deltaC = instance_data["c_com_d"]
        else:
            self.cO = instance_data["c_occ_o"]
            self.deltaO = instance_data["c_occ_d"]
        
        self.rev = instance_data["rev_order"]
        
        # Build coordinates and distance matrices
        self._build_geometry()
        
        # Build time windows and other constraints
        self._build_constraints()
        
        # Best solution tracking
        self.best_profit = float('-inf')
        self.best_route = None
        
    def _build_geometry(self):
        """Build coordinate dictionary and distance matrices."""
        self.coords = {}
        for req in self.instance["requests"]:
            pid = req["id"]
            self.coords[pid] = req["pickup"]
            self.coords[pid + self.num_requests] = req["dest"]
        
        # Depot location
        self.depot_loc = self.driver["location"]
        
        # Distance function
        def dist(loc1, loc2):
            return math.dist(loc1, loc2)
        
        # Build distance dictionaries
        self.d_ij = {}
        for i in self.P + self.D:
            for j in self.P + self.D:
                if i != j:
                    self.d_ij[i, j] = dist(self.coords[i], self.coords[j])
        
        # Depot to pickup distances
        self.d_0j = {}
        for j in self.P:
            self.d_0j[j] = dist(self.depot_loc, self.coords[j])
    
    def _build_constraints(self):
        """Build time windows, service times, demands, and capacity."""
        self.e = {}  # Early time windows
        self.l = {}  # Late time windows
        self.s = {}  # Service times
        self.q = {}  # Demands
        
        for req in self.instance["requests"]:
            pid = req["id"]
            did = pid + self.num_requests
            
            # Pickup node
            self.e[pid] = req["e_pickup"]
            self.l[pid] = req["l_pickup"]
            self.s[pid] = req["delay_p"]
            self.q[pid] = req["l_dem"]
            
            # Delivery node
            self.e[did] = req["e_dest"]
            self.l[did] = req["l_dest"]
            self.s[did] = req["delay_d"]
            self.q[did] = -req["l_dem"]
        
        # Vehicle constraints
        self.E_k = self.driver["e"]
        self.L_k = self.driver["l"]
        self.Q_k = self.driver["cap"]
    
    def _check_feasibility(self, P_tilde, R_tilde, D_tilde):
        """
        Check if current partial route violates constraints.
        Returns True if feasible, False otherwise.

        Checks (modified):
        - Time windows based on END of service:
            end_service_i = max(arrival_i, e_i) + s_i <= l_i
        - Vehicle horizon: end time at last node must be <= L_k
        - Capacity (Eq. 12)
        """
        route = list(R_tilde)
        if not route:
            return True

        current_time = self.E_k
        current_load = 0

        # ---- Depot -> first node ----
        first_node = route[0]
        travel_time = self.d_0j.get(first_node, 0.0)
        arrival_time = current_time + travel_time

        start_service = max(arrival_time, self.e[first_node] - self.s[first_node])
        end_service = start_service + self.s[first_node]

        # TW check based on end of service
        if end_service > self.l[first_node]:
            return False

        current_time = end_service
        current_load += self.q[first_node]

        if current_load > self.Q_k or current_load < 0:
            return False

        # ---- Between consecutive nodes ----
        for idx in range(len(route) - 1):
            from_node = route[idx]
            to_node = route[idx + 1]

            travel_time = self.d_ij.get((from_node, to_node), 0.0)
            arrival_time = current_time + travel_time

            start_service = max(arrival_time, self.e[to_node] - self.s[to_node])
            end_service = start_service + self.s[to_node]

            # TW check based on end of service
            if end_service > self.l[to_node]:
                return False

            current_time = end_service
            current_load += self.q[to_node]

            if current_load > self.Q_k or current_load < 0:
                return False

        # ---- Vehicle horizon ----
        if current_time > self.L_k:
            return False

        return True
    
    def _compute_route_profit(self, route):
        """
        Compute objective function for a given route.
        Π̃ = Σ(p_i - u_i) for served pickups - vehicle costs
        """
        if not route:
            if self.is_committed:
                return - self.wC * (self.L_k - self.E_k)
            else:
                return 0.0
        
        # Revenue from served requests
        profit = 0.0
        served_pickups = [node for node in route if node in self.P]
        
        for pickup in served_pickups:
            profit += self.rev - self.u.get(pickup, 0.0)
        
        # Subtract vehicle costs
        if self.is_committed:
            # Committed driver cost: wC * (L_k - E_k) + deltaC * distance
            profit -= self.wC * (self.L_k - self.E_k)
            
            # Calculate total distance
            total_dist = self.d_0j.get(route[0], 0)
            for i in range(len(route) - 1):
                total_dist += self.d_ij.get((route[i], route[i + 1]), 0)
            
            profit -= self.deltaC * total_dist
        else:
            # Occasional driver cost: cO * #requests + deltaO * pickup-to-delivery distance
            profit -= self.cO * len(served_pickups)
            
            # Calculate pickup-to-delivery distances
            pd_dist = 0.0
            for pickup in served_pickups:
                delivery = pickup + self.num_requests
                # Find positions in route
                try:
                    p_idx = route.index(pickup)
                    d_idx = route.index(delivery)
                    
                    # Sum distances from pickup to delivery
                    for j in range(p_idx, d_idx):
                        pd_dist += self.d_ij.get((route[j], route[j + 1]), 0)
                except ValueError:
                    pass  # Should not happen in valid route
            
            profit -= self.deltaO * pd_dist
        
        return profit
    
    def _graph_search(self, P_tilde, R_tilde, D_tilde, k, vehicle):
        """
        Recursive GraphSearch function following the pseudocode.
        
        Args:
            P_tilde: Set of unassigned pickup nodes
            R_tilde: Current route (list of nodes)
            D_tilde: Set of unserved delivery nodes
            k: Remaining vehicles (for this single-vehicle problem, always 0 or 1)
        """
        # Line 1-3: Base case - all vehicles processed
        if k == 0:
            return
        
        # Line 4-6: Check feasibility
        if not self._check_feasibility(P_tilde, R_tilde, D_tilde):
            return
        
        # Line 7-10: Compute current profit and check bound
        current_profit = self._compute_route_profit(R_tilde)
        
        # Compute upper bound: current profit + max possible from remaining requests
        remaining_profit = sum(max(self.rev - self.u.get(p, 0.0), 0) for p in P_tilde if self.z_UB[p,k] == 1)
        upper_bound = current_profit + remaining_profit
        
        if upper_bound < self.best_profit:
            return  # Pruning
        
        # Line 11-20: Branch on each unassigned node
        nodes_to_try = list(P_tilde) + list(D_tilde)
        
        for i in nodes_to_try:
            if i in P_tilde:
                # Try adding pickup i
                P_new = P_tilde - {i}
                R_new = R_tilde + [i]
                D_new = D_tilde | {i + self.n}
                
                self._graph_search(P_new, R_new, D_new, k, self.k)
                
                # Backtrack (implicit in recursive calls)
                
            else:  # i in D_tilde
                # Lines 16-20: Try adding delivery i
                R_new = R_tilde + [i]
                D_new = D_tilde - {i}
                
                self._graph_search(P_tilde, R_new, D_new, k, self.k)
        
        # Line 21-27: Check if all deliveries done (complete route)
        if len(D_tilde) == 0:
            route_profit = self._compute_route_profit(R_tilde)
            
            if route_profit > self.best_profit:
                self.best_profit = route_profit
                self.best_route = list(R_tilde)
                
                if self.verbose:
                    print(f"  [GraphSearch] New best: profit={route_profit:.4f}, route={R_tilde}")
    
    def solve(self):
        """
        Solve the single-vehicle subproblem using GraphSearch.
        
        Returns:
            Dictionary with 'objVal' and 'route'
        """
        # Initialize search
        P_initial = set(self.P)
        R_initial = []
        D_initial = set()
        k_initial = 1  # Single vehicle
        
        self.best_profit = float('-inf')
        self.best_route = None
        
        if self.verbose:
            print(f"[GraphSearch] Starting search for vehicle {self.k}")
        
        # Run search
        self._graph_search(P_initial, R_initial, D_initial, k_initial, self.k)
        
        # If no route found, return empty route with zero profit
        if self.best_route is None:
            # Calculate fixed cost for committed driver even with empty route
            if self.is_committed:
                self.best_profit = -self.wC * (self.L_k - self.E_k)
            else:
                self.best_profit = 0.0
            self.best_route = []
        
        if self.verbose:
            print(f"[GraphSearch] Final: profit={self.best_profit:.4f}, route={self.best_route}")
        
        return {
            'objVal': self.best_profit,
            'route': self.best_route,
            'served_pickups': [node for node in self.best_route if node in self.P]
        }
