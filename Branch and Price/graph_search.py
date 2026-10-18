"""
GraphSearch algorithm for solving single-vehicle SPDPTW subproblems.
Based on the provided pseudocode.
"""

import math
from typing import Dict, List, Tuple, Set, Optional

from pricing_bounds import TimeBudgetBound


class GraphSearchSolver:
    def __init__(self, instance_data, vehicle_k, z_UB, x_UB, x_ind, multipliers, verbose=False):
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

        # Extract z_UB and A
        self.z_UB = z_UB
        self.x_ind = x_ind
        self.x_UB = x_UB
        
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
        
        self.rev = {req["id"]: req["rev"] for req in instance_data["requests"]}   # per-request revenue
        
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
            profit += self.rev[pickup] - self.u.get(pickup, 0.0)
        
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
            profit -= self.cO * len(served_pickups)
            a2_dist = sum(
                self.d_ij.get((route[j], route[j + 1]), 0.0)
                for j in range(len(route) - 1)
            )
            profit -= self.deltaO * a2_dist
        
        return profit

    def _compute_current_state(self, R_tilde):
        """
        Walk the current partial route and return (current_time, last_node).
        last_node is None if the route is empty.
        """
        if not R_tilde:
            return self.E_k, None

        current_time = self.E_k
        first_node = R_tilde[0]
        current_time += self.d_0j.get(first_node, 0.0)
        current_time = max(current_time, self.e[first_node] - self.s[first_node])
        current_time += self.s[first_node]

        for idx in range(len(R_tilde) - 1):
            from_node = R_tilde[idx]
            to_node = R_tilde[idx + 1]
            current_time += self.d_ij.get((from_node, to_node), 0.0)
            current_time = max(current_time, self.e[to_node] - self.s[to_node])
            current_time += self.s[to_node]

        return current_time, R_tilde[-1]

    def _can_complete_request(self, pickup, current_time, last_node):
        delivery = pickup + self.n

        # Travel to pickup
        if last_node is None:
            travel_to_pickup = self.d_0j[pickup]
        else:
            travel_to_pickup = self.d_ij.get((last_node, pickup), 0.0)

        arrival_pickup = current_time + travel_to_pickup
        end_pickup = max(arrival_pickup, self.e[pickup] - self.s[pickup]) + self.s[pickup]

        if end_pickup > self.l[pickup]:
            return False

        # Travel to delivery
        travel_to_delivery = self.d_ij.get((pickup, delivery), 0.0)
        arrival_delivery = end_pickup + travel_to_delivery
        end_delivery = max(arrival_delivery, self.e[delivery] - self.s[delivery]) + self.s[delivery]

        # Delivery must meet its own time window and the vehicle horizon (same test as labeling)
        return end_delivery <= self.l[delivery] and end_delivery <= self.L_k

    def _can_reach_delivery(self, delivery, current_time, last_node):
        """True if driving straight from last_node to an on-board request's delivery
        still meets its time window and the vehicle horizon."""
        arrival = current_time + self.d_ij.get((last_node, delivery), 0.0)
        end_delivery = max(arrival, self.e[delivery] - self.s[delivery]) + self.s[delivery]
        return end_delivery <= self.l[delivery] and end_delivery <= self.L_k

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
        
        # NEW: Use current time and position to filter reachable pickups
        current_time, last_node = self._compute_current_state(R_tilde)

        # NEW: Dead branch — if some on-board request can't be delivered in time even by driving
        # straight to it, no extension of this partial route can be completed, so stop here.
        if any(not self._can_reach_delivery(d, current_time, last_node) for d in D_tilde):
            return

        # NEW: Permanently drop pickups that can no longer be completed in time. Time only moves
        # forward along a branch and travel times obey the triangle inequality, so a pickup that
        # fails here fails at every deeper step; the filtered list is what gets passed down.
        P_tilde = [p for p in P_tilde if self._can_complete_request(p, current_time, last_node)]

        # Compute upper bound: current profit + max possible from remaining requests
        # NEW: Only requests still reachable from the last node count, and only as many as fit
        # in the time left before L_k (time-budget bound, shared with labeling; see pricing_bounds)
        reachable = {p for p in P_tilde
                     if not R_tilde or self.x_UB.get((R_tilde[-1], p), 0) == 1}   # NEW
        remaining_profit = self.time_bound.remaining_gain(
            current_time, [d - self.n for d in D_tilde], reachable.__contains__)

        upper_bound = current_profit + remaining_profit

        self.n_bound_checks += 1
        # -inf means the on-board deliveries alone can't fit before L_k: no completion exists
        if math.isinf(remaining_profit) or upper_bound < self.best_profit:
            self.n_bound_pruned += 1
            return  # Pruning
        
        # Line 11-20: Branch on each unassigned node
        nodes_to_try = list(P_tilde) + list(D_tilde)

        # NEW: Branch on each unassigned node if it makes sense from a filtering perspective
        if not R_tilde:
            # P_tilde already holds only pickups that can be completed in time (filtered above)
            nodes_to_try = [
                p for p in P_tilde
                if self.z_UB.get((p, self.k), 0) == 1                       # NEW
            ]
        else:
            last = R_tilde[-1]
            nodes_to_try = [
                i for i in list(P_tilde) + list(D_tilde)
                if self.x_ind.get((last, i), 0) == 1                        # NEW
            ]
        
        for i in nodes_to_try:
            if i in P_tilde:
                # Try adding pickup i
                P_new = [p for p in P_tilde if p != i]
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

    def _tw_compatibility_key(self, pickup_id):
        """
        Sort key for ordering pickups by time-window compatibility with this courier.

        We want pickups that the courier can reach earliest (and with the least
        waiting) to come first, so the branch-and-bound tree locks in a good
        incumbent quickly and prunes more aggressively thereafter.

        Key components (all ascending — lower = more compatible):
          1. earliest_start : max(E_k + d_0j, e[p] - s[p])
                              — soonest service can actually begin at p given
                                depot departure and the pickup's own time window
          2. urgency_slack  : l[p] - (earliest_start + s[p])
                              — remaining slack after earliest possible service;
                                tighter windows constrain sequencing more, so
                                explore them sooner to surface infeasibility early
          3. e[p]           : raw earliest pickup time as a tiebreaker
        """
        earliest_arrival = self.E_k + self.d_0j[pickup_id]
        earliest_start   = max(earliest_arrival, self.e[pickup_id] - self.s[pickup_id])
        end_service      = earliest_start + self.s[pickup_id]
        urgency_slack    = self.l[pickup_id] - end_service   # smaller = tighter window
        return (earliest_start, urgency_slack, self.e[pickup_id])

    def _profit_then_time_key(self, pickup_id):
        marginal_profit = self.rev[pickup_id] - self.u.get(pickup_id, 0.0)
        earliest_arrival = self.E_k + self.d_0j[pickup_id]
        end_service = max(earliest_arrival + self.s[pickup_id], self.e[pickup_id])
        return (-marginal_profit, end_service)  # negate profit so higher profit sorts first

    def _time_then_profit_key(self, pickup_id):
        marginal_profit = self.rev[pickup_id] - self.u.get(pickup_id, 0.0)
        earliest_arrival = self.E_k + self.d_0j[pickup_id]
        end_service = max(earliest_arrival + self.s[pickup_id], self.e[pickup_id])
        return (end_service, -marginal_profit)

    def solve(self):
        """
        Solve the single-vehicle subproblem using GraphSearch.
        
        Returns:
            Dictionary with 'objVal' and 'route'
        """
        # --- Filter: only consider requests that are feasible for this vehicle based on z_UB ---
        feasible_pickups = [p for p in self.P if self.z_UB.get((p, self.k), 0) == 1]
        # feasible_pickups = [p for p in self.P]

        # Reduced gain per request, including the per-order fee for occasional couriers
        # (the same gains the labeling solver uses).
        req_cost = 0.0 if self.is_committed else self.cO
        gain = {p: self.rev[p] - self.u.get(p, 0.0) - req_cost for p in feasible_pickups}

        # NEW: Dual-based elimination: a request with gain <= 0 never improves a route (removing
        # it never adds travel or delays later stops, by the triangle inequality), so some optimal
        # route skips it. Exact for this pricing call; recomputed every call as the duals change.
        self.n_eligible = len(feasible_pickups)
        feasible_pickups = [p for p in feasible_pickups if gain[p] > 0]
        self.n_excluded = self.n_eligible - len(feasible_pickups)

        # --- Order: most TW-compatible requests first for stronger early pruning ---
        # Requests the courier can reach earliest (and whose windows are tightest)
        # are explored first so the tree finds a good incumbent quickly, allowing
        # subsequent branches to be cut earlier.
        feasible_pickups.sort(key=self._tw_compatibility_key)

        P_initial = feasible_pickups
        R_initial = []
        D_initial = set()
        k_initial = 1  # Single vehicle

        self.time_bound = TimeBudgetBound(feasible_pickups, self.n, gain, self.s,
                                          self.d_0j, self.d_ij, self.x_ind, self.L_k)

        self.best_profit = float('-inf')
        self.best_route = None
        self.n_bound_checks = 0   # partial routes whose bound was evaluated
        self.n_bound_pruned = 0   # of those, partial routes cut because the bound couldn't beat the best
        
        if self.verbose:
            print(f"[GraphSearch] Vehicle {self.k}: {len(feasible_pickups)}/{len(self.P)} "
                  f"requests eligible after z_UB filter")
            print(f"[GraphSearch] Exploration order: {feasible_pickups}")
        
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
            'served_pickups': [node for node in self.best_route if node in self.P],
            'n_bound_checks': self.n_bound_checks,
            'n_bound_pruned': self.n_bound_pruned,
            'n_eligible': self.n_eligible,
            'n_excluded': self.n_excluded,
        }