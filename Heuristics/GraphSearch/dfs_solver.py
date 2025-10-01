# dfs_solver.py
from dataclasses import dataclass
from typing import List, Dict, Tuple
import math
import copy

# ----------------- Shared data classes -----------------
@dataclass
class Request:
    i: int
    pickup: int
    delivery: int
    demand: float
    profit: float

@dataclass
class NodeTW:
    earliest: float
    latest: float
    service: float = 0.0

@dataclass
class PDPInstance:
    n_requests: int
    n_vehicles: int
    capacity: float
    depot_start: int
    depot_end: int
    requests: List[Request]
    travel_time: List[List[float]]
    travel_cost: List[List[float]]
    tw: Dict[int, NodeTW]

    @property
    def n_nodes(self):
        return len(self.travel_time)

# ----------------- Depth-first heuristic -----------------
class DFSHeuristic:
    """
    Depth-first branch-and-bound for profit-maximizing PDP with selection.
    Objective: sum(profits of completed requests) - sum(travel costs).
    """
    def __init__(self, inst: PDPInstance):
        self.I = inst
        self.best_profit = -math.inf
        self.best_routes: List[List[int]] = []

    # ---- Feasibility for a partial route ----
    def _simulate_route_partial(self, route: List[int]) -> Tuple[bool, float, float]:
        if not route:
            # start at depot at its earliest
            return True, 0.0, max(self.I.tw[self.I.depot_start].earliest, 0.0)

        load = 0.0
        t = max(self.I.tw[self.I.depot_start].earliest, 0.0)
        cost = 0.0
        prev = self.I.depot_start

        # quick maps
        pickup_of = {r.pickup: r for r in self.I.requests}
        delivery_of = {r.delivery: r for r in self.I.requests}
        seen_pickups = set()

        for node in route:
            t += self.I.travel_time[prev][node]
            cost += self.I.travel_cost[prev][node]

            tw = self.I.tw[node]
            if t < tw.earliest:
                t = tw.earliest  # wait
            if t > tw.latest + 1e-9:
                return False, cost, t

            # service
            t += tw.service

            # capacity & precedence
            if node in pickup_of:
                r = pickup_of[node]
                load += r.demand
                seen_pickups.add(r.i)
            elif node in delivery_of:
                r = delivery_of[node]
                if r.i not in seen_pickups:
                    return False, cost, t  # deliver before picking up
                load -= r.demand

            if load < -1e-9 or load - self.I.capacity > 1e-9:
                return False, cost, t

            prev = node

        return True, cost, t

    def _route_objective_if_closed(self, route: List[int]) -> float:
        feasible, cost_so_far, t_last = self._simulate_route_partial(route)
        if not feasible:
            return -math.inf

        last = self.I.depot_start if not route else route[-1]

        # travel to end depot
        t_arrive_end = t_last + self.I.travel_time[last][self.I.depot_end]

        # respect end depot window (allow waiting if early)
        end_tw = self.I.tw[self.I.depot_end]
        if t_arrive_end < end_tw.earliest:
            t_arrive_end = end_tw.earliest
        if t_arrive_end > end_tw.latest + 1e-9:
            return -math.inf

        # (service at depot usually 0, so no need to add)
        cost = cost_so_far + self.I.travel_cost[last][self.I.depot_end]

        served_nodes = set(route)
        profit = 0.0
        for r in self.I.requests:
            if r.pickup in served_nodes and r.delivery in served_nodes:
                profit += r.profit

        return profit - cost

    # ---- Simple admissible upper bound for pruning ----
    def _upper_bound(self, profit_so_far: float, route_partial: List[int],
                     remaining_pickups: List[int], available_deliveries: List[int]) -> float:
        # profit completed within partial route
        #NOTE: profit_so_far is total across all vehicles, whereas completed is for the vehicle being considered
        served = set(route_partial)
        completed = 0.0
        for r in self.I.requests:
            if r.pickup in served and r.delivery in served:
                completed += r.profit
        # optimistic: every remaining request could still be completed
        optimistic = 0.0
        for r in self.I.requests:
            if not (r.pickup in served and r.delivery in served):
                optimistic += r.profit
        return profit_so_far + completed + optimistic  # ignore future costs to stay admissible

    # ---- Public API ----
    def solve(self):
        all_pickups = [r.pickup for r in self.I.requests]
        self._dfs_vehicle(
            k=0,
            routes_so_far=[],
            profit_so_far=0.0,
            P_pending=all_pickups,
            D_available=[]
        )
        return self.best_routes, self.best_profit

    # ---- Recursive vehicle-level DFS ----
    def _dfs_vehicle(self, k: int, routes_so_far, profit_so_far,
                     P_pending: List[int], D_available: List[int]):

        if k == self.I.n_vehicles:
            # accept whatever profit we accumulated (unserved requests just contribute 0)
            if profit_so_far > self.best_profit:
                self.best_profit = profit_so_far
                self.best_routes = copy.deepcopy(routes_so_far)
            return

        current_route: List[int] = []
        visited_pickups = set()

        def backtrack(P, D):
            feasible, _, t_now = self._simulate_route_partial(current_route)
            if not feasible:
                return

            ub = self._upper_bound(profit_so_far, current_route, P, D)
            if self.best_profit >= ub - 1e-9:
                return

            # NOTE:--- NEW PRUNE THAT I ADDED: if current time is past the latest window of all remaining nodes, end the vehicle route ---
            remaining = list(P) + list(D)
            if remaining and all(t_now > self.I.tw[n].latest + 1e-9 for n in remaining):
                # Nothing else can be served in time: close this vehicle now and move on.
                contrib = self._route_objective_if_closed(current_route)
                if contrib != -math.inf:
                    routes_so_far.append(current_route.copy())
                    self._dfs_vehicle(k + 1, routes_so_far, profit_so_far + contrib, P, D)
                    routes_so_far.pop()
                return

            candidates = list(P) + list(D)
            if not candidates:
                # close this vehicle and continue
                contrib = self._route_objective_if_closed(current_route)
                if contrib == -math.inf:
                    return
                routes_so_far.append(current_route.copy())
                self._dfs_vehicle(k + 1, routes_so_far, profit_so_far + contrib, P, D)
                routes_so_far.pop()
                return

            # explore candidates depth-first
            for node in list(candidates):
                if node in P:
                    # choose pickup
                    r = next(r for r in self.I.requests if r.pickup == node)
                    current_route.append(node)
                    visited_pickups.add(r.i)
                    P2 = [x for x in P if x != node]
                    D2 = D + [r.delivery]
                    backtrack(P2, D2)
                    visited_pickups.remove(r.i)
                    current_route.pop()
                else:
                    # delivery
                    r = next(r for r in self.I.requests if r.delivery == node)
                    if r.i not in visited_pickups:
                        continue
                    current_route.append(node)
                    D2 = [x for x in D if x != node]
                    backtrack(P, D2)
                    current_route.pop()

            # option: end this vehicle early and leave remaining work to others
            contrib = self._route_objective_if_closed(current_route)
            if contrib != -math.inf:
                routes_so_far.append(current_route.copy())
                self._dfs_vehicle(k + 1, routes_so_far, profit_so_far + contrib, P, D)
                routes_so_far.pop()

        backtrack(P_pending, D_available)