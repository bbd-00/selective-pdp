"""
Time-budget bound shared by the GraphSearch and labeling pricing solvers.

Every stop still to be visited needs at least its cheapest allowed entry travel
time plus its service time; stops are served one after another (waiting only adds
time) and the last one must end by the courier's horizon L_k. So, from a partial
route that ends service at time t, the requests served later must satisfy

    Σ tau_p  <=  L_k − t − Σ_{on-board i} (s_{n+i} + m_{n+i}),

with tau_p = s_p + m_p + s_{n+p} + m_{n+p} and m_j the shortest allowed arc into j.
The best total gain under that time budget is bounded by the fractional knapsack,
filled greedily by gain per minute (Dantzig's rule), which is never below the true
optimum. Travel time equals Euclidean distance, as in both solvers.
"""

import math


class TimeBudgetBound:
    def __init__(self, pickups, n, gain, service, start_dist, dist, x_ind, L_k):
        """
        pickups    : eligible pickups for this courier (the only requests a route may serve)
        gain       : {p: reduced revenue of serving p}, e.g. rev − π_p (− cO for occasional)
        service    : {node: service time}
        start_dist : {p: travel time from the courier's start to pickup p}
        dist       : {(i, j): travel time between pickup/delivery nodes}
        x_ind      : {(i, j): 1 if arc i→j is allowed}
        """
        self.L_k = L_k
        nodes = list(pickups) + [p + n for p in pickups]

        def min_entry(j, from_start):
            # Shortest allowed arc into j from any node a route could visit before it.
            best = min((dist[i, j] for i in nodes
                        if i != j and x_ind.get((i, j), 0) == 1), default=math.inf)
            if from_start:
                best = min(best, start_dist[j])
            return best

        # Minimum time an on-board request still needs: entering its delivery and serving it.
        self.delivery_need = {p: service[p + n] + min_entry(p + n, False) for p in pickups}

        items = []
        for p in pickups:
            if gain[p] <= 0:
                continue                   # contributes nothing to an upper bound
            tau = service[p] + min_entry(p, True) + self.delivery_need[p]
            if math.isinf(tau):
                continue                   # no allowed way to serve p at all
            items.append((p, gain[p], tau))
        # Highest gain per minute first; requests needing no time go first.
        items.sort(key=lambda it: it[1] / it[2] if it[2] > 0 else math.inf, reverse=True)
        self.items = items

    def remaining_gain(self, t, open_requests, is_available):
        """Upper bound on the gain still collectable from end-of-service time t.

        open_requests : requests picked up but not yet delivered
        is_available  : predicate, True if request p can still be picked up
        Returns -inf when the on-board deliveries alone cannot fit before L_k.
        """
        budget = self.L_k - t - sum(self.delivery_need[i] for i in open_requests)
        if budget < 0:
            return -math.inf
        total = 0.0
        for p, g, tau in self.items:
            if not is_available(p):
                continue
            if tau <= budget:
                total += g
                budget -= tau
            else:
                total += g * budget / tau
                break
        return total
