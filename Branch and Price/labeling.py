"""
Labeling algorithm (ESPPRC) for the single-vehicle SPDPTW pricing subproblem.

Drop-in replacement for GraphSearchSolver: same constructor arguments and the
same solve() return dict, and it searches the same route space (z_UB filters
pickups, x_ind filters arcs, identical time-window and cost rules), so both
solvers return the same optimal objVal.

Label (one partial route from the courier's start location):
    node        : last node visited (0 = start location, 2n+1 = end depot)
    time        : end-of-service time at node
    profit      : reduced profit so far (committed fixed cost included up front)
    open        : frozenset of requests picked up but not yet delivered
    unreachable : frozenset of pickups that can no longer be taken — already
                  picked up, or ruled out by time (Feillet et al., 2004).
                  Picked-up requests are in here, which makes routes elementary.

Dominance at the same node: L1 dominates L2 if
    profit1 >= profit2, time1 <= time2, open1 ⊆ open2, unreachable1 ⊆ unreachable2.
Load is not compared: it equals Σ q over open, so open1 ⊆ open2 implies load1 <= load2.
Validity relies on Euclidean (triangle-inequality) travel times and costs.

Algorithm (solve):
    1. Start from one label at the courier's start location.
    2. Repeatedly pop the earliest-time label and skip it if it was dominated
       or its optimistic bound cannot beat the best complete route.
    3. Extend it by one node (_successors → _extend): a pickup it can still
       take, a delivery of a request it carries, or — if nothing is on board —
       the end depot 2n+1 (zero cost and time, like the MIP's A3 arcs).
    4. A child at the end depot is a complete route: record it as best if it
       is more profitable, and never extend it. For any other child: update
       unreachable (_mark_unreachable), drop it if an on-board delivery can no
       longer be made (_rroute_completable), and keep it only if no label at
       the same node dominates it (_insert).
    5. When no labels remain, rebuild the best route from parent pointers
       (the end depot itself is left out of the returned route).

Reading order: solve → _successors → _extend → _mark_unreachable / _can_complete
→ _insert / _dominates → _bound.  _serve is the shared time-window rule.
"""

import heapq
import math

from pricing_bounds import TimeBudgetBound


class _Label:
    """One partial route; parent links back to the label it was extended from.
    alive is set to False when a later label dominates it while it waits in the heap."""
    # load is a cached function of open, kept only for the capacity check.
    __slots__ = ("node", "time", "profit", "open", "unreachable", "load", "parent", "alive")

    def __init__(self, node, time, profit, open_, unreachable, load, parent):
        self.node = node
        self.time = time
        self.profit = profit
        self.open = open_
        self.unreachable = unreachable
        self.load = load
        self.parent = parent
        self.alive = True


def _dominates(a, b):
    """True if a can do everything b can from here, at least as profitably.
    Only called on labels at the same node (see _insert)."""
    return (a.profit >= b.profit
            and a.time <= b.time
            and a.open <= b.open
            and a.unreachable <= b.unreachable)


class LabelingSolver:
    def __init__(self, instance_data, vehicle_k, z_UB, x_UB, x_ind, multipliers, verbose=False):
        """Precompute courier k's cost rates, time windows, distances and the
        reduced gain of each eligible pickup (rev_p − π_p, minus cO for occasional).
        x_UB is accepted only for interface parity with GraphSearchSolver."""
        self.k = vehicle_k
        self.x_ind = x_ind
        self.verbose = verbose

        n = len(instance_data["requests"])
        self.n = n
        self.sink = 2 * n + 1

        num_comm = len(instance_data["committed"])
        driver = (instance_data["committed"] + instance_data["occasional"])[vehicle_k]
        self.is_committed = vehicle_k < num_comm
        self.E_k = driver["e"]
        self.L_k = driver["l"]
        self.Q_k = driver["cap"]

        b = int(instance_data["b"])
        if self.is_committed:
            # block cost on the original shift, even if preprocessing tightened E_k/L_k
            shift_length = driver.get("l_orig", self.L_k) - driver.get("e_orig", self.E_k)
            self.fixed_cost = instance_data["c_com_h"] / b * shift_length
            self.arc_cost = instance_data["c_com_d"]
            self.start_arc_cost = instance_data["c_com_d"]
            req_cost = 0.0
        else:
            # Occasional couriers pay deltaO on A2 arcs only; the start→first-pickup arc is free.
            self.fixed_cost = 0.0
            self.arc_cost = instance_data["c_occ_d"]
            self.start_arc_cost = 0.0
            req_cost = instance_data["c_occ_o"]

        self.e, self.l, self.s, self.q = {}, {}, {}, {}
        coords = {}
        for req in instance_data["requests"]:
            p, d = req["id"], req["id"] + n
            coords[p], coords[d] = req["pickup"], req["dest"]
            self.e[p], self.l[p], self.s[p], self.q[p] = (
                req["e_pickup"], req["l_pickup"], req["delay_p"], req["l_dem"])
            self.e[d], self.l[d], self.s[d] = req["e_dest"], req["l_dest"], req["delay_d"]

        eligible = [p for p in range(1, n + 1) if z_UB.get((p, vehicle_k), 0) == 1]
        rev = {req["id"]: req["rev"] for req in instance_data["requests"]}   # per-request revenue
        gain = {p: rev[p] - multipliers.get(p, 0.0) - req_cost for p in eligible}

        # Dual-based elimination: a request with gain <= 0 never improves a route (removing it
        # never adds travel or delays later stops, by the triangle inequality), so some optimal
        # route skips it. Exact for this pricing call; recomputed every call as the duals change.
        self.pickups = [p for p in eligible if gain[p] > 0]
        self.n_eligible = len(eligible)
        self.n_excluded = len(eligible) - len(self.pickups)
        self.gain = {p: gain[p] for p in self.pickups}

        nodes = self.pickups + [p + n for p in self.pickups]
        self.d0 = {p: math.dist(driver["location"], coords[p]) for p in self.pickups}
        self.d = {(i, j): math.dist(coords[i], coords[j])
                  for i in nodes for j in nodes if i != j}

        self.time_bound = TimeBudgetBound(self.pickups, n, self.gain, self.s,
                                          self.d0, self.d, x_ind, self.L_k)

        self.best_profit = float("-inf")
        self.best_route = None
        self.n_labels = 0
        self.n_bound_checks = 0   # live labels whose bound was evaluated
        self.n_bound_pruned = 0   # of those, labels discarded because the bound couldn't beat the best

    # ── Feasibility helpers (same time rule as GraphSearch) ─────────────────

    def _serve(self, depart, dist, j):
        """End-of-service time at j, or None if j's time window or the horizon is violated."""
        start = max(depart + dist, self.e[j] - self.s[j])
        end = start + self.s[j]
        if end > self.l[j] or end > self.L_k:
            return None
        return end

    def _dist(self, node, j):
        """Travel distance (= travel time) from node to j; node 0 is the courier's start."""
        return self.d0[j] if node == 0 else self.d[node, j]

    def _can_complete(self, p, node, t):
        """Direct node→p→p+n is the fastest way to serve p (triangle inequality),
        so if it fails, p is unreachable from this state for good."""
        end_p = self._serve(t, self._dist(node, p), p)
        return end_p is not None and self._serve(end_p, self.d[p, p + self.n], p + self.n) is not None

    def _rroute_completable(self, label):
        """False if some on-board request can't be delivered in time even directly,
        so the label can never become a complete route."""
        for i in label.open:
            if self._serve(label.time, self.d[label.node, i + self.n], i + self.n) is None:
                return False
        return True

    def _mark_unreachable(self, label):
        """Add every pickup that can no longer be served from this label's node and time."""
        newly_unreachable = {p for p in self.pickups
                             if p not in label.unreachable
                             and not self._can_complete(p, label.node, label.time)}
        if newly_unreachable:
            label.unreachable = label.unreachable | newly_unreachable

    def _bound(self, label):
        """Optimistic final profit: current profit plus the most gain the remaining requests
        could bring within the time left in the shift (time-budget bound, see pricing_bounds),
        ignoring further travel cost. If it can't beat the best route, the label is useless."""
        return label.profit + self.time_bound.remaining_gain(
            label.time, label.open, lambda p: p not in label.unreachable)

    # ── Main algorithm ──────────────────────────────────────────────────────

    def _insert(self, buckets, label):
        """Dominance check against labels stored at the same node. Returns False (discard) if an existing label dominates the new one; otherwise returns True (keep), stores it, and discards any stored labels it dominates."""
        bucket = buckets.setdefault(label.node, [])     # Getting current node bucket (empty list if no labels exist)
        for other in bucket:
            if _dominates(other, label):
                return False
        survivors = []
        for other in bucket:
            if _dominates(label, other):
                other.alive = False             # marks any dominated labels as dead (to be discarded later when popped out of the heap)
            else:
                survivors.append(other)
        survivors.append(label)
        buckets[label.node] = survivors     # collecting all nondominated labels at the node

        return True

    def _extend(self, label, j, end, is_pickup):
        """Build the child label for moving from label.node to j, finishing service at end.
        Pickup: collect its gain and mark it open + unreachable. Delivery: close the request."""
        n = self.n
        cost = (self.start_arc_cost * self.d0[j] if label.node == 0
                else self.arc_cost * self.d[label.node, j])
        if is_pickup:
            return _Label(j, end, label.profit + self.gain[j] - cost,
                          label.open | {j}, label.unreachable | {j},
                          label.load + self.q[j], label)
        i = j - n
        return _Label(j, end, label.profit - cost,
                      label.open - {i}, label.unreachable,
                      label.load - self.q[i], label)

    def _successors(self, label):
        """Yields every feasible one-node extension of label: pickups that are reachable, fit in capacity, use an allowed arc and leave their delivery
        reachable; then deliveries of on-board requests that can be made in time; then, if nothing is on board, the end depot 2n+1."""
        n = self.n
        node, t = label.node, label.time
        for p in self.pickups:
            # Only reachable pickups
            if p in label.unreachable:
                continue
            # Only allowed by x_ind (excluding the depot is just because x_ind does not include the depot)
            if node != 0 and self.x_ind.get((node, p), 0) != 1:
                continue
            # Only capacity-feasible pickups
            if label.load + self.q[p] > self.Q_k:
                continue
            # Start from t, and simulate direct travel to p, and check TW feasibility. Then, check delivery feasibility
            end = self._serve(t, self._dist(node, p), p)
            if end is None or self._serve(end, self.d[p, p + n], p + n) is None:
                continue
            yield self._extend(label, p, end, is_pickup=True)

        for i in label.open:        # only visit deliveries of open pickups
            dnode = i + n
            # If allowed
            if self.x_ind.get((node, dnode), 0) != 1:
                continue
            # Simulate feasibility of delivery from current node to destination
            end = self._serve(t, self.d[node, dnode], dnode)
            if end is not None:
                yield self._extend(label, dnode, end, is_pickup=False)

        # Go to the end depot only when every picked-up request has been delivered.
        # The arc to the depot costs nothing and takes no time.
        if not label.open:
            yield _Label(self.sink, t, label.profit, label.open, label.unreachable,
                         label.load, label)

    def solve(self):
        """Run the labeling search and return the best route."""
        # Empty route at the start location. Its end-depot child is the empty route.
        root = _Label(0, self.E_k, -self.fixed_cost, frozenset(), frozenset(), 0, None)
        self._mark_unreachable(root)

        best = None                        # end-depot label of the best complete route
        best_profit = float("-inf")
        buckets = {}                       # dict mapping nodes → labels currently stored there
        heap = [(root.time, 0, root)]      # labels waiting to be extended, earliest time first
        counter = 1                        # tie-breaker for the heap; also counts labels kept

        # This loop grabs a label and extends it if feasible (it does so ordered by time)
        while heap:
            # Get first label from the heap based on smallest time
            _, _, label = heapq.heappop(heap)

            # Check if label is actually dead/discarded or if optimistic UB from the label does not improve on best found so far. If so, don't continue with it
            if not label.alive:
                continue
            self.n_bound_checks += 1
            if self._bound(label) <= best_profit:
                self.n_bound_pruned += 1
                continue

            # For each possible extension of the current label
            for child in self._successors(label):
                # Reached the end depot: a complete route. Update the best if it improves; never extend it
                if child.node == self.sink:
                    if child.profit > best_profit:
                        best, best_profit = child, child.profit
                    continue
                # Check pickups for unreachable ones and add to U: what was already unreachable from the parent + what cannot be completed starting from the child label
                self._mark_unreachable(child)
                # Check if deliveries of child can be made starting from that node/label; otherwise discards the child's label
                if not self._rroute_completable(child):
                    continue
                # Check to see if dominated and add to the heap of labels (the to-do list of labels)
                if self._insert(buckets, child):
                    heapq.heappush(heap, (child.time, counter, child))
                    counter += 1

        # Rebuild the route by walking parent pointers back to the root,
        # starting from the end depot's parent so the end depot is not in the route.
        self.n_labels = counter
        route = []
        node = best.parent
        while node.parent is not None:
            route.append(node.node)
            node = node.parent
        route.reverse()

        self.best_profit = best.profit
        self.best_route = route
        if self.verbose:
            print(f"[Labeling] Vehicle {self.k}: profit={best.profit:.4f}, "
                  f"route={route}, labels={counter}")

        return {
            "objVal": best.profit,
            "route": route,
            "served_pickups": [i for i in route if i <= self.n],
            "n_bound_checks": self.n_bound_checks,
            "n_bound_pruned": self.n_bound_pruned,
            "n_eligible": self.n_eligible,
            "n_excluded": self.n_excluded,
        }
