"""
Shared preprocessing for the SPDPTW with two courier types: time-window tightening and
courier-request / arc elimination.

preprocess_instance(instance, tighten) returns a dict with
    instance   : deep copy of the instance with (if tighten) tightened request windows
                 (e_pickup, l_pickup, e_dest, l_dest) and courier windows (e, l); every courier
                 keeps its original window in e_orig / l_orig, which costs must use. Requests
                 found unservable keep their original windows; they are excluded through z_UB.
    z_UB       : {(p, k): 0/1}, 1 if courier k can serve request p
    x_UB, x_ind: {(i, j): 0/1} over pickup/delivery arcs; x_ind is x_UB combined with the two
                 pickup-delivery path rules and is the final "arc is possible" flag
    unservable : set of requests no courier can serve
    stats      : time, rounds, window widths, counts

Conventions: nodes 1..n are pickups, n+1..2n deliveries (delivery of p is p + n); couriers are
indexed k = 0.. (committed first); time windows bound the END of service, i.e. service at j
starts at max(arrival, e_j - s_j); travel times are Euclidean distances. With tighten=False only
z_UB and x_ind are computed, on the original windows, so both modes share the same elimination
rules. Every rule only removes times, arcs or pairs that no feasible route can use.
"""

import copy
import math
import time


class _Data:
    """Node and courier data of one instance, indexed as described in the module docstring."""

    def __init__(self, instance):
        self.n = n = len(instance["requests"])
        self.P = list(range(1, n + 1))
        self.D = [p + n for p in self.P]
        self.couriers = instance["committed"] + instance["occasional"]
        self.K = list(range(len(self.couriers)))
        coords, self.e, self.l, self.s = {}, {}, {}, {}
        for req in instance["requests"]:
            p = req["id"]
            coords[p], coords[p + n] = req["pickup"], req["dest"]
            self.e[p], self.l[p], self.s[p] = req["e_pickup"], req["l_pickup"], req["delay_p"]
            self.e[p + n], self.l[p + n], self.s[p + n] = req["e_dest"], req["l_dest"], req["delay_d"]
        self.Ek = {k: c["e"] for k, c in enumerate(self.couriers)}
        self.Lk = {k: c["l"] for k, c in enumerate(self.couriers)}
        self.A2 = [(i, j) for i in self.P + self.D for j in self.P + self.D if i != j]
        self.d = {(i, j): math.dist(coords[i], coords[j]) for (i, j) in self.A2}
        self.d0 = {(k, p): math.dist(c["location"], coords[p])
                   for k, c in enumerate(self.couriers) for p in self.P}

    def request(self, i):
        return i if i <= self.n else i - self.n


def compute_z_UB(g, unservable):
    """
    Courier-request elimination: z_UB[p,k] = 1 if courier k can serve request p on the fastest
    possible route (start -> pickup -> delivery; travel times are Euclidean, so detours only
    make things later).
    """
    z = {}
    for k in g.K:
        for p in g.P:
            d = p + g.n
            pickup_end = max(g.Ek[k] + g.d0[k, p] + g.s[p], g.e[p])          # earliest end of pickup
            delivery_end = max(g.e[d], pickup_end + g.d[p, d] + g.s[d])      # earliest end of delivery
            z[p, k] = (1 if (p not in unservable
                             and g.Lk[k] >= pickup_end                        # pickup within the shift
                             and g.l[p] >= pickup_end                         # pickup before its deadline
                             and min(g.l[d], g.Lk[k]) >= delivery_end)        # delivery before its deadline and the shift end
                       else 0)
    return z


def compute_arc_elimination(g):
    """Arc elimination from the current windows; returns (x_UB, x_ind)."""
    n, P, D = g.n, set(g.P), set(g.D)
    x_UB = {(i, j): 0 if (g.e[i] + g.d[i, j] + g.s[j] > g.l[j]     # j cannot directly follow i in time
                          or (i in D and j == i - n))               # a delivery never precedes its own pickup
                    else 1
            for (i, j) in g.A2}
    # pickup-delivery path rules: arc i -> d needs d's pickup to reach i in time; arc p -> j needs
    # j to reach p's delivery in time (direct is fastest, so failing directly means failing always)
    ipc1 = {(i, d): 0 if (d in D and d - n != i and x_UB[d - n, i] == 0) else 1 for (i, d) in g.A2}
    ipc2 = {(p, j): 0 if (p in P and p + n != j and x_UB[j, p + n] == 0) else 1 for (p, j) in g.A2}
    x_ind = {a: x_UB[a] * ipc1[a] * ipc2[a] for a in g.A2}
    return x_UB, x_ind


def tighten_time_windows(g, arcs, z_UB, tol=1e-2, max_iters=100):
    """
    In-place tightening of the node windows g.e / g.l (Desrochers, Desrosiers & Solomon, 1992),
    adapted to windows on the END of service, plus a pickup-delivery rule.
    arcs: set of (i, j) pickup/delivery arcs that are still possible; they define each node's
          predecessors and successors. Courier starts (before pickups) and the end depot (after
          deliveries) are added for the couriers with z_UB = 1.
    Returns (number of passes, set of requests found unservable): a node that cannot be reached in
    time from any predecessor, cannot be left in time for any successor, or whose pickup and
    delivery windows are incompatible makes its request unservable.
    """
    E, L, S, n = g.e, g.l, g.s, g.n
    preds = {i: [j for (j, m) in arcs if m == i] for i in g.P + g.D}
    succs = {i: [j for (m, j) in arcs if m == i] for i in g.P + g.D}
    unservable = set()

    for ite in range(max_iters):
        oldE, oldL = dict(E), dict(L)
        for i in g.P + g.D:
            req = g.request(i)
            if req in unservable:
                continue
            couriers = [k for k in g.K if z_UB[req, k] == 1]

            # earliest arrival from predecessors: finish a predecessor, travel, serve i
            arriv = [E[j] + g.d[j, i] + S[i] for j in preds[i]]
            if i <= n:
                arriv += [g.Ek[k] + g.d0[k, i] + S[i] for k in couriers]
            min_arriv = min(arriv) if arriv else math.inf
            if min_arriv > L[i] + 1e-9:            # no predecessor reaches i in time
                unservable.add(req)
                continue
            E[i] = max(E[i], min_arriv)

            # earliest departure time to successors: finishing i earlier only adds waiting before
            # every possible next stop (the end depot contributes the courier start times)
            dep = [E[j] - g.d[i, j] - S[j] for j in succs[i]]
            if i > n:
                dep += [g.Ek[k] for k in couriers]
            min_dep = min(dep) if dep else -math.inf
            E[i] = max(E[i], min(L[i], min_dep))

            # latest arrival time from predecessors: i cannot finish later than the latest arrival
            # from any predecessor plus its service
            arriv_l = [L[j] + g.d[j, i] + S[i] for j in preds[i]]
            if i <= n:
                arriv_l += [g.Lk[k] + g.d0[k, i] + S[i] for k in couriers]
            max_arriv = max(arriv_l) if arriv_l else math.inf
            L[i] = min(L[i], max(E[i], max_arriv))

            # latest departure time to successors: i must finish early enough to reach and serve
            # some next stop (deliveries: or the end depot by the shift end)
            dep_l = [L[j] - g.d[i, j] - S[j] for j in succs[i]]
            if i > n:
                dep_l += [g.Lk[k] for k in couriers]
            max_dep = max(dep_l) if dep_l else -math.inf
            if max_dep < E[i] - 1e-9:              # i cannot be left in time for any next stop
                unservable.add(req)
                continue
            L[i] = min(L[i], max_dep)

        # pickup-delivery rule: the delivery follows its pickup at least t_pd + S_d later
        for p in g.P:
            if p in unservable:
                continue
            d = p + n
            L[p] = min(L[p], L[d] - g.d[p, d] - S[d])
            E[d] = max(E[d], E[p] + g.d[p, d] + S[d])
            if L[p] < E[p] - 1e-9 or E[d] > L[d] + 1e-9:
                unservable.add(p)

        if sum(abs(L[i] - oldL[i]) + abs(E[i] - oldE[i]) for i in g.P + g.D) < tol:
            break
    return ite + 1, unservable


def tighten_courier_windows(g, z_UB):
    """Start a courier no earlier than needed for its earliest pickup, end it at its latest delivery."""
    for k in g.K:
        mine = [p for p in g.P if z_UB[p, k] == 1]
        if not mine:
            continue
        # service at p may start at e_p - s_p, so leaving at e_p - s_p - d_kp is still useful
        g.Ek[k] = max(g.Ek[k], min(g.e[p] - g.s[p] - g.d0[k, p] for p in mine))
        g.Lk[k] = min(g.Lk[k], max(g.l[p + g.n] for p in mine))


def preprocess_instance(instance, tighten=True, tol=1e-2, max_iters=100, max_rounds=20):
    """See the module docstring."""
    t0 = time.time()
    g = _Data(instance)
    width_before = sum(g.l[i] - g.e[i] for i in g.P + g.D)
    orig_e, orig_l = dict(g.e), dict(g.l)

    unservable = set()
    z_UB = compute_z_UB(g, unservable)
    unservable |= {p for p in g.P if not any(z_UB[p, k] for k in g.K)}
    x_UB, x_ind = compute_arc_elimination(g)
    rounds = 0
    if tighten:
        # alternate tightening and elimination until nothing changes: tighter windows remove
        # more arcs and pairs, and fewer arcs and pairs allow tighter windows
        for rounds in range(1, max_rounds + 1):
            old = (dict(g.e), dict(g.l), dict(g.Ek), dict(g.Lk), z_UB, x_ind)
            arcs = {(i, j) for (i, j) in g.A2 if x_ind[i, j] == 1
                    and g.request(i) not in unservable and g.request(j) not in unservable}
            _, new_unservable = tighten_time_windows(g, arcs, z_UB, tol, max_iters)
            unservable |= new_unservable
            tighten_courier_windows(g, z_UB)
            z_UB = compute_z_UB(g, unservable)
            unservable |= {p for p in g.P if not any(z_UB[p, k] for k in g.K)}
            x_UB, x_ind = compute_arc_elimination(g)
            change = (sum(abs(g.e[i] - old[0][i]) + abs(g.l[i] - old[1][i]) for i in g.P + g.D)
                      + sum(abs(g.Ek[k] - old[2][k]) + abs(g.Lk[k] - old[3][k]) for k in g.K))
            if change < tol and z_UB == old[4] and x_ind == old[5]:
                break

    # unservable requests keep their original windows (z_UB = 0 excludes them)
    for p in unservable:
        for i in (p, p + g.n):
            g.e[i], g.l[i] = orig_e[i], orig_l[i]

    out = copy.deepcopy(instance)
    for req in out["requests"]:
        p = req["id"]
        req["e_pickup"], req["l_pickup"] = g.e[p], g.l[p]
        req["e_dest"], req["l_dest"] = g.e[p + g.n], g.l[p + g.n]
    for k, c in enumerate(out["committed"] + out["occasional"]):
        c["e_orig"], c["l_orig"] = c["e"], c["l"]
        c["e"], c["l"] = g.Ek[k], g.Lk[k]

    servable = [i for i in g.P + g.D if g.request(i) not in unservable]
    stats = {
        "prep_time": time.time() - t0,
        "prep_rounds": rounds,
        "tw_width_before": width_before,
        "tw_width_after": sum(g.l[i] - g.e[i] for i in g.P + g.D),
        "tw_width_reduction_servable_pct": (
            100.0 * (1 - sum(g.l[i] - g.e[i] for i in servable)
                     / sum(orig_l[i] - orig_e[i] for i in servable))
            if servable and sum(orig_l[i] - orig_e[i] for i in servable) > 0 else 0.0),
        "unservable_requests": len(unservable),
        "zUB_pairs": sum(z_UB.values()),
        "arcs_kept": sum(x_ind.values()),
    }
    return {"instance": out, "z_UB": z_UB, "x_UB": x_UB, "x_ind": x_ind,
            "unservable": unservable, "stats": stats}
