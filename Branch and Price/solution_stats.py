"""
Statistics of an SPDPTW-2C solution, recomputed from its routes.

solution_statistics(instance, routes, reported_profit) re-simulates every route with the same
rules as the solvers: travel time = Euclidean distance, service at node j starts at
max(arrival, e_j − s_j) and must end by min(l_j, l_k); committed couriers pay the block cost
whether used or not and their distance rate on every arc; occasional couriers pay the per-request
fee and their distance rate on every arc except the first one from their start — and returns a
flat dict with the keys in STAT_KEYS.

`routes` maps courier index k (committed first, then occasional) to its node sequence, with
pickups 1..n and deliveries n+1..2n and without depots, as returned by the B&P solver and by
solve_exact_mip. Couriers missing from `routes`, or with an empty route, are idle.

profit_check (= reported − recomputed), tw_violations and capacity_violations should all be 0;
anything else points to an inconsistency between a solver and the model.
"""

import math

STAT_KEYS = [
    # service
    "requests", "served", "served_pct", "served_by_committed", "served_by_occasional",
    "revenue_collected", "revenue_unserved", "avg_rev_served", "avg_rev_unserved",
    # fleet usage
    "committed_used", "committed_total", "occasional_used", "occasional_total",
    "req_per_used_courier_avg", "req_per_used_courier_max",
    "utilization_committed_avg", "utilization_occasional_avg",
    # profit breakdown
    "committed_fixed_cost", "committed_distance_cost", "occasional_order_fees",
    "occasional_distance_cost", "profit_recomputed", "profit_check",
    # route structure
    "distance_committed", "distance_occasional", "stops_per_route_avg", "waiting_time_total",
    "peak_load_ratio_avg", "peak_load_ratio_max", "interleaved_routes",
    # time windows and validation
    "tw_slack_avg", "tw_slack_min", "tw_violations", "capacity_violations",
]


def _mean(values):
    return sum(values) / len(values) if values else None


def solution_statistics(instance, routes, reported_profit=None):
    n = len(instance["requests"])
    req = {r["id"]: r for r in instance["requests"]}
    couriers = instance["committed"] + instance["occasional"]
    n_com = len(instance["committed"])
    w_c = instance["c_com_h"] / instance["b"]
    delta_c, c_o, delta_o = instance["c_com_d"], instance["c_occ_o"], instance["c_occ_d"]
    routes = routes or {}

    def node(j):
        """(location, e, l, service time, load change) of pickup or delivery node j."""
        if j <= n:
            r = req[j]
            return r["pickup"], r["e_pickup"], r["l_pickup"], r["delay_p"], r["l_dem"]
        r = req[j - n]
        return r["dest"], r["e_dest"], r["l_dest"], r["delay_d"], -r["l_dem"]

    served_by = {}                      # request id -> "committed" / "occasional"
    used = {"committed": 0, "occasional": 0}
    util = {"committed": [], "occasional": []}
    dist = {"committed": 0.0, "occasional": 0.0}
    charged = {"committed": 0.0, "occasional": 0.0}
    fixed_cost = 0.0
    req_per_route, stops_per_route, peak_ratios, slacks = [], [], [], []
    waiting = 0.0
    interleaved = tw_violations = capacity_violations = 0

    for k, drv in enumerate(couriers):
        kind = "committed" if k < n_com else "occasional"
        shift = drv["l"] - drv["e"]
        if kind == "committed":
            fixed_cost += w_c * shift
        route = routes.get(k) or []
        if not route:
            continue

        used[kind] += 1
        t, loc = drv["e"], drv["location"]
        load = peak = on_board = max_on_board = 0
        busy = 0.0
        for idx, j in enumerate(route):
            pos, e, l, s, q = node(j)
            d = math.dist(loc, pos)
            arrival = t + d
            start = max(arrival, e - s)
            end = start + s
            waiting += start - arrival
            busy += d + s
            dist[kind] += d
            if kind == "committed" or idx > 0:      # occasional: first leg from the start is free
                charged[kind] += d
            slack = min(l, drv["l"]) - end
            slacks.append(slack)
            if slack < -1e-6:
                tw_violations += 1
            load += q
            peak = max(peak, load)
            on_board += 1 if j <= n else -1
            max_on_board = max(max_on_board, on_board)
            if j <= n:
                served_by[j] = kind
            t, loc = end, pos

        if peak > drv["cap"] + 1e-9:
            capacity_violations += 1
        n_req_route = sum(1 for j in route if j <= n)
        req_per_route.append(n_req_route)
        stops_per_route.append(len(route))
        peak_ratios.append(peak / drv["cap"] if drv["cap"] > 0 else None)
        interleaved += max_on_board >= 2
        util[kind].append(busy / shift if shift > 0 else None)

    served = list(served_by)
    unserved = [i for i in req if i not in served_by]
    rev_served = [req[i]["rev"] for i in served]
    rev_unserved = [req[i]["rev"] for i in unserved]
    n_occ_served = sum(1 for kind in served_by.values() if kind == "occasional")

    revenue = sum(rev_served)
    com_dist_cost = delta_c * charged["committed"]
    occ_fees = c_o * n_occ_served
    occ_dist_cost = delta_o * charged["occasional"]
    profit = revenue - fixed_cost - com_dist_cost - occ_fees - occ_dist_cost

    stats = {
        "requests": n,
        "served": len(served),
        "served_pct": 100.0 * len(served) / n if n else None,
        "served_by_committed": len(served) - n_occ_served,
        "served_by_occasional": n_occ_served,
        "revenue_collected": revenue,
        "revenue_unserved": sum(rev_unserved),
        "avg_rev_served": _mean(rev_served),
        "avg_rev_unserved": _mean(rev_unserved),
        "committed_used": used["committed"],
        "committed_total": n_com,
        "occasional_used": used["occasional"],
        "occasional_total": len(couriers) - n_com,
        "req_per_used_courier_avg": _mean(req_per_route),
        "req_per_used_courier_max": max(req_per_route) if req_per_route else None,
        "utilization_committed_avg": _mean([u for u in util["committed"] if u is not None]),
        "utilization_occasional_avg": _mean([u for u in util["occasional"] if u is not None]),
        "committed_fixed_cost": fixed_cost,
        "committed_distance_cost": com_dist_cost,
        "occasional_order_fees": occ_fees,
        "occasional_distance_cost": occ_dist_cost,
        "profit_recomputed": profit,
        "profit_check": (reported_profit - profit) if reported_profit is not None else None,
        "distance_committed": dist["committed"],
        "distance_occasional": dist["occasional"],
        "stops_per_route_avg": _mean(stops_per_route),
        "waiting_time_total": waiting,
        "peak_load_ratio_avg": _mean([r for r in peak_ratios if r is not None]),
        "peak_load_ratio_max": max((r for r in peak_ratios if r is not None), default=None),
        "interleaved_routes": interleaved,
        "tw_slack_avg": _mean(slacks),
        "tw_slack_min": min(slacks) if slacks else None,
        "tw_violations": tw_violations,
        "capacity_violations": capacity_violations,
    }
    return {key: stats[key] for key in STAT_KEYS}
